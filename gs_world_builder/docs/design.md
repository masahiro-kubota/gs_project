# gs_world_builder 実装設計

**バージョン**: 1.0.0
**最終更新**: 2026-02-14

## 概要

このドキュメントは、`gs_world_builder` の**実装レベルの設計**（クラス構成、関数シグネチャ）を定義します。

**ドキュメントの関係**:
- **[architecture.md](architecture.md)** - 各コンポーネントの責務・処理内容
- **[interface.md](interface.md)** - コンポーネント間のデータ契約
- **このドキュメント** - 実装設計（クラス・関数）

**言語**: Python 3.10+（型ヒント必須）

---

## 設計原則

1. **型安全性**: すべての関数に型ヒントを付ける
2. **不変性**: Config 系は dataclass (frozen=True)
3. **明示的エラー**: カスタム例外で詳細なエラー情報
4. **テスタビリティ**: 依存注入、純粋関数を優先
5. **プログレス報告**: tqdm または Progress で進捗表示
6. **ロギング**: 標準 logging モジュール使用

---

## 共通データクラス

### Config データクラス

```python
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional

@dataclass(frozen=True)
class PipelineConfig:
    """パイプライン全体の設定"""

    # 入出力
    input_mcap: Path
    output_world_bundle: Path
    scene_id: str
    workspace_dir: Path

    # トピックマッピング
    topic_mapping: 'TopicMapping'

    # フレームID
    frame_ids: 'FrameIds'

    # 処理範囲
    time_range: Optional[tuple[float, float]] = None  # (start, end) [s]

    # 座標系原点
    origin: Optional['OriginConfig'] = None

    # パイプライン有効化
    enable_gs_training: bool = True
    enable_static_mesh: bool = False

@dataclass(frozen=True)
class TopicMapping:
    """トピック名マッピング"""
    cameras: Dict[str, str]  # {camera_id: topic_name}
    lidars: Dict[str, str]   # {lidar_id: topic_name}
    odom: str
    tf: str = "/tf"
    tf_static: str = "/tf_static"

@dataclass(frozen=True)
class FrameIds:
    """フレームID定義"""
    map: str = "map"
    odom: str = "odom"
    base_link: str = "base_link"
    cameras: Dict[str, str] = None  # {camera_id: frame_id}
    lidars: Dict[str, str] = None   # {lidar_id: frame_id}

@dataclass(frozen=True)
class OriginConfig:
    """座標系原点"""
    latitude: float   # [deg]
    longitude: float  # [deg]
    altitude: float   # [m]

@dataclass(frozen=True)
class GSTrainingConfig:
    """3DGS 学習設定（gs_training_config.yaml から読み込み）"""
    seed: int
    max_iterations: int
    mode: str  # "background_only"
    # その他 DriveStudio 固有の設定
    raw_config: Dict[str, Any]  # 元のYAML全体を保持

@dataclass(frozen=True)
class GeometryConfig:
    """geometry 生成設定（geometry_config.yaml から読み込み）"""
    heightmap_resolution: float  # [m/cell]
    heightmap_buffer: float      # [m]
    drivable_buffer_width: float # [m]
    voxel_size: float            # [m] for mesh
    mesh_method: str             # "poisson" or "tsdf"
```

---

### 設定ファイル読み込み

```python
import yaml
from pathlib import Path

def load_pipeline_config(
    config_file: Path,
    gs_config_file: Path,
    geometry_config_file: Path,
    input_mcap: Path,
    output_dir: Path,
    scene_id: str,
    workers: Optional[int] = None
) -> PipelineConfig:
    """3つの設定ファイルを読み込んで PipelineConfig を構築

    Args:
        config_file: パイプライン全体設定（config.yaml）
        gs_config_file: 3DGS学習設定（gs_training_config.yaml）
        geometry_config_file: 幾何生成設定（geometry_config.yaml）
        input_mcap: 入力MCAPファイルパス
        output_dir: 出力ディレクトリ
        scene_id: シーンID
        workers: 並列処理数（Noneの場合はCPU数）

    Returns:
        PipelineConfig インスタンス

    Raises:
        FileNotFoundError: 設定ファイルが見つからない
        yaml.YAMLError: YAML パースエラー
    """
    # 1. config.yaml 読み込み
    with open(config_file) as f:
        config_data = yaml.safe_load(f)

    # 2. gs_training_config.yaml 読み込み
    with open(gs_config_file) as f:
        gs_config_data = yaml.safe_load(f)

    # 3. geometry_config.yaml 読み込み
    with open(geometry_config_file) as f:
        geometry_config_data = yaml.safe_load(f)

    # TopicMapping 構築
    topics = config_data.get('topics', {})
    topic_mapping = TopicMapping(
        cameras=topics.get('cameras', {}),
        lidars=topics.get('lidars', {}),
        odom=topics.get('odom', '/odom'),
        tf=topics.get('tf', '/tf'),
        tf_static=topics.get('tf_static', '/tf_static')
    )

    # FrameIds 構築
    frames = config_data.get('frames', {})
    frame_ids = FrameIds(
        map=frames.get('map', 'map'),
        odom=frames.get('odom', 'odom'),
        base_link=frames.get('base_link', 'base_link'),
        cameras=frames.get('cameras', {}),
        lidars=frames.get('lidars', {})
    )

    # OriginConfig 構築（オプション）
    origin_data = config_data.get('origin')
    origin = None
    if origin_data:
        origin = OriginConfig(
            latitude=origin_data['latitude'],
            longitude=origin_data['longitude'],
            altitude=origin_data['altitude']
        )

    # 時間範囲（オプション）
    time_range = None
    if 'processing' in config_data and 'time_range' in config_data['processing']:
        tr = config_data['processing']['time_range']
        time_range = (tr['start'], tr['end'])

    # パイプライン有効化フラグ
    pipeline = config_data.get('pipeline', {})
    enable_gs_training = pipeline.get('enable_gs_training', True)
    enable_static_mesh = pipeline.get('enable_static_mesh', False)

    # Workspace 決定
    workspace_dir = Path(f".gs_world_builder_workspace/{scene_id}")

    # Workers 数決定
    if workers is None:
        import os
        workers = os.cpu_count()

    return PipelineConfig(
        input_mcap=input_mcap,
        output_world_bundle=output_dir,
        scene_id=scene_id,
        workspace_dir=workspace_dir,
        topic_mapping=topic_mapping,
        frame_ids=frame_ids,
        time_range=time_range,
        origin=origin,
        enable_gs_training=enable_gs_training,
        enable_static_mesh=enable_static_mesh
    )
```

---

### Result データクラス

```python
from dataclasses import dataclass
from typing import List, Optional
from enum import Enum

class ProcessStatus(Enum):
    """処理ステータス"""
    SUCCESS = "success"
    FAILED = "failed"
    SKIPPED = "skipped"

@dataclass
class ComponentResult:
    """コンポーネント実行結果"""
    component_name: str
    status: ProcessStatus
    processing_time_sec: float
    output_files: List[Path]
    warnings: List[str]
    errors: List[str]
    metadata: Dict[str, Any]  # コンポーネント固有の情報

@dataclass
class PipelineResult:
    """パイプライン全体の結果"""
    scene_id: str
    status: ProcessStatus
    total_time_sec: float
    component_results: List[ComponentResult]
    world_bundle_path: Optional[Path]
    build_report_path: Path

    def get_component_result(self, name: str) -> Optional[ComponentResult]:
        """コンポーネント結果を取得"""
        for result in self.component_results:
            if result.component_name == name:
                return result
        return None
```

---

## 共通例外クラス

```python
from typing import Optional, Dict, Any

class WorldBuilderError(Exception):
    """gs_world_builder の基底例外

    エラーメッセージ形式（JSON）をサポート:
    {
      "error": {
        "code": "ERROR_CODE",
        "component": "component_name",
        "message": "...",
        "details": {...},
        "suggestion": "..."
      }
    }
    """

    def __init__(
        self,
        code: str,
        message: str,
        component: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None,
        suggestion: Optional[str] = None
    ):
        self.code = code
        self.component = component
        self.message = message
        self.details = details or {}
        self.suggestion = suggestion
        super().__init__(message)

    def to_dict(self) -> Dict[str, Any]:
        """エラー情報を辞書形式で取得"""
        error_dict = {
            "code": self.code,
            "message": self.message,
            "details": self.details
        }
        if self.component:
            error_dict["component"] = self.component
        if self.suggestion:
            error_dict["suggestion"] = self.suggestion
        return {"error": error_dict}

class McapReadError(WorldBuilderError):
    """MCAP 読み込みエラー"""

    def __init__(self, mcap_path: str, reason: str):
        super().__init__(
            code="MCAP_READ_ERROR",
            message=f"Failed to read MCAP file: {mcap_path}. Reason: {reason}",
            component="mcap_ingest",
            details={"mcap_path": mcap_path, "reason": reason},
            suggestion="Check if the MCAP file exists and is not corrupted."
        )

class TopicNotFoundError(WorldBuilderError):
    """必須トピックが見つからない"""

    def __init__(self, topic: str, available: List[str]):
        super().__init__(
            code="TOPIC_NOT_FOUND",
            message=f"Required topic '{topic}' not found in MCAP",
            component="mcap_ingest",
            details={"topic": topic, "available_topics": available},
            suggestion="Check topic mapping in config.yaml. Available topics are listed in 'details'."
        )

class InsufficientDataError(WorldBuilderError):
    """データ量不足"""

    def __init__(self, data_type: str, actual: float, required: float, unit: str):
        super().__init__(
            code="INSUFFICIENT_DATA",
            message=f"Insufficient {data_type}: {actual}{unit} < {required}{unit}",
            component="mcap_ingest",
            details={"data_type": data_type, "actual": actual, "required": required, "unit": unit},
            suggestion=f"Ensure the MCAP file contains at least {required}{unit} of {data_type}."
        )

class ValidationError(WorldBuilderError):
    """検証エラー"""

    def __init__(self, validation_name: str, details: Dict[str, Any]):
        super().__init__(
            code="VALIDATION_FAILED",
            message=f"Validation '{validation_name}' failed",
            component="world_bundle_packer",
            details=details,
            suggestion="Fix the issues listed in 'details' and rebuild."
        )

class CalibrationError(WorldBuilderError):
    """キャリブレーションエラー"""

    def __init__(self, reason: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(
            code="CALIBRATION_ERROR",
            message=f"Calibration failed: {reason}",
            component="calibration_builder",
            details=details or {},
            suggestion="Check TF information and camera_info in the MCAP file."
        )

class TrainingError(WorldBuilderError):
    """3DGS 学習エラー"""

    def __init__(self, reason: str):
        super().__init__(
            code="TRAINING_FAILED",
            message=f"3DGS training failed: {reason}",
            component="gs_train_orchestrator",
            details={"reason": reason},
            suggestion="Check GPU availability and training configuration. See training log for details."
        )
```

---

## 1. mcap_ingest

### クラス構成

```python
from abc import ABC, abstractmethod
from typing import Protocol

class McapIngestor:
    """MCAP 取り込みメインクラス"""

    def __init__(
        self,
        config: PipelineConfig,
        workspace: Path,
        logger: logging.Logger
    ):
        self.config = config
        self.workspace = workspace
        self.logger = logger

        # サブコンポーネント
        self.topic_mapper = TopicMapper(config.topic_mapping)
        self.image_extractor = ImageExtractor()
        self.lidar_extractor = LidarExtractor()
        self.tf_extractor = TFExtractor()
        self.pose_extractor = PoseExtractor(config.frame_ids)

    def ingest(self) -> ComponentResult:
        """MCAP 取り込み実行"""
        ...

class TopicMapper:
    """トピック名解決"""

    def __init__(self, mapping: TopicMapping):
        self.mapping = mapping

    def get_camera_topic(self, camera_id: str) -> str:
        """カメラトピック取得"""
        if camera_id not in self.mapping.cameras:
            raise TopicNotFoundError(
                camera_id, list(self.mapping.cameras.keys())
            )
        return self.mapping.cameras[camera_id]

    def get_lidar_topic(self, lidar_id: str) -> str:
        """LiDAR トピック取得"""
        ...

class ImageExtractor:
    """カメラ画像抽出"""

    def extract(
        self,
        mcap_reader: McapReader,
        topic: str,
        output_dir: Path
    ) -> List[ImageFrame]:
        """画像フレーム抽出

        Args:
            mcap_reader: MCAP リーダー
            topic: トピック名
            output_dir: 出力ディレクトリ

        Returns:
            抽出したフレーム情報

        Raises:
            TopicNotFoundError: トピックが見つからない
        """
        ...

class LidarExtractor:
    """LiDAR 点群抽出"""

    def extract(
        self,
        mcap_reader: McapReader,
        topic: str,
        output_dir: Path,
        format: str = "pcd"  # "pcd" or "npz"
    ) -> List[LidarFrame]:
        """点群フレーム抽出"""
        ...

class TFExtractor:
    """TF 情報抽出"""

    def extract(
        self,
        mcap_reader: McapReader
    ) -> tuple[List[TFTransform], List[TFTransform]]:
        """TF と TF_static を抽出

        Returns:
            (tf_transforms, tf_static_transforms)
        """
        ...

    def save_as_json(
        self,
        transforms: List[TFTransform],
        output_path: Path
    ):
        """JSON 形式で保存"""
        ...

class PoseExtractor:
    """Ego 軌跡抽出"""

    def __init__(self, frame_ids: FrameIds):
        self.frame_ids = frame_ids

    def extract(
        self,
        mcap_reader: McapReader,
        odom_topic: str,
        tf_transforms: List[TFTransform]
    ) -> pd.DataFrame:
        """Ego 軌跡を抽出し DataFrame で返す

        Returns:
            DataFrame with columns: timestamp, x, y, z, qx, qy, qz, qw
        """
        ...

    def save_as_csv(self, poses: pd.DataFrame, output_path: Path):
        """CSV 保存"""
        ...
```

### データクラス

```python
@dataclass
class ImageFrame:
    """画像フレーム情報"""
    timestamp: float
    frame_index: int
    file_path: Path
    width: int
    height: int
    encoding: str

@dataclass
class LidarFrame:
    """LiDAR フレーム情報"""
    timestamp: float
    frame_index: int
    file_path: Path
    point_count: int

@dataclass
class TFTransform:
    """TF 変換"""
    timestamp: Optional[float]  # tf_static の場合は None
    parent_frame: str
    child_frame: str
    translation: tuple[float, float, float]  # (x, y, z)
    rotation: tuple[float, float, float, float]  # (qx, qy, qz, qw)
```

---

## 2. calibration_builder

### クラス構成

```python
class CalibrationBuilder:
    """キャリブレーション構築"""

    def __init__(
        self,
        config: PipelineConfig,
        workspace: Path,
        logger: logging.Logger
    ):
        self.config = config
        self.workspace = workspace
        self.logger = logger

    def build(
        self,
        tf_static: List[TFTransform],
        camera_infos: Dict[str, CameraInfo]
    ) -> ComponentResult:
        """キャリブレーション構築実行

        Args:
            tf_static: 静的TF情報（mcap_ingest から）
            camera_infos: カメラ情報（MCAP から抽出）

        Returns:
            実行結果
        """
        ...

    def build_calibration_yaml(
        self,
        tf_static: List[TFTransform],
        camera_infos: Dict[str, CameraInfo]
    ) -> CalibrationData:
        """calibration.yaml データ構築"""
        ...

    def derive_tf_static_json(
        self,
        calibration: CalibrationData
    ) -> List[TFTransform]:
        """calibration.yaml から tf_static.json を派生"""
        ...

    def validate_consistency(
        self,
        calibration: CalibrationData,
        tf_static: List[TFTransform]
    ) -> bool:
        """整合性検証

        Raises:
            CalibrationError: 不一致がある場合
        """
        ...

@dataclass
class CameraInfo:
    """カメラ情報（MCAP から抽出）"""
    frame_id: str
    width: int
    height: int
    K: np.ndarray  # 3x3 intrinsic matrix
    D: np.ndarray  # distortion coefficients
    distortion_model: str  # "plumb_bob" or "fisheye"

@dataclass
class CalibrationData:
    """calibration.yaml の内部表現"""
    version: str
    cameras: Dict[str, CameraCalibration]
    lidars: Dict[str, LidarCalibration]

@dataclass
class CameraCalibration:
    """カメラキャリブレーション"""
    frame_id: str
    image_width: int
    image_height: int
    camera_convention: str  # "opencv" or "ros"
    intrinsics: CameraIntrinsics
    extrinsics: Extrinsics
    rate_hz: float

@dataclass
class CameraIntrinsics:
    """カメラ内部パラメータ"""
    model: str  # "pinhole" or "fisheye"
    fx: float
    fy: float
    cx: float
    cy: float
    distortion_model: str
    k1: float
    k2: float
    p1: float
    p2: float

@dataclass
class Extrinsics:
    """外部パラメータ (base_link → sensor)"""
    translation: tuple[float, float, float]
    rotation_quat: tuple[float, float, float, float]  # [x,y,z,w]

@dataclass
class LidarCalibration:
    """LiDAR キャリブレーション"""
    frame_id: str
    extrinsics: Extrinsics
    spec: LidarSpec
    rate_hz: float

@dataclass
class LidarSpec:
    """LiDAR 仕様"""
    model: str
    channels: int
    horizontal_resolution: float  # [deg]
    vertical_fov: tuple[float, float]  # [min, max] [deg]
    max_range: float  # [m]
    min_range: float  # [m]
```

---

## 3. drivesim_dataset_converter

### クラス構成

```python
class DriveSimDatasetConverter:
    """DriveStudio データセット変換"""

    def __init__(
        self,
        config: PipelineConfig,
        workspace: Path,
        logger: logging.Logger
    ):
        self.config = config
        self.workspace = workspace
        self.logger = logger

    def convert(
        self,
        raw_extract_dir: Path,
        calibration: CalibrationData,
        frameset: Frameset
    ) -> ComponentResult:
        """DriveStudio 形式に変換

        Args:
            raw_extract_dir: mcap_ingest の出力ディレクトリ
            calibration: キャリブレーションデータ
            frameset: フレームセット情報
        """
        ...

    def convert_images(self, ...):
        """画像をコピー・配置"""
        ...

    def convert_poses(self, ...):
        """カメラ pose を DriveStudio 形式に変換"""
        ...

    def convert_intrinsics(self, calibration: CalibrationData) -> Dict:
        """intrinsics.json 生成"""
        ...

    def create_dataset_config(self, ...) -> Dict:
        """dataset_config.json 生成"""
        ...

@dataclass
class Frameset:
    """frameset.json の内部表現"""
    version: str
    target_rates: Dict[str, float]  # {"camera": 12.0, "lidar": 20.0}
    frames: List[FramesetFrame]

@dataclass
class FramesetFrame:
    """1フレーム分の情報"""
    sim_time: float
    sensors: Dict[str, SensorFrameInfo]  # {sensor_id: info}

@dataclass
class SensorFrameInfo:
    """センサフレーム情報"""
    original_timestamp: float
    frame_index: int
    file_path: Path
```

---

## 4. gs_train_orchestrator

### クラス構成

```python
class GSTrainOrchestrator:
    """3DGS 学習オーケストレーター"""

    def __init__(
        self,
        config: PipelineConfig,
        gs_config_path: Path,
        workspace: Path,
        logger: logging.Logger
    ):
        self.config = config
        self.gs_config = self._load_gs_config(gs_config_path)
        self.workspace = workspace
        self.logger = logger

    def train(
        self,
        dataset_dir: Path
    ) -> ComponentResult:
        """3DGS 学習実行

        Args:
            dataset_dir: DriveStudio データセットディレクトリ

        Returns:
            学習結果（最良iterationを含む）
        """
        ...

    def _run_drivesim_training(
        self,
        dataset_dir: Path,
        output_dir: Path
    ) -> subprocess.CompletedProcess:
        """DriveStudio の学習スクリプトを実行"""
        ...

    def _monitor_training(self, log_file: Path) -> TrainingMetrics:
        """学習ログを監視し、メトリクスを取得"""
        ...

    def _find_best_iteration(
        self,
        metrics: TrainingMetrics
    ) -> int:
        """最良iteration を特定（PSNR基準）"""
        ...

    def _create_training_report(
        self,
        metrics: TrainingMetrics,
        best_iter: int
    ) -> TrainingReport:
        """training_report.json 生成"""
        ...

@dataclass
class GSTrainingConfig:
    """3DGS 学習設定"""
    seed: int
    max_iterations: int
    mode: str  # "background_only"
    drivesim_commit: str
    # その他 DriveStudio 固有の設定

@dataclass
class TrainingMetrics:
    """学習メトリクス"""
    iterations: List[IterationMetric]

@dataclass
class IterationMetric:
    """各イテレーションのメトリクス"""
    iteration: int
    psnr: float
    loss: float
    timestamp: float

@dataclass
class TrainingReport:
    """training_report.json の内部表現"""
    version: str
    config: GSTrainingConfig
    training_time_sec: float
    best_iteration: int
    best_psnr: float
    iterations: List[IterationMetric]
```

---

## 5. gs_exporter

### クラス構成

```python
class GSExporter:
    """Gaussian エクスポーター"""

    def __init__(
        self,
        workspace: Path,
        logger: logging.Logger
    ):
        self.workspace = workspace
        self.logger = logger

    def export(
        self,
        checkpoint_dir: Path,
        training_report: TrainingReport
    ) -> ComponentResult:
        """ランタイム用 Gaussian をエクスポート

        Args:
            checkpoint_dir: checkpoints/ ディレクトリ
            training_report: 学習レポート

        Returns:
            エクスポート結果
        """
        ...

    def load_checkpoint(
        self,
        checkpoint_dir: Path,
        iteration: int
    ) -> GaussianData:
        """DriveStudio checkpoint から Gaussian データを読み込み"""
        ...

    def export_to_ply(
        self,
        gaussian: GaussianData,
        output_path: Path
    ):
        """PLY 形式でエクスポート

        ヘッダ: format binary_little_endian 1.0
        必須属性: x,y,z, scale_0-2, rot_0-3, opacity, f_dc_0-2, f_rest_*
        """
        ...

    def create_render_config(
        self,
        gaussian: GaussianData
    ) -> RenderConfig:
        """render_config.json 生成"""
        ...

@dataclass
class GaussianData:
    """Gaussian データの内部表現"""
    positions: np.ndarray  # [N, 3] float32
    scales: np.ndarray     # [N, 3] float32
    rotations: np.ndarray  # [N, 4] float32, quaternion [x,y,z,w]
    opacities: np.ndarray  # [N, 1] float32
    sh_dc: np.ndarray      # [N, 3] float32, DC component [R,G,B]
    sh_rest: np.ndarray    # [N, (deg+1)^2-1, 3] float32
    sh_degree: int

@dataclass
class RenderConfig:
    """render_config.json の内部表現"""
    version: str
    gaussian_format: str
    sh_degree: int
    color_correction: ColorCorrection
    rendering: RenderingSettings

@dataclass
class ColorCorrection:
    """色補正設定"""
    white_balance: tuple[float, float, float]
    exposure_compensation: float
    gamma: float

@dataclass
class RenderingSettings:
    """レンダリング設定"""
    background_color: tuple[float, float, float]
    near_plane: float
    far_plane: float
```

---

## 6. geometry_builder

### 6-1. pointcloud_integrator

```python
class PointCloudIntegrator:
    """点群統合"""

    def __init__(
        self,
        workspace: Path,
        logger: logging.Logger
    ):
        self.workspace = workspace
        self.logger = logger

    def integrate(
        self,
        lidar_frames_dir: Path,
        poses_csv: Path
    ) -> ComponentResult:
        """点群統合実行

        Args:
            lidar_frames_dir: フレーム単位の点群ディレクトリ
            poses_csv: Ego軌跡

        Returns:
            統合結果（point_cloud_map.pcd）
        """
        ...

    def load_frame(self, frame_path: Path) -> o3d.geometry.PointCloud:
        """1フレームの点群を読み込み"""
        ...

    def transform_to_map(
        self,
        cloud: o3d.geometry.PointCloud,
        pose: Pose
    ) -> o3d.geometry.PointCloud:
        """base_link → map frame 変換"""
        ...

    def filter_dynamic_objects(
        self,
        cloud: o3d.geometry.PointCloud
    ) -> o3d.geometry.PointCloud:
        """動的物体フィルタリング（簡易版: voxel grid）"""
        ...

@dataclass
class Pose:
    """姿勢"""
    timestamp: float
    position: np.ndarray  # [3]
    orientation: np.ndarray  # [4] quaternion [x,y,z,w]
```

---

### 6-2. ground_surface_builder

```python
class GroundSurfaceBuilder:
    """地面サーフェス構築"""

    def __init__(
        self,
        geometry_config: GeometryConfig,
        workspace: Path,
        logger: logging.Logger
    ):
        self.config = geometry_config
        self.workspace = workspace
        self.logger = logger

    def build(
        self,
        point_cloud_path: Path,
        poses_csv: Path
    ) -> ComponentResult:
        """heightmap 生成

        Args:
            point_cloud_path: 統合点群
            poses_csv: Ego軌跡（範囲決定用）
        """
        ...

    def determine_grid_bounds(
        self,
        poses: pd.DataFrame,
        buffer: float
    ) -> GridBounds:
        """グリッド範囲決定（Ego軌跡 ± buffer）"""
        ...

    def project_to_grid(
        self,
        cloud: o3d.geometry.PointCloud,
        bounds: GridBounds,
        resolution: float
    ) -> np.ndarray:
        """点群を2.5Dグリッドに投影

        Returns:
            heightmap: [height, width] float32, NaN for invalid
        """
        ...

    def apply_filters(
        self,
        heightmap: np.ndarray
    ) -> np.ndarray:
        """メディアンフィルタ・外れ値除去"""
        ...

    def save_heightmap(
        self,
        heightmap: np.ndarray,
        bounds: GridBounds,
        resolution: float,
        output_bin: Path,
        output_yaml: Path
    ):
        """heightmap.bin と heightmap.yaml を保存"""
        ...

@dataclass
class GeometryConfig:
    """geometry 生成設定"""
    heightmap_resolution: float  # [m/cell]
    heightmap_buffer: float      # [m]
    drivable_buffer_width: float # [m]
    voxel_size: float            # [m] for mesh
    mesh_method: str             # "poisson" or "tsdf"

@dataclass
class GridBounds:
    """グリッド範囲"""
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    origin_x: float
    origin_y: float
    origin_z: float
    width: int   # X方向セル数
    height: int  # Y方向セル数
```

---

### 6-3. drivable_area_builder

```python
from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union

class DrivableAreaBuilder:
    """走行可能領域構築"""

    def __init__(
        self,
        geometry_config: GeometryConfig,
        workspace: Path,
        logger: logging.Logger
    ):
        self.config = geometry_config
        self.workspace = workspace
        self.logger = logger

    def build(
        self,
        poses_csv: Path,
        hdmap_path: Optional[Path] = None
    ) -> ComponentResult:
        """走行可能領域生成

        Args:
            poses_csv: Ego軌跡
            hdmap_path: HDMap (Lanelet2等、オプション)
        """
        ...

    def create_buffer_polygon(
        self,
        poses: pd.DataFrame,
        buffer_width: float
    ) -> Polygon:
        """軌跡にバッファを設定してポリゴン生成"""
        line = LineString(poses[['x', 'y']].values)
        return line.buffer(buffer_width)

    def simplify_polygon(
        self,
        polygon: Polygon,
        tolerance: float = 0.5
    ) -> Polygon:
        """ポリゴン簡略化（Douglas-Peucker）"""
        return polygon.simplify(tolerance)

    def save_as_geojson(
        self,
        polygon: Polygon,
        output_path: Path
    ):
        """GeoJSON 形式で保存"""
        ...
```

---

### 6-4. static_mesh_builder

```python
class StaticMeshBuilder:
    """静的メッシュ構築（オプション）"""

    def __init__(
        self,
        geometry_config: GeometryConfig,
        workspace: Path,
        logger: logging.Logger
    ):
        self.config = geometry_config
        self.workspace = workspace
        self.logger = logger

    def build(
        self,
        point_cloud_path: Path
    ) -> ComponentResult:
        """3Dメッシュ生成

        Args:
            point_cloud_path: 統合点群
        """
        ...

    def reconstruct_mesh(
        self,
        cloud: o3d.geometry.PointCloud,
        method: str
    ) -> o3d.geometry.TriangleMesh:
        """メッシュ再構成

        Args:
            method: "poisson" or "tsdf"
        """
        if method == "poisson":
            return self._poisson_reconstruction(cloud)
        elif method == "tsdf":
            return self._tsdf_reconstruction(cloud)
        else:
            raise ValueError(f"Unknown method: {method}")

    def _poisson_reconstruction(
        self,
        cloud: o3d.geometry.PointCloud
    ) -> o3d.geometry.TriangleMesh:
        """Poisson Surface Reconstruction"""
        ...

    def _tsdf_reconstruction(
        self,
        cloud: o3d.geometry.PointCloud
    ) -> o3d.geometry.TriangleMesh:
        """TSDF Reconstruction"""
        ...

    def simplify_mesh(
        self,
        mesh: o3d.geometry.TriangleMesh,
        target_triangles: int
    ) -> o3d.geometry.TriangleMesh:
        """メッシュ簡略化"""
        ...

    def save_as_glb(
        self,
        mesh: o3d.geometry.TriangleMesh,
        output_path: Path
    ):
        """glTF 2.0 Binary 形式で保存"""
        ...
```

---

## 7. world_bundle_packer

### クラス構成

```python
class WorldBundlePacker:
    """world_bundle パッカー"""

    def __init__(
        self,
        config: PipelineConfig,
        workspace: Path,
        logger: logging.Logger
    ):
        self.config = config
        self.workspace = workspace
        self.logger = logger
        self.validator = WorldBundleValidator()

    def pack(
        self,
        component_results: List[ComponentResult]
    ) -> ComponentResult:
        """world_bundle 生成

        Args:
            component_results: すべてのコンポーネント結果

        Returns:
            パッキング結果
        """
        ...

    def create_directory_structure(self, output_dir: Path):
        """ディレクトリ構造作成"""
        ...

    def copy_files(self, ...):
        """各ファイルを適切な場所にコピー"""
        ...

    def generate_world_yaml(self, output_dir: Path) -> Path:
        """world.yaml 生成"""
        ...

    def finalize_metadata_json(
        self,
        partial_metadata: Dict,
        component_results: List[ComponentResult],
        output_path: Path
    ) -> Path:
        """metadata.json 最終化"""
        ...

    def validate(self, world_bundle_dir: Path) -> ValidationResult:
        """world_bundle 検証"""
        return self.validator.validate(world_bundle_dir)

    def create_build_report(
        self,
        component_results: List[ComponentResult],
        validation_result: ValidationResult,
        output_path: Path
    ) -> Path:
        """build_report.json 生成"""
        ...

class WorldBundleValidator:
    """world_bundle 検証器"""

    def validate(self, world_bundle_dir: Path) -> ValidationResult:
        """検証実行"""
        result = ValidationResult()

        # 1. 必須ファイル存在確認
        result.add_check(self.check_required_files(world_bundle_dir))

        # 2. world.yaml スキーマ準拠
        result.add_check(self.check_world_yaml_schema(world_bundle_dir))

        # 3. calibration.yaml と tf_static.json 一貫性
        result.add_check(self.check_calibration_consistency(world_bundle_dir))

        # 4. heightmap サイズ整合性
        result.add_check(self.check_heightmap_size(world_bundle_dir))

        # 5. drivable.geojson 妥当性
        result.add_check(self.check_drivable_geojson(world_bundle_dir))

        # 6. Gaussian データ読み込み可能性
        result.add_check(self.check_gaussian_loadable(world_bundle_dir))

        return result

    def check_required_files(self, world_bundle_dir: Path) -> CheckResult:
        """必須ファイル存在確認"""
        ...

    def check_calibration_consistency(
        self,
        world_bundle_dir: Path
    ) -> CheckResult:
        """calibration.yaml と tf_static.json の一貫性確認"""
        ...

    # ... 他の検証メソッド

@dataclass
class ValidationResult:
    """検証結果"""
    checks: List[CheckResult]

    @property
    def is_valid(self) -> bool:
        return all(check.passed for check in self.checks)

    @property
    def errors(self) -> List[str]:
        return [check.message for check in self.checks if not check.passed]

@dataclass
class CheckResult:
    """個別チェック結果"""
    check_name: str
    passed: bool
    message: str
```

---

## パイプライン実行オーケストレーション

### メインパイプライン

```python
class WorldBuilderPipeline:
    """パイプライン全体のオーケストレーター"""

    def __init__(self, config: PipelineConfig):
        self.config = config
        self.logger = self._setup_logger()
        self.workspace = self._setup_workspace()

    def run(self) -> PipelineResult:
        """パイプライン全体を実行"""
        start_time = time.time()
        results: List[ComponentResult] = []

        try:
            # 1. mcap_ingest
            result = self._run_mcap_ingest()
            results.append(result)
            if result.status != ProcessStatus.SUCCESS:
                return self._create_failed_result(results)

            # 2. calibration_builder
            result = self._run_calibration_builder()
            results.append(result)
            if result.status != ProcessStatus.SUCCESS:
                return self._create_failed_result(results)

            # 3. drivesim_dataset_converter
            result = self._run_drivesim_converter()
            results.append(result)
            if result.status != ProcessStatus.SUCCESS:
                return self._create_failed_result(results)

            # 4. gs_train_orchestrator
            if self.config.enable_gs_training:
                result = self._run_gs_training()
                results.append(result)
                if result.status != ProcessStatus.SUCCESS:
                    return self._create_failed_result(results)
            else:
                results.append(self._create_skipped_result("gs_train_orchestrator"))

            # 5. gs_exporter
            result = self._run_gs_exporter()
            results.append(result)
            if result.status != ProcessStatus.SUCCESS:
                return self._create_failed_result(results)

            # 6. geometry_builder (4つのサブコンポーネント)
            result = self._run_geometry_builder()
            results.append(result)
            if result.status != ProcessStatus.SUCCESS:
                return self._create_failed_result(results)

            # 7. world_bundle_packer
            result = self._run_world_bundle_packer(results)
            results.append(result)
            if result.status != ProcessStatus.SUCCESS:
                return self._create_failed_result(results)

            total_time = time.time() - start_time

            return PipelineResult(
                scene_id=self.config.scene_id,
                status=ProcessStatus.SUCCESS,
                total_time_sec=total_time,
                component_results=results,
                world_bundle_path=self.config.output_world_bundle,
                build_report_path=self.workspace / "build_report.json"
            )

        except Exception as e:
            self.logger.exception("Pipeline failed with exception")
            total_time = time.time() - start_time
            return PipelineResult(
                scene_id=self.config.scene_id,
                status=ProcessStatus.FAILED,
                total_time_sec=total_time,
                component_results=results,
                world_bundle_path=None,
                build_report_path=self.workspace / "build_report.json"
            )

    def _run_mcap_ingest(self) -> ComponentResult:
        """mcap_ingest 実行"""
        ingestor = McapIngestor(
            config=self.config,
            workspace=self.workspace / "00_mcap_ingest",
            logger=self.logger
        )
        return ingestor.ingest()

    def _run_calibration_builder(self) -> ComponentResult:
        """calibration_builder 実行"""
        ...

    # ... 他のコンポーネント実行メソッド

    def _setup_logger(self) -> logging.Logger:
        """ロガー設定"""
        ...

    def _setup_workspace(self) -> Path:
        """ワークスペース準備"""
        workspace = Path(f".gs_world_builder_workspace/{self.config.scene_id}")
        workspace.mkdir(parents=True, exist_ok=True)
        return workspace
```

---

## CLI インターフェース

```python
import click
from pathlib import Path

@click.group()
def cli():
    """gs-world-builder CLI"""
    pass

@cli.command()
@click.option('--input', type=click.Path(exists=True),
              required=True, help='Input MCAP file')
@click.option('--output', type=click.Path(),
              required=True, help='Output world_bundle directory')
@click.option('--config', type=click.Path(exists=True),
              default='config.yaml', help='Pipeline configuration file')
@click.option('--scene-id', type=str, default=None,
              help='Scene ID (default: output directory name)')
@click.option('--gs-config', type=click.Path(exists=True),
              default='gs_training_config.yaml', help='3DGS training configuration')
@click.option('--geometry-config', type=click.Path(exists=True),
              default='geometry_config.yaml', help='Geometry generation configuration')
@click.option('--workers', type=int, default=None,
              help='Number of parallel workers (default: CPU count)')
@click.option('--keep-workspace', is_flag=True,
              help='Keep workspace directory after completion')
@click.option('--skip-validation', is_flag=True,
              help='Skip world_bundle validation')
@click.option('--verbose', '-v', is_flag=True,
              help='Verbose output')
def build(
    input: str,
    output: str,
    config: str,
    scene_id: Optional[str],
    gs_config: str,
    geometry_config: str,
    workers: Optional[int],
    keep_workspace: bool,
    skip_validation: bool,
    verbose: bool
):
    """Build world_bundle from MCAP"""

    # Scene ID のデフォルト設定
    if scene_id is None:
        scene_id = Path(output).name

    # ログレベル設定
    log_level = logging.DEBUG if verbose else logging.INFO

    try:
        # Config 読み込み
        pipeline_config = load_pipeline_config(
            config_file=Path(config),
            gs_config_file=Path(gs_config),
            geometry_config_file=Path(geometry_config),
            input_mcap=Path(input),
            output_dir=Path(output),
            scene_id=scene_id,
            workers=workers
        )

        # Pipeline 実行
        pipeline = WorldBuilderPipeline(pipeline_config, log_level=log_level)
        result = pipeline.run()

        # ワークスペース削除（オプション）
        if not keep_workspace and result.status == ProcessStatus.SUCCESS:
            import shutil
            shutil.rmtree(pipeline.workspace, ignore_errors=True)

        # 結果表示と終了コード決定
        exit_code = get_exit_code(result, skip_validation)

        if exit_code == 0:
            click.echo(click.style("✓ Build succeeded!", fg='green'))
            click.echo(f"World bundle: {result.world_bundle_path}")
            click.echo(f"Build report: {result.build_report_path}")
        else:
            click.echo(click.style("✗ Build failed!", fg='red'))
            for comp_result in result.component_results:
                if comp_result.status == ProcessStatus.FAILED:
                    click.echo(f"  Failed at: {comp_result.component_name}")
                    for error in comp_result.errors:
                        click.echo(f"    - {error}")

        exit(exit_code)

    except FileNotFoundError as e:
        click.echo(click.style(f"✗ File not found: {e}", fg='red'))
        exit(3)  # 入力MCAPエラー
    except yaml.YAMLError as e:
        click.echo(click.style(f"✗ Configuration file error: {e}", fg='red'))
        exit(2)  # 設定ファイルエラー
    except Exception as e:
        click.echo(click.style(f"✗ Unexpected error: {e}", fg='red'))
        if verbose:
            import traceback
            traceback.print_exc()
        exit(1)  # 一般的なエラー

def get_exit_code(result: PipelineResult, skip_validation: bool) -> int:
    """終了コードを決定

    0: 成功
    1: 一般的なエラー
    2: 設定ファイルエラー
    3: 入力MCAPエラー
    4: トピック不足エラー
    5: 処理失敗
    6: 検証失敗
    """
    if result.status == ProcessStatus.SUCCESS:
        return 0

    # エラー種別を特定
    for comp_result in result.component_results:
        if comp_result.status == ProcessStatus.FAILED:
            # エラーコードから終了コードを決定
            if comp_result.component_name == "mcap_ingest":
                # TOPIC_NOT_FOUND などのエラーを確認
                for error in comp_result.errors:
                    if "TOPIC_NOT_FOUND" in error or "topic" in error.lower():
                        return 4
                    if "MCAP" in error or "file" in error.lower():
                        return 3
            elif comp_result.component_name == "world_bundle_packer":
                if not skip_validation:
                    return 6  # 検証失敗

            # その他の処理失敗
            return 5

    return 1  # 一般的なエラー

@cli.command()
@click.argument('world_bundle_dir', type=click.Path(exists=True))
def validate(world_bundle_dir: str):
    """Validate world_bundle"""

    validator = WorldBundleValidator()
    result = validator.validate(Path(world_bundle_dir))

    if result.is_valid:
        click.echo(click.style("✓ Validation passed!", fg='green'))
        exit(0)
    else:
        click.echo(click.style("✗ Validation failed!", fg='red'))
        for error in result.errors:
            click.echo(f"  - {error}")
        exit(1)

if __name__ == '__main__':
    cli()
```

---

## 依存関係

### 必須パッケージ

```python
# pyproject.toml または requirements.txt

# Core
python = "^3.10"
numpy = "^1.24.0"
pandas = "^2.0.0"
pyyaml = "^6.0"

# MCAP
mcap = "^1.0.0"
mcap-ros2-support = "^0.5.0"

# ROS2 (message definitions)
rclpy = "^3.0"  # for message parsing

# Point cloud processing
open3d = "^0.17.0"

# GIS
shapely = "^2.0.0"
geojson = "^3.0.0"

# 3DGS (DriveStudio dependencies)
torch = "^2.0.0"
# [DriveStudio requirements...]

# CLI
click = "^8.1.0"
tqdm = "^4.65.0"
rich = "^13.0.0"

# Testing
pytest = "^7.4.0"
pytest-cov = "^4.1.0"
```

---

## 変更履歴

| バージョン | 日付 | 変更内容 |
|-----------|------|---------|
| 1.0.0 | 2026-02-14 | 初版作成 |
