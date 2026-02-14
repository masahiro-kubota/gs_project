# オフライン処理契約（MCAP → world_bundle）

**バージョン**: 1.0.1
**最終更新**: 2026-02-15

## 概要

`gs_world_builder` は MCAP形式の実走行ログから、`gs_ros2_simulator` で使用可能な
world_bundle を生成するオフライン処理パイプラインです。

**準拠規約**: この契約は [`conventions.md`](../conventions.md) および
[`world_bundle_schema.md`](world_bundle_schema.md) で定義された規約に従います。

**実装コンポーネント**: `gs_world_builder`
**内部設計**: [`gs_world_builder/docs/architecture.md`](../../gs_world_builder/docs/architecture.md)

---

## 設計原則

1. **再現可能性**: 同じ入力から常に同じ world_bundle を生成
2. **トレーサビリティ**: すべての処理パラメータと中間生成物を記録
3. **モジュール性**: 各処理ステップを独立して実行・検証可能
4. **ランタイム非依存**: DriveStudio等の依存は builder 内で完結
5. **設定ファイル駆動**: すべての設定を YAML で外部化

---

## 入力インターフェース

### 1. 必須入力

#### MCAP ファイル

```
入力: *.mcap
場所: 任意のパス（コマンドライン引数で指定）
```

**期待される MCAP の内容**:

| トピック名（例） | メッセージ型 | 必須/任意 | 説明 |
|----------------|------------|---------|------|
| `/camera/front/image_raw` | `sensor_msgs/Image` | 必須 | フロントカメラ画像 |
| `/camera/left/image_raw` | `sensor_msgs/Image` | 任意 | 左カメラ画像 |
| `/camera/right/image_raw` | `sensor_msgs/Image` | 任意 | 右カメラ画像 |
| `/lidar/points` | `sensor_msgs/PointCloud2` | 必須 | LiDAR点群 |
| `/tf` | `tf2_msgs/TFMessage` | 必須 | 動的TF |
| `/tf_static` | `tf2_msgs/TFMessage` | 必須 | 静的TF |
| `/odom` または `/localization/pose` | `nav_msgs/Odometry` または `geometry_msgs/PoseStamped` | 必須 | Ego位置 |
| `/camera/*/camera_info` | `sensor_msgs/CameraInfo` | 必須 | カメラ較正情報 |

**データ品質要件**:
- カメラ: 最低 10Hz、推奨 12Hz 以上
- LiDAR: 最低 10Hz、推奨 20Hz 以上
- 記録時間: 最低 30秒、推奨 60秒 以上
- TF: すべてのセンサから base_link への変換が利用可能
- 同期: センサ間の時刻同期が取れていること（許容誤差 ±50ms）

---

### 2. 設定ファイル

#### config.yaml（パイプライン全体設定）

```yaml
version: "1.0.0"

# 入出力
input:
  mcap_file: "/path/to/input.mcap"

output:
  world_bundle_dir: "worlds/my_scene_001"
  scene_id: "my_scene_001"

# トピックマッピング
topics:
  cameras:
    front: "/camera/front/image_raw"
    left: "/camera/left/image_raw"
    right: "/camera/right/image_raw"

  lidars:
    top: "/lidar/points"

  odom: "/odom"  # または "/localization/pose"

  tf: "/tf"
  tf_static: "/tf_static"

# フレームID
frames:
  map: "map"
  odom: "odom"
  base_link: "base_link"
  cameras:
    front: "camera_front"
    left: "camera_left"
    right: "camera_right"
  lidars:
    top: "lidar_top"

# 処理範囲（オプション: 全体を使う場合は省略）
time_range:
  start_sec: 0.0      # MCAP開始からのオフセット [s]
  duration_sec: 60.0  # 処理する長さ [s]

# 座標系原点（オプション: 緯度経度がある場合）
origin:
  latitude: 35.681236
  longitude: 139.767125
  altitude: 40.0

# パイプライン有効化フラグ
pipeline:
  enable_mcap_ingest: true
  enable_calibration_builder: true
  enable_drivesim_converter: true
  enable_gs_training: true
  enable_gs_export: true
  enable_geometry_builder: true
  enable_world_bundle_packer: true
  enable_validation: true
```

---

#### gs_training_config.yaml（3DGS学習設定）

```yaml
version: "1.0.0"

model:
  type: "background_only"  # 背景のみ学習
  sh_degree: 3

training:
  iterations: 30000
  batch_size: 1
  learning_rate:
    position: 0.00016
    feature: 0.0025
    opacity: 0.05
    scaling: 0.005
    rotation: 0.001

  densify:
    start_iter: 500
    end_iter: 15000
    interval: 100
    grad_threshold: 0.0002

  # 再現性
  seed: 42

optimization:
  refine_pose: false       # 初期はfalse推奨
  refine_exposure: false   # 露出補正が必要な場合 true

output:
  save_iterations: [7000, 15000, 30000]
  checkpoint_dir: "checkpoints"
```

---

#### geometry_config.yaml（幾何生成設定）

```yaml
version: "1.0.0"

heightmap:
  resolution: 0.1         # [m/cell]
  extent:
    x_min: -100.0
    x_max: 300.0
    y_min: -100.0
    y_max: 100.0

  method: "lidar_projection"  # or "mesh_rasterization"
  filter:
    median_kernel: 5
    outlier_threshold: 0.5  # [m]

drivable:
  method: "trajectory_buffer"  # 最初はこれで十分
  buffer_width: 10.0           # [m] 軌跡の左右バッファ
  simplify_tolerance: 0.5      # [m] ポリゴン簡略化

static_mesh:
  enable: false                # 最初は無効化推奨
  method: "poisson"            # or "tsdf"
  voxel_size: 0.05            # [m]
```

---

## 出力インターフェース

### 1. 主出力: World Bundle

```
出力先: ${output.world_bundle_dir}/
フォーマット: world_bundle_schema.md に準拠
```

**必須ファイル**:
- `world.yaml`
- `metadata.json`
- `gaussians/background.splat.ply`
- `gaussians/render_config.json`
- `geometry/heightmap.bin`
- `geometry/heightmap.yaml`
- `geometry/drivable.geojson`
- `sensors/calibration.yaml`
- `sensors/tf_static.json`
- `sim/timebase.yaml`

**任意ファイル**:
- `geometry/static_mesh.glb`
- `preview/*.png`, `preview/*.mp4`

---

### 2. 副出力: 処理ログとアーティファクト

```
作業ディレクトリ: .gs_world_builder_workspace/<scene_id>/
```

**ディレクトリ構成**:
```
.gs_world_builder_workspace/<scene_id>/
├── 00_mcap_ingest/
│   ├── frames/              # 抽出フレーム情報
│   ├── images/              # 抽出画像（必要な場合）
│   ├── lidar/               # 抽出点群
│   ├── poses.csv            # Ego軌跡
│   └── timeline.json        # タイムスタンプ一覧
├── 01_calibration/
│   ├── calibration.yaml     # 生成したキャリブレーション
│   └── tf_static.json
├── 02_drivesim_dataset/
│   ├── images/              # DriveStudio用画像
│   ├── poses/               # DriveStudio用pose
│   └── dataset_config.json
├── 03_gs_training/
│   ├── checkpoints/         # 学習チェックポイント
│   ├── training_log.txt     # 学習ログ
│   └── metrics.json         # PSNR等の指標
├── 04_gs_export/
│   ├── background.splat.ply
│   └── render_config.json
├── 05_geometry/
│   ├── heightmap.bin
│   ├── heightmap.yaml
│   ├── drivable.geojson
│   └── point_cloud_map.pcd  # 統合点群
├── 06_world_bundle/
│   └── (world_bundleのコピー)
└── build_report.json        # 全体のサマリ
```

---

### 3. ビルドレポート

#### build_report.json

```json
{
  "version": "1.0.0",
  "scene_id": "my_scene_001",
  "build_time": "2026-02-14T12:34:56Z",
  "builder_version": "gs_world_builder-0.1.0",

  "input": {
    "mcap_file": "/path/to/input.mcap",
    "mcap_size_bytes": 1234567890,
    "mcap_duration_sec": 120.5,
    "mcap_md5": "a1b2c3..."
  },

  "config": {
    "pipeline_config": "config.yaml",
    "gs_training_config": "gs_training_config.yaml",
    "geometry_config": "geometry_config.yaml"
  },

  "processing_time": {
    "mcap_ingest_sec": 45.2,
    "calibration_builder_sec": 2.1,
    "drivesim_converter_sec": 120.5,
    "gs_training_sec": 3600.0,
    "gs_export_sec": 10.3,
    "geometry_builder_sec": 180.0,
    "world_bundle_packer_sec": 5.0,
    "total_sec": 3963.1
  },

  "statistics": {
    "num_frames": 1446,
    "num_cameras": 3,
    "num_lidars": 1,
    "trajectory_length_m": 1234.5,
    "gaussian_count": 2500000,
    "final_psnr": 28.5
  },

  "validation": {
    "world_bundle_valid": true,
    "errors": [],
    "warnings": [
      "static_mesh.glb not generated (disabled in config)"
    ]
  },

  "output": {
    "world_bundle_dir": "worlds/my_scene_001",
    "world_bundle_size_bytes": 456789012
  }
}
```

---

## コマンドラインインターフェース

### 基本実行

```bash
gs-world-builder build \
    --input /path/to/input.mcap \
    --output worlds/my_scene_001 \
    --config config.yaml
```

### オプション

```bash
gs-world-builder build \
    --input INPUT_MCAP \
    --output OUTPUT_DIR \
    --config CONFIG_YAML \
    --scene-id SCENE_ID \
    --gs-config GS_CONFIG_YAML \
    --geometry-config GEOMETRY_CONFIG_YAML \
    --workers NUM_WORKERS \
    --keep-workspace \
    --skip-validation \
    --verbose
```

**引数詳細**:
- `--input`: 入力MCAPファイルパス（必須）
- `--output`: 出力world_bundleディレクトリ（必須）
- `--config`: パイプライン設定ファイル（デフォルト: `config.yaml`）
- `--scene-id`: シーンID（デフォルト: 出力ディレクトリ名）
- `--gs-config`: 3DGS学習設定（デフォルト: `gs_training_config.yaml`）
- `--geometry-config`: 幾何生成設定（デフォルト: `geometry_config.yaml`）
- `--workers`: 並列処理数（デフォルト: CPU数）
- `--keep-workspace`: 作業ディレクトリを削除せず保持
- `--skip-validation`: 検証を省略
- `--verbose`: 詳細ログ出力

---

### ステップ単位実行

パイプラインの特定ステップのみ実行:

```bash
# MCAP取り込みのみ
gs-world-builder run-step mcap_ingest \
    --input input.mcap \
    --output-workspace workspace/

# 3DGS学習のみ（既存データセットから）
gs-world-builder run-step gs_training \
    --input-workspace workspace/ \
    --config gs_training_config.yaml
```

---

### 検証のみ

既存の world_bundle を検証:

```bash
gs-world-builder validate worlds/my_scene_001
```

出力:
```
✓ world.yaml: valid
✓ metadata.json: valid
✓ gaussians/background.splat.ply: valid (2.5M points)
✓ geometry/heightmap.bin: valid (2048x2048)
✓ geometry/drivable.geojson: valid (5 polygons)
✓ sensors/calibration.yaml: valid (3 cameras, 1 lidar)
✓ sensors/tf_static.json: valid (4 transforms)
✓ sim/timebase.yaml: valid
⚠ geometry/static_mesh.glb: missing (optional)

World bundle is valid.
```

---

## Python APIインターフェース

### 基本使用

```python
from gs_world_builder import WorldBuilder

builder = WorldBuilder(
    input_mcap="/path/to/input.mcap",
    output_dir="worlds/my_scene_001",
    config="config.yaml"
)

# 全パイプライン実行
report = builder.build()

print(f"Build completed: {report['validation']['world_bundle_valid']}")
print(f"Total time: {report['processing_time']['total_sec']} sec")
```

---

### ステップ単位実行

```python
from gs_world_builder import WorldBuilder

builder = WorldBuilder(...)

# ステップごとに実行
builder.run_mcap_ingest()
builder.run_calibration_builder()
builder.run_drivesim_converter()

# 3DGS学習（時間がかかる）
builder.run_gs_training(
    config="gs_training_config.yaml",
    on_progress=lambda iter, loss: print(f"Iter {iter}: {loss}")
)

builder.run_gs_export()
builder.run_geometry_builder()
builder.run_world_bundle_packer()

# 検証
is_valid = builder.validate()
```

---

## 依存パッケージ

### システム依存

```bash
# Ubuntu/Debian
apt-get install -y \
    python3.10 \
    ffmpeg \
    libgl1-mesa-glx \
    libglib2.0-0
```

### Python依存

```txt
# Core
numpy>=1.24.0
pyyaml>=6.0
opencv-python>=4.8.0
scipy>=1.10.0

# MCAP
mcap>=0.8.0

# ROS2 (rosbag2, message definitions)
rosbags>=0.9.0

# Point cloud processing
open3d>=0.17.0
plyfile>=1.0.0

# GIS
shapely>=2.0.0
geojson>=3.0.0

# 3DGS (DriveStudio依存)
torch>=2.0.0
[DriveStudio requirements...]

# CLI
click>=8.1.0
tqdm>=4.65.0
rich>=13.0.0  # 進捗表示
```

---

## エラーハンドリング

### 終了コード

| コード | 意味 |
|-------|------|
| 0 | 成功 |
| 1 | 一般的なエラー |
| 2 | 設定ファイルエラー |
| 3 | 入力MCAPエラー（ファイルが存在しない、壊れている等） |
| 4 | トピック不足エラー（必須トピックが見つからない） |
| 5 | 処理失敗（中間ステップでエラー） |
| 6 | 検証失敗 |

---

### エラーメッセージ形式

```json
{
  "error": {
    "code": "MISSING_TOPIC",
    "message": "Required topic '/camera/front/image_raw' not found in MCAP",
    "details": {
      "available_topics": ["/camera/back/image_raw", "/lidar/points"]
    },
    "suggestion": "Check topic mapping in config.yaml"
  }
}
```

---

## 設定のバリデーション

builder 起動時に設定ファイルを検証:

```bash
gs-world-builder validate-config config.yaml
```

チェック項目:
- [ ] YAML構文の正しさ
- [ ] 必須フィールドの存在
- [ ] ファイルパスの妥当性
- [ ] 数値範囲の妥当性
- [ ] トピック名の重複チェック

---

## 再現可能性の保証

### 固定すべき要素

1. **乱数シード**: `gs_training_config.yaml` の `seed`
2. **ソフトウェアバージョン**: `build_report.json` に記録
3. **設定ファイル**: workspace に設定ファイルのコピーを保存
4. **入力データ**: MCAP の MD5 ハッシュを記録

### 再ビルド

```bash
# build_report.json から再ビルド
gs-world-builder rebuild --report workspace/build_report.json
```

---

## パフォーマンス目安

| 入力MCAP | 処理時間（目安） | 備考 |
|---------|----------------|------|
| 60秒, 3カメラ, 1LiDAR | 60-90分 | GPU必須（3DGS学習） |
| 120秒, 3カメラ, 1LiDAR | 90-120分 | 学習時間が支配的 |

**ボトルネック**: 3DGS学習（全体の90%）

---

## 変更履歴

| バージョン | 日付 | 変更内容 |
|-----------|------|---------|
| 1.0.1 | 2026-02-15 | スキーマリファレンス修正 |
| 1.0.0 | 2026-02-14 | 初版作成 |
