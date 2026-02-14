# gs_world_builder 内部コンポーネント間インターフェース

**バージョン**: 1.0.0
**最終更新**: 2026-02-14

## 概要

このドキュメントは、`gs_world_builder` の内部コンポーネント間でやり取りされるすべてのデータの**厳密な仕様**を定義します。

**重要な区別**:
- **[architecture.md](architecture.md)** - 各コンポーネントの処理内容と実装方針
- **このドキュメント** - コンポーネント間のインターフェース契約（データフォーマット、検証ルール）

**準拠規約**: すべてのファイルは [`../../docs/conventions.md`](../../docs/conventions.md) に従います。

---

## コンポーネント間の依存関係

### データフロー図

```
mcap
 ↓
[1. mcap_ingest] ────────────────────────────────┐
 ↓                                               │
 ├─→ raw_extract/                                │
 ├─→ timeline.json                               │
 ├─→ frameset.json                               │
 └─→ timebase.yaml                               │
                                                  │
 ┌────────────────────────────────────────────────┤
 │                                                │
 ↓                                                │
[2. calibration_builder]                         │
 ├─→ calibration.yaml ────────┐                  │
 ├─→ tf_static.json           │                  │
 └─→ metadata.json (部分)     │                  │
                               ↓                  │
[3. drivesim_dataset_converter] ←────────────────┤
 ├─→ images/                                      │
 ├─→ poses/                                       │
 ├─→ intrinsics.json                              │
 └─→ dataset_config.json                          │
      ↓                                           │
[4. gs_train_orchestrator]                       │
 ├─→ checkpoints/                                 │
 ├─→ training_log.txt                             │
 ├─→ metrics.json                                 │
 └─→ training_report.json                         │
      ↓                                           │
[5. gs_exporter]                                  │
 ├─→ background.splat.ply ────┐                  │
 └─→ render_config.json       │                  │
                               │                  │
[6. geometry_builder] ←────────────────────────────┘
 ├─→ [6a. pointcloud_integrator]
 │    └─→ point_cloud_map.pcd ────┐
 ├─→ [6b. ground_surface_builder] │
 │    ├─→ heightmap.bin            │
 │    └─→ heightmap.yaml           │
 ├─→ [6c. drivable_area_builder]  │
 │    └─→ drivable.geojson         │
 └─→ [6d. static_mesh_builder] ←──┘
      └─→ static_mesh.glb (optional)
                               │
                               ↓
[7. world_bundle_packer] ←─────┴─ (すべての成果物)
 ├─→ worlds/<scene_id>/
 └─→ build_report.json
```

### 依存関係マトリクス

| ファイル | 生成者 | 消費者 |
|---------|-------|--------|
| `raw_extract/poses.csv` | mcap_ingest | calibration_builder, geometry_builder |
| `raw_extract/tf/` | mcap_ingest | calibration_builder |
| `raw_extract/images/` | mcap_ingest | drivesim_dataset_converter |
| `raw_extract/lidar/` | mcap_ingest | geometry_builder |
| `timeline.json` | mcap_ingest | (参照用) |
| `frameset.json` | mcap_ingest | drivesim_dataset_converter |
| `timebase.yaml` | mcap_ingest | world_bundle_packer |
| `calibration.yaml` | calibration_builder | drivesim_dataset_converter, world_bundle_packer |
| `tf_static.json` | calibration_builder | world_bundle_packer |
| `metadata.json (部分)` | calibration_builder | world_bundle_packer |
| `02_drivesim_dataset/*` | drivesim_dataset_converter | gs_train_orchestrator |
| `checkpoints/` | gs_train_orchestrator | gs_exporter |
| `training_report.json` | gs_train_orchestrator | gs_exporter |
| `background.splat.ply` | gs_exporter | world_bundle_packer |
| `render_config.json` | gs_exporter | world_bundle_packer |
| `point_cloud_map.pcd` | pointcloud_integrator | ground_surface_builder, static_mesh_builder |
| `heightmap.*` | ground_surface_builder | world_bundle_packer |
| `drivable.geojson` | drivable_area_builder | world_bundle_packer |
| `static_mesh.glb` | static_mesh_builder | world_bundle_packer |

---

## 1. mcap_ingest 出力インターフェース

### 1.1 raw_extract/poses.csv

**目的**: Ego車両の軌跡データ

**ファイルパス**: `workspace/00_mcap_ingest/raw_extract/poses.csv`

**フォーマット**: CSV (UTF-8, LF改行)

**ヘッダー**: `timestamp,x,y,z,qx,qy,qz,qw`

**カラム仕様**:

| カラム | データ型 | 単位 | 説明 | 制約 |
|--------|---------|------|------|------|
| `timestamp` | float64 | [s] | UNIX epoch time | 昇順ソート済み |
| `x` | float64 | [m] | map frame X座標 | - |
| `y` | float64 | [m] | map frame Y座標 | - |
| `z` | float64 | [m] | map frame Z座標 | - |
| `qx` | float64 | - | クォータニオン x成分 | \|\|q\|\| = 1 |
| `qy` | float64 | - | クォータニオン y成分 | \|\|q\|\| = 1 |
| `qz` | float64 | - | クォータニオン z成分 | \|\|q\|\| = 1 |
| `qw` | float64 | - | クォータニオン w成分 | \|\|q\|\| = 1 |

**データ制約**:
- 最小行数: 10行
- timestamp間隔: 0.001s ≤ Δt ≤ 1.0s
- クォータニオン正規化: `sqrt(qx² + qy² + qz² + qw²) = 1.0 ± 1e-6`
- 座標系: map frame (ENU)

**消費者**: calibration_builder, geometry_builder

---

### 1.2 raw_extract/tf/tf.json

**目的**: 動的TF情報（フレーム間変換）

**ファイルパス**: `workspace/00_mcap_ingest/raw_extract/tf/tf.json`

**フォーマット**: JSON (UTF-8)

**スキーマ**:
```json
{
  "version": "1.0.0",
  "transforms": [
    {
      "timestamp": 1234567890.123,
      "header": {
        "frame_id": "map"
      },
      "child_frame_id": "odom",
      "transform": {
        "translation": {
          "x": 0.0,
          "y": 0.0,
          "z": 0.0
        },
        "rotation": {
          "x": 0.0,
          "y": 0.0,
          "z": 0.0,
          "w": 1.0
        }
      }
    }
  ]
}
```

**フィールド仕様**:
- `version`: string, "1.0.0"
- `transforms`: array of Transform
  - `timestamp`: float64, UNIX epoch time [s]
  - `header.frame_id`: string, 親フレームID
  - `child_frame_id`: string, 子フレームID
  - `transform.translation.{x,y,z}`: float64, [m]
  - `transform.rotation.{x,y,z,w}`: float64, クォータニオン [x,y,z,w]

**制約**:
- transforms配列は timestamp 昇順でソート
- クォータニオン正規化: ||q|| = 1.0 ± 1e-6

**消費者**: calibration_builder

---

### 1.3 raw_extract/tf/tf_static.json

**目的**: 静的TF情報（固定フレーム間変換）

**ファイルパス**: `workspace/00_mcap_ingest/raw_extract/tf/tf_static.json`

**フォーマット**: JSON (UTF-8)

**スキーマ**: `tf.json` と同一構造だが、timestamp は無視される

**制約**:
- 同一の parent-child ペアは1つのみ
- クォータニオン正規化: ||q|| = 1.0 ± 1e-6

**消費者**: calibration_builder

---

### 1.4 timeline.json

**目的**: 全センサのタイムスタンプ記録（トレーサビリティ用）

**ファイルパス**: `workspace/00_mcap_ingest/timeline.json`

**フォーマット**: JSON (UTF-8)

**スキーマ**:
```json
{
  "version": "1.0.0",
  "source_mcap": "input.mcap",
  "sensors": {
    "camera_front": {
      "type": "camera",
      "topic": "/camera/front/image_raw",
      "frame_count": 1200,
      "timestamps": [1234567890.000, 1234567890.083, ...],
      "start_time": 1234567890.000,
      "end_time": 1234567990.000,
      "avg_rate": 12.0
    },
    "lidar_top": {
      "type": "lidar",
      "topic": "/lidar/points",
      "frame_count": 2000,
      "timestamps": [1234567890.000, 1234567890.050, ...],
      "start_time": 1234567890.000,
      "end_time": 1234567990.000,
      "avg_rate": 20.0
    }
  }
}
```

**フィールド仕様**:
- `version`: string, "1.0.0"
- `source_mcap`: string, MCAP ファイル名
- `sensors`: object
  - `<sensor_id>`: string key, センサID
    - `type`: string, "camera" | "lidar"
    - `topic`: string, MCAP内のトピック名
    - `frame_count`: integer, フレーム数
    - `timestamps`: array of float64, UNIX epoch time [s], 昇順ソート
    - `start_time`: float64, 最初のタイムスタンプ
    - `end_time`: float64, 最後のタイムスタンプ
    - `avg_rate`: float64, 平均レート [Hz]

**制約**:
- timestamps配列は昇順ソート済み
- `frame_count == len(timestamps)`
- `avg_rate = frame_count / (end_time - start_time)`

**消費者**: (参照用、トレーサビリティ)

---

### 1.5 frameset.json

**目的**: シミュレータ用リサンプリング対応表

**ファイルパス**: `workspace/00_mcap_ingest/frameset.json`

**フォーマット**: JSON (UTF-8)

**スキーマ**:
```json
{
  "version": "1.0.0",
  "target_rates": {
    "camera": 12.0,
    "lidar": 20.0
  },
  "frames": [
    {
      "sim_time": 0.000,
      "camera_front": {
        "original_timestamp": 1234567890.000,
        "frame_index": 0,
        "file_path": "raw_extract/images/camera_front/0000000000.jpg"
      },
      "lidar_top": {
        "original_timestamp": 1234567890.000,
        "frame_index": 0,
        "file_path": "raw_extract/lidar/0000000000.pcd"
      }
    },
    {
      "sim_time": 0.050,
      "lidar_top": {
        "original_timestamp": 1234567890.050,
        "frame_index": 1,
        "file_path": "raw_extract/lidar/0000000001.pcd"
      }
    }
  ]
}
```

**フィールド仕様**:
- `version`: string, "1.0.0"
- `target_rates`: object
  - `camera`: float64, カメラの目標レート [Hz]
  - `lidar`: float64, LiDARの目標レート [Hz]
- `frames`: array of Frame
  - `sim_time`: float64, シミュレーション時刻 [s], 0.0 から開始
  - `<sensor_id>`: object (センサごと、トリガーされたフレームのみ)
    - `original_timestamp`: float64, 元のMCAPタイムスタンプ
    - `frame_index`: integer, センサ内でのフレーム番号（0始まり）
    - `file_path`: string, 抽出ファイルへの相対パス

**制約**:
- frames配列は sim_time 昇順でソート
- sim_time は 0.0 から開始
- すべてのセンサが最低1回はトリガーされる

**消費者**: drivesim_dataset_converter

---

### 1.6 timebase.yaml

**目的**: シミュレータ用タイムベース設定の初期値

**ファイルパス**: `workspace/00_mcap_ingest/timebase.yaml`

**フォーマット**: YAML (UTF-8)

**スキーマ**:
```yaml
version: "1.0.0"

simulation:
  dt: 0.01              # [s] 推奨シミュレーションタイムステップ
  start_time: 0.0       # [s] 開始時刻

sensor_rates:
  camera: 12.0          # [Hz] カメラレート
  lidar: 20.0           # [Hz] LiDARレート

initial_pose:
  position: [0.0, 0.0, 0.0]
  orientation: [0.0, 0.0, 0.0, 1.0]  # quaternion [x,y,z,w]
  velocity: [0.0, 0.0, 0.0]
```

**フィールド仕様**:
- `version`: string, "1.0.0"
- `simulation.dt`: float64, [s], シミュレーションタイムステップ
- `simulation.start_time`: float64, [s], 開始時刻
- `sensor_rates.camera`: float64, [Hz], カメラレート
- `sensor_rates.lidar`: float64, [Hz], LiDARレート
- `initial_pose.position`: array of 3 float64, [m], [x, y, z]
- `initial_pose.orientation`: array of 4 float64, クォータニオン [x,y,z,w]
- `initial_pose.velocity`: array of 3 float64, [m/s], [vx, vy, vz]

**制約**:
- `dt > 0.0`
- `sensor_rates > 0.0`
- クォータニオン正規化: ||q|| = 1.0 ± 1e-6

**消費者**: world_bundle_packer

---

### 1.7 raw_extract/images/

**目的**: 抽出・デコード済みカメラ画像

**ファイルパス**: `workspace/00_mcap_ingest/raw_extract/images/<camera_id>/XXXXXXXXXX.jpg`

**ディレクトリ構造**:
```
raw_extract/images/
├── camera_front/
│   ├── 0000000000.jpg
│   ├── 0000000001.jpg
│   └── ...
├── camera_left/
└── camera_right/
```

**ファイル名規則**:
- フォーマット: `%010d.jpg` (10桁ゼロ埋め)
- frame_index に対応（0始まり）

**画像フォーマット**:
- エンコーディング: JPEG または PNG
- カラースペース: RGB
- サイズ: 元のMCAP画像と同一（リサイズなし）

**制約**:
- 各カメラディレクトリ内のファイルは連番
- 欠番は許容されない
- ファイル名の番号 == frame_index

**消費者**: drivesim_dataset_converter

---

### 1.8 raw_extract/lidar/

**目的**: 抽出済みLiDAR点群（フレーム単位）

**ファイルパス**: `workspace/00_mcap_ingest/raw_extract/lidar/XXXXXXXXXX.pcd` (または `.npz`)

**ファイル名規則**:
- フォーマット: `%010d.pcd` または `%010d.npz` (10桁ゼロ埋め)
- frame_index に対応（0始まり）

**フォーマット（PCD推奨）**:
```
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH <N>
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS <N>
DATA binary
<binary data>
```

**フィールド仕様（PCD）**:
- `x, y, z`: float32, [m], base_link frame での座標
- `intensity`: float32, 反射強度（オプション、0.0-255.0）

**フォーマット（NPZ代替）**:
```python
{
  'points': np.ndarray[N, 3],  # float32, [x, y, z]
  'intensity': np.ndarray[N],  # float32, optional
  'timestamp': float64         # UNIX epoch time
}
```

**制約**:
- 座標系: base_link frame (FLU)
- 点数: 最低100点、最大200,000点
- x, y, z に NaN/Inf を含まない

**消費者**: geometry_builder (pointcloud_integrator)

---

## 2. calibration_builder 出力インターフェース

### 2.1 calibration.yaml

**目的**: センサキャリブレーション情報の確定

**ファイルパス**: `workspace/01_calibration/calibration.yaml`

**フォーマット**: YAML (UTF-8)

**スキーマ**: [`../../docs/interfaces/world_bundle_schema.md#5-1`](../../docs/interfaces/world_bundle_schema.md#5-1) に準拠

**重要な追加制約**:
- **calibration.yaml が正**: tf_static.json はこれから派生
- extrinsics の translation/rotation は tf_static.json と厳密に一致
- camera_convention ("opencv" | "ros") を明示

**消費者**: drivesim_dataset_converter, world_bundle_packer

---

### 2.2 tf_static.json

**目的**: 静的TF情報（calibration.yaml から派生）

**ファイルパス**: `workspace/01_calibration/tf_static.json`

**フォーマット**: JSON (UTF-8)

**スキーマ**: [`../../docs/interfaces/world_bundle_schema.md#5-2`](../../docs/interfaces/world_bundle_schema.md#5-2) に準拠

**生成ルール**:
```
calibration.yaml の cameras.<id>.extrinsics
  ↓
tf_static.json の transforms (base_link → camera_<id>)

calibration.yaml の lidars.<id>.extrinsics
  ↓
tf_static.json の transforms (base_link → lidar_<id>)
```

**整合性制約**:
- calibration.yaml の extrinsics と tf_static.json の transform は**完全一致**
- 不一致は検証エラー

**消費者**: world_bundle_packer

---

### 2.3 metadata.json (部分)

**目的**: メタ情報の座標系セクション

**ファイルパス**: `workspace/01_calibration/metadata.json`

**フォーマット**: JSON (UTF-8)

**スキーマ**:
```json
{
  "version": "1.0.0",
  "coordinate_system": {
    "map_frame": "map",
    "odom_frame": "odom",
    "base_link_frame": "base_link",
    "convention": "ROS2 REP-103 (FLU)",
    "origin_lat": 35.681236,
    "origin_lon": 139.767125,
    "origin_alt": 40.0
  }
}
```

**フィールド仕様**:
- `version`: string, "1.0.0"
- `coordinate_system`: object
  - `map_frame`: string, map フレームID
  - `odom_frame`: string, odom フレームID
  - `base_link_frame`: string, base_link フレームID
  - `convention`: string, "ROS2 REP-103 (FLU)"
  - `origin_lat`: float64, オプション, 緯度 [deg]
  - `origin_lon`: float64, オプション, 経度 [deg]
  - `origin_alt`: float64, オプション, 高度 [m]

**制約**:
- origin_* は3つセットで存在するか、すべて省略

**消費者**: world_bundle_packer（最終metadata.jsonに統合）

---

## 3. drivesim_dataset_converter 出力インターフェース

### 3.1 02_drivesim_dataset/

**目的**: DriveStudio が読み込める形式のデータセット

**ディレクトリ構造**:
```
workspace/02_drivesim_dataset/
├── images/
│   ├── camera_front/
│   ├── camera_left/
│   └── camera_right/
├── poses/
│   ├── camera_front.txt
│   ├── camera_left.txt
│   └── camera_right.txt
├── intrinsics.json
└── dataset_config.json
```

**フォーマット**: DriveStudio 固有のフォーマット

**制約**:
- **DriveStudio のバージョンに依存**
- 内部フォーマットの詳細は DriveStudio ドキュメント参照
- commit hash を dataset_config.json に記録

**重要なフィールド**:
- `dataset_config.json` には以下を含む:
  - `drivesim_commit`: string, DriveStudio の commit hash
  - `mode`: string, "background_only"
  - `camera_ids`: array of string
  - `frame_count`: integer

**消費者**: gs_train_orchestrator

---

## 4. gs_train_orchestrator 出力インターフェース

### 4.1 checkpoints/

**目的**: 3DGS 学習チェックポイント

**ディレクトリ構造**:
```
workspace/03_gs_training/checkpoints/
├── iteration_7000/
├── iteration_15000/
└── iteration_30000/
```

**フォーマット**: DriveStudio 内部フォーマット

**制約**:
- 各 iteration ディレクトリには DriveStudio の checkpoint が含まれる
- 内部構造は DriveStudio に依存

**消費者**: gs_exporter

---

### 4.2 training_report.json

**目的**: 学習結果サマリ（最良iterationの特定）

**ファイルパス**: `workspace/03_gs_training/training_report.json`

**フォーマット**: JSON (UTF-8)

**スキーマ**:
```json
{
  "version": "1.0.0",
  "config": {
    "seed": 42,
    "max_iterations": 30000,
    "mode": "background_only",
    "drivesim_commit": "abc123..."
  },
  "training_time_sec": 3600.0,
  "best_iteration": 30000,
  "best_psnr": 28.5,
  "iterations": [
    {
      "iteration": 7000,
      "psnr": 26.2,
      "loss": 0.012
    },
    {
      "iteration": 15000,
      "psnr": 27.8,
      "loss": 0.008
    },
    {
      "iteration": 30000,
      "psnr": 28.5,
      "loss": 0.005
    }
  ]
}
```

**フィールド仕様**:
- `version`: string, "1.0.0"
- `config`: object, 学習設定
  - `seed`: integer, ランダムシード
  - `max_iterations`: integer, 最大イテレーション数
  - `mode`: string, "background_only"
  - `drivesim_commit`: string, DriveStudio commit hash
- `training_time_sec`: float64, [s], 学習時間
- `best_iteration`: integer, 最良PSNR時のiteration
- `best_psnr`: float64, 最良PSNR値
- `iterations`: array of Iteration
  - `iteration`: integer, イテレーション番号
  - `psnr`: float64, PSNR値
  - `loss`: float64, Loss値

**制約**:
- `best_iteration` は `iterations` 配列に存在する iteration
- iterations配列は iteration 昇順でソート

**消費者**: gs_exporter

---

## 5. gs_exporter 出力インターフェース

### 5.1 background.splat.ply

**目的**: ランタイム用 Gaussian データ（PLY形式）

**ファイルパス**: `workspace/04_gs_export/background.splat.ply`

**フォーマット**: PLY (binary_little_endian 1.0)

**スキーマ**: [`../../docs/conventions.md#L322-L377`](../../docs/conventions.md#L322-L377) の「Gaussian形式の詳細仕様」に準拠

**必須属性**:
- `x, y, z`: float32, [m], 位置
- `scale_0, scale_1, scale_2`: float32, スケール
- `rot_0, rot_1, rot_2, rot_3`: float32, クォータニオン [x,y,z,w]
- `opacity`: float32, 不透明度
- `f_dc_0, f_dc_1, f_dc_2`: float32, SH DC成分 [R,G,B]
- `f_rest_*`: float32, SH高次成分（sh_degree に依存）

**制約**:
- エンディアン: Little Endian
- Gaussian数: 100 ~ 5,000,000
- クォータニオン正規化: ||q|| = 1.0 ± 1e-6

**消費者**: world_bundle_packer

---

### 5.2 render_config.json

**目的**: レンダリング設定

**ファイルパス**: `workspace/04_gs_export/render_config.json`

**フォーマット**: JSON (UTF-8)

**スキーマ**: [`../../docs/interfaces/world_bundle_schema.md#3-2`](../../docs/interfaces/world_bundle_schema.md#3-2) に準拠

**制約**:
- `sh_degree` は background.splat.ply の SH係数数と整合

**消費者**: world_bundle_packer

---

## 6. geometry_builder 出力インターフェース

### 6.1 point_cloud_map.pcd

**目的**: 統合静的点群地図

**ファイルパス**: `workspace/05_geometry/point_cloud_map.pcd`

**フォーマット**: PCD (binary)

**フィールド仕様**:
```
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH <N>
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS <N>
DATA binary
<binary data>
```

- `x, y, z`: float32, [m], map frame での座標
- `intensity`: float32, 統合後の反射強度（オプション）

**制約**:
- 座標系: map frame (ENU)
- 点数: 最低10,000点
- 動的物体がフィルタリングされている（推奨）
- x, y, z に NaN/Inf を含まない

**消費者**: ground_surface_builder, static_mesh_builder

---

### 6.2 heightmap.bin

**目的**: 地面高さマップ（2.5D）

**ファイルパス**: `workspace/05_geometry/heightmap.bin`

**フォーマット**: バイナリ (Little Endian, float32配列)

**スキーマ**: [`../../docs/interfaces/world_bundle_schema.md#4-1`](../../docs/interfaces/world_bundle_schema.md#4-1) に準拠

**メモリレイアウト**:
- row-major (Y x X)
- 各セル: float32, [m], 高さ値
- 無効領域: NaN (IEEE 754 NaN)

**制約**:
- エンディアン: Little Endian
- データサイズ: `width * height * 4 bytes`
- NaN/Inf は無効領域のみで使用

**消費者**: world_bundle_packer

---

### 6.3 heightmap.yaml

**目的**: heightmap メタ情報

**ファイルパス**: `workspace/05_geometry/heightmap.yaml`

**フォーマット**: YAML (UTF-8)

**スキーマ**: [`../../docs/interfaces/world_bundle_schema.md#4-2`](../../docs/interfaces/world_bundle_schema.md#4-2) に準拠

**制約**:
- `width * height` == heightmap.bin のセル数
- `resolution > 0.0`

**消費者**: world_bundle_packer

---

### 6.4 drivable.geojson

**目的**: 走行可能領域

**ファイルパス**: `workspace/05_geometry/drivable.geojson`

**フォーマット**: GeoJSON (UTF-8)

**スキーマ**: [`../../docs/conventions.md#L380-L434`](../../docs/conventions.md#L380-L434) および [`../../docs/interfaces/world_bundle_schema.md#4-3`](../../docs/interfaces/world_bundle_schema.md#4-3) に準拠

**制約**:
- 座標系: map frame の X-Y 平面
- 最低1つの Polygon feature
- 外周は反時計回り (CCW)
- 穴は時計回り (CW)

**消費者**: world_bundle_packer

---

### 6.5 static_mesh.glb (optional)

**目的**: 精密静的メッシュ

**ファイルパス**: `workspace/05_geometry/static_mesh.glb`

**フォーマット**: glTF 2.0 Binary (.glb)

**スキーマ**: [`../../docs/interfaces/world_bundle_schema.md#4-4`](../../docs/interfaces/world_bundle_schema.md#4-4) に準拠

**制約**:
- 座標系: map frame
- 三角形数: 推奨 ≤ 10,000,000
- **オプション**: 初期実装では省略可

**消費者**: world_bundle_packer

---

## 7. world_bundle_packer 出力インターフェース

### 7.1 worlds/<scene_id>/

**目的**: 最終 world_bundle

**フォーマット**: [`../../docs/interfaces/world_bundle_schema.md`](../../docs/interfaces/world_bundle_schema.md) に準拠

**検証項目**:
1. 必須ファイルの存在確認
2. world.yaml の schema 準拠
3. calibration.yaml と tf_static.json の一貫性
4. heightmap.yaml と heightmap.bin のサイズ整合性
5. drivable.geojson の妥当性（空でない）
6. Gaussian データの読み込み可能性

**消費者**: gs_ros2_simulator（外部）

---

### 7.2 build_report.json

**目的**: ビルド処理の完全なトレーサビリティ

**ファイルパス**: `workspace/build_report.json`

**フォーマット**: JSON (UTF-8)

**スキーマ**: [`../../docs/interfaces/offline_builder.md#L269-L324`](../../docs/interfaces/offline_builder.md#L269-L324) に準拠

**重要なフィールド**:
- `input.mcap_md5`: string, 入力MCAPのMD5ハッシュ
- `processing_time.*`: 各ステップの処理時間
- `statistics.*`: 統計情報（Gaussian数、heightmap解像度等）
- `validation.status`: "success" | "failed"
- `errors`, `warnings`: array of string

**制約**:
- validation.status == "success" の場合のみ world_bundle が生成される
- validation.status == "failed" の場合、errors 配列に詳細

**消費者**: トレーサビリティ、再ビルド

---

## データ整合性ルール

### 1. calibration.yaml と tf_static.json の一貫性

**ルール**: calibration.yaml が正、tf_static.json は派生

**検証方法**:
```python
def validate_calibration_tf_consistency(calib_yaml, tf_static_json):
    for sensor_id, sensor in calib_yaml['cameras'].items():
        frame_id = sensor['frame_id']
        expected_transform = sensor['extrinsics']

        # tf_static.json から該当 transform を検索
        actual_transform = find_transform(tf_static_json,
                                          parent='base_link',
                                          child=frame_id)

        # translation の一致確認
        assert np.allclose(expected_transform['translation'],
                          actual_transform['translation'],
                          atol=1e-6)

        # rotation (quaternion) の一致確認
        assert np.allclose(expected_transform['rotation_quat'],
                          [actual_transform['rotation']['x'],
                           actual_transform['rotation']['y'],
                           actual_transform['rotation']['z'],
                           actual_transform['rotation']['w']],
                          atol=1e-6)
```

---

### 2. heightmap.yaml と heightmap.bin のサイズ整合性

**ルール**: `file_size(heightmap.bin) == width * height * 4`

**検証方法**:
```python
def validate_heightmap_size(yaml_path, bin_path):
    meta = yaml.safe_load(open(yaml_path))
    width = meta['width']
    height = meta['height']

    file_size = os.path.getsize(bin_path)
    expected_size = width * height * 4  # float32 = 4 bytes

    assert file_size == expected_size, \
        f"Heightmap size mismatch: {file_size} != {expected_size}"
```

---

### 3. frameset.json と raw_extract/ の整合性

**ルール**: frameset.json 内のすべての file_path が存在する

**検証方法**:
```python
def validate_frameset_files(frameset_json, workspace_dir):
    frameset = json.load(open(frameset_json))

    for frame in frameset['frames']:
        for sensor_id, sensor_data in frame.items():
            if sensor_id == 'sim_time':
                continue

            file_path = os.path.join(workspace_dir, sensor_data['file_path'])
            assert os.path.exists(file_path), \
                f"Missing file: {file_path}"
```

---

## エラー定義

### コンポーネント別エラーコード

| コンポーネント | エラー種別 | 説明 |
|-------------|----------|------|
| mcap_ingest | MCAP_READ_ERROR | MCAP ファイルが開けない、壊れている |
| mcap_ingest | TOPIC_NOT_FOUND | 必須トピックが見つからない |
| mcap_ingest | INSUFFICIENT_DATA | データ量が不足（最低時間・フレーム数未満） |
| calibration_builder | TF_MISSING | 必要な TF が見つからない |
| calibration_builder | CAMERA_INFO_MISSING | camera_info が見つからない |
| drivesim_dataset_converter | INVALID_CALIBRATION | calibration.yaml の形式エラー |
| gs_train_orchestrator | TRAINING_FAILED | 3DGS 学習が失敗 |
| gs_train_orchestrator | GPU_OUT_OF_MEMORY | GPU メモリ不足 |
| gs_exporter | CHECKPOINT_NOT_FOUND | checkpoint が見つからない |
| geometry_builder | POINTCLOUD_EMPTY | 統合点群が空 |
| geometry_builder | HEIGHTMAP_INVALID | heightmap 生成失敗 |
| world_bundle_packer | VALIDATION_FAILED | world_bundle の検証失敗 |
| world_bundle_packer | FILE_MISSING | 必須ファイルが欠損 |

### エラーメッセージ形式

```json
{
  "error": {
    "code": "TOPIC_NOT_FOUND",
    "component": "mcap_ingest",
    "message": "Required topic '/camera/front/image_raw' not found in MCAP",
    "details": {
      "available_topics": ["/camera/rear/image_raw", "/lidar/points"]
    },
    "suggestion": "Check topic mapping in config.yaml. Available topics are listed in 'details'."
  }
}
```

**フィールド仕様**:
- `code`: string, エラーコード（上記の表を参照）
- `component`: string, エラーが発生したコンポーネント名
- `message`: string, 人間可読なエラーメッセージ
- `details`: object, エラーの詳細情報（任意）
- `suggestion`: string, 解決方法の提案（任意だが推奨）

---

## バージョン互換性

### インターフェースバージョニング

各JSONファイルに `version` フィールドを含める:
- MAJOR: 非互換な変更（スキーマ変更）
- MINOR: 後方互換な機能追加（新フィールド追加）
- PATCH: バグフィックス、ドキュメント修正

### 互換性マトリクス

| Interface Version | gs_world_builder Version | 備考 |
|------------------|------------------------|------|
| 1.0.0 | 0.1.x | 初版 |

---

## 変更履歴

| バージョン | 日付 | 変更内容 |
|-----------|------|---------|
| 1.0.0 | 2026-02-14 | 初版作成 |
