# World Bundle データフォーマット契約

**バージョン**: 1.0.0
**最終更新**: 2026-02-14

## 概要

World Bundle は、`gs_world_builder` が生成し、`gs_ros2_simulator` が読み込む、
シーン単位の自己完結したデータセットです。

**準拠規約**: このスキーマは [`conventions.md`](../conventions.md) で定義された
共通規約（座標系、単位系、バイナリフォーマット等）に従います。

---

## 設計原則

1. **自己完結性**: ひとつの world_bundle で 1シーンが完全に再現可能
2. **ランタイム独立性**: シミュレータは world_bundle のみに依存（DriveStudio等への依存なし）
3. **バージョニング**: フォーマット変更に対応できるようバージョン情報を含む
4. **可搬性**: 相対パスで構成し、ディレクトリごと移動可能
5. **段階的拡張**: 必須ファイルと任意ファイルを明確に分離

---

## ディレクトリ構造

```
worlds/<scene_id>/
├── world.yaml              # 必須: エントリーポイント
├── metadata.json           # 必須: メタ情報
├── gaussians/              # 必須: 3DGS データ
│   ├── background.splat.ply    # または background.npz
│   └── render_config.json      # レンダリング設定
├── geometry/               # 必須: 走行用幾何
│   ├── heightmap.bin           # 地面高さマップ（2.5D）
│   ├── heightmap.yaml          # heightmap メタ情報
│   ├── drivable.geojson        # 走行可能領域
│   └── static_mesh.glb         # オプション: 精密メッシュ
├── sensors/                # 必須: センサ設定
│   ├── calibration.yaml        # センサキャリブレーション
│   └── tf_static.json          # 静的 TF
├── sim/                    # 必須: シミュレーション設定
│   └── timebase.yaml           # 時刻・レート設定
└── preview/                # オプション: プレビュー
    ├── overview.png            # 俯瞰図
    └── camera_front.mp4        # カメラプレビュー
```

---

## ファイル仕様

### 1. world.yaml（エントリーポイント）

**目的**: すべてのファイルパスを集約し、world_bundle の入口とする

```yaml
# World Bundle Format Version
version: "1.0.0"

# Scene identification
scene_id: "nuscenes_scene_0103"

# File paths (相対パス)
metadata: "metadata.json"

gaussians:
  background: "gaussians/background.splat.ply"
  render_config: "gaussians/render_config.json"

geometry:
  heightmap: "geometry/heightmap.bin"
  heightmap_meta: "geometry/heightmap.yaml"
  drivable: "geometry/drivable.geojson"
  static_mesh: "geometry/static_mesh.glb"  # optional

sensors:
  calibration: "sensors/calibration.yaml"
  tf_static: "sensors/tf_static.json"

sim:
  timebase: "sim/timebase.yaml"

preview:  # optional
  overview: "preview/overview.png"
  cameras:
    - "preview/camera_front.mp4"
```

---

### 2. metadata.json（メタ情報）

**目的**: 生成元情報、座標系、バージョンなどを記録

```json
{
  "version": "1.0.0",
  "scene_id": "nuscenes_scene_0103",
  "created_at": "2026-02-14T10:30:00Z",
  "builder_version": "gs_world_builder-0.1.0",

  "source": {
    "type": "mcap",
    "file": "run_2026_02_14.mcap",
    "md5": "a1b2c3d4...",
    "duration_sec": 120.5,
    "size_bytes": 1234567890
  },

  "coordinate_system": {
    "map_frame": "map",
    "odom_frame": "odom",
    "base_link_frame": "base_link",
    "convention": "ROS2 REP-103 (FLU)",
    "origin_lat": 35.681236,
    "origin_lon": 139.767125,
    "origin_alt": 40.0
  },

  "spatial_extent": {
    "min_x": -50.0,
    "max_x": 250.0,
    "min_y": -100.0,
    "max_y": 100.0,
    "min_z": -5.0,
    "max_z": 20.0,
    "units": "meters"
  },

  "sensors": {
    "cameras": ["front", "left", "right", "rear"],
    "lidars": ["top"]
  },

  "processing_info": {
    "drivesim_commit": "abc123...",
    "gs_training_iterations": 30000,
    "gs_training_config": "background_only.yaml"
  }
}
```

---

### 3. gaussians/ （3DGS データ）

#### 3-1. background.splat.ply（Gaussian データ）

**フォーマット**: PLY 形式

**PLY ヘッダ形式**: `format binary_little_endian 1.0` （推奨）
- ASCII形式（`format ascii 1.0`）も許容だが、ファイルサイズが大きくなる

**必須属性**:
- `x`, `y`, `z`: 位置 (float32) [m]
- `scale_0`, `scale_1`, `scale_2`: スケール (float32)
- `rot_0`, `rot_1`, `rot_2`, `rot_3`: 回転クォータニオン (float32) **[x, y, z, w] 順**
- `opacity`: 不透明度 (float32)
- `f_dc_0`, `f_dc_1`, `f_dc_2`: SH係数 DC成分 (float32) [R, G, B]
- `f_rest_*`: SH係数 高次成分（オプション, float32）

**詳細**: [`conventions.md`](../conventions.md) の「Gaussian形式」セクション参照

**代替フォーマット**: `.npz` (NumPy形式)も許容（キー名は conventions.md 参照）

---

#### 3-2. render_config.json（レンダリング設定）

```json
{
  "version": "1.0.0",
  "gaussian_format": "splat_ply",
  "sh_degree": 3,

  "color_correction": {
    "white_balance": [1.0, 1.0, 1.0],
    "exposure_compensation": 0.0,
    "gamma": 2.2
  },

  "rendering": {
    "background_color": [0.0, 0.0, 0.0],
    "near_plane": 0.1,
    "far_plane": 1000.0
  }
}
```

---

### 4. geometry/ （走行用幾何）

#### 4-1. heightmap.bin（地面高さマップ）

**フォーマット**: バイナリ（float32配列）

- **エンディアン**: Little Endian（[conventions.md](../conventions.md) 参照）
- **メモリレイアウト**: row-major (Y x X)
- **データ型**: IEEE 754 float32
- 各セルは高さ値 [m]
- **無効領域**: NaN (IEEE 754 NaN) で表現

**読み込み例（Python）**:
```python
import numpy as np
meta = yaml.safe_load(open("heightmap.yaml"))
data = np.fromfile("heightmap.bin", dtype=np.float32)
heightmap = data.reshape((meta['height'], meta['width']))
```

---

#### 4-2. heightmap.yaml（heightmap メタ情報）

```yaml
version: "1.0.0"

# グリッドサイズ
width: 2048           # X方向セル数
height: 2048          # Y方向セル数
resolution: 0.1       # [m/cell]

# 原点（map frame）
origin:
  x: -100.0           # [m]
  y: -100.0           # [m]
  z: 0.0              # [m] 基準高さ

# データ範囲
min_height: -5.0      # [m]
max_height: 15.0      # [m]
```

---

#### 4-3. drivable.geojson（走行可能領域）

**フォーマット**: GeoJSON (Polygon または MultiPolygon)

```json
{
  "type": "FeatureCollection",
  "crs": {
    "type": "name",
    "properties": {
      "name": "map_frame"
    }
  },
  "features": [
    {
      "type": "Feature",
      "geometry": {
        "type": "Polygon",
        "coordinates": [
          [
            [x0, y0],
            [x1, y1],
            [x2, y2],
            [x0, y0]
          ]
        ]
      },
      "properties": {
        "type": "drivable",
        "speed_limit": 50.0
      }
    }
  ]
}
```

**座標系**: map frame の X-Y 平面（Z は考慮しない）

---

#### 4-4. static_mesh.glb（オプション: 精密メッシュ）

**フォーマット**: glTF 2.0 Binary (.glb)

- 静的環境の三角メッシュ
- 高精度 LiDAR raycast や衝突判定に使用
- オプション：無い場合は heightmap で代用

---

### 5. sensors/ （センサ設定）

#### 5-1. calibration.yaml（センサキャリブレーション）

```yaml
version: "1.0.0"

cameras:
  front:
    frame_id: "camera_front"
    image_width: 1920
    image_height: 1080

    # カメラ座標系規約
    camera_convention: "opencv"  # "opencv" (X:右, Y:下, Z:前) or "ros" (X:右, Y:上, Z:後)

    intrinsics:
      model: "pinhole"  # or "fisheye"
      fx: 1500.0
      fy: 1500.0
      cx: 960.0
      cy: 540.0
      distortion_model: "radtan"  # or "equidistant"
      k1: 0.0
      k2: 0.0
      p1: 0.0
      p2: 0.0

    extrinsics:  # base_link -> camera
      translation: [2.0, 0.0, 1.5]  # [x, y, z]
      rotation_quat: [0.0, 0.0, 0.0, 1.0]  # [x, y, z, w]

    rate_hz: 12.0

  # left, right, rear も同様

lidars:
  top:
    frame_id: "lidar_top"

    extrinsics:  # base_link -> lidar
      translation: [0.0, 0.0, 2.0]
      rotation_quat: [0.0, 0.0, 0.0, 1.0]

    spec:
      model: "velodyne_vls128"
      channels: 128
      horizontal_resolution: 0.2  # [deg]
      vertical_fov: [-25.0, 15.0]  # [deg]
      max_range: 200.0  # [m]
      min_range: 0.5    # [m]

    rate_hz: 20.0
```

---

#### 5-2. tf_static.json（静的TF）

**フォーマット**: tf2_msgs/TFMessage の JSON 表現

```json
{
  "transforms": [
    {
      "header": {
        "frame_id": "base_link"
      },
      "child_frame_id": "camera_front",
      "transform": {
        "translation": {
          "x": 2.0,
          "y": 0.0,
          "z": 1.5
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

**重要: データ一貫性**

- **`calibration.yaml` が正**: tf_static.json は calibration.yaml から派生して生成される
- **食い違いは許容しない**: validation で不一致を検出した場合はエラー
- 詳細: [`conventions.md`](../conventions.md) の「データの一貫性規則」参照

---

### 6. sim/ （シミュレーション設定）

#### 6-1. timebase.yaml

```yaml
version: "1.0.0"

# シミュレーション時刻設定
simulation:
  dt: 0.01              # [s] シミュレーションタイムステップ (100Hz)
  start_time: 0.0       # [s] 開始時刻

# センサ更新レート（calibration.yamlと一致させる）
sensor_rates:
  camera: 12.0          # [Hz]
  lidar: 20.0           # [Hz]

# 初期Ego位置（オプション、空の場合は原点）
initial_pose:
  position: [0.0, 0.0, 0.0]
  orientation: [0.0, 0.0, 0.0, 1.0]  # quaternion [x,y,z,w]
  velocity: [0.0, 0.0, 0.0]
```

---

### 7. preview/ （オプション: プレビュー）

ビジュアル確認用のファイル群（シミュレータは使用しない）

- `overview.png`: シーン俯瞰図
- `camera_*.mp4`: 各カメラのプレビュー動画
- `trajectory.png`: Ego軌跡の可視化

---

## 依存関係マトリクス

| ファイル | gs_world_builder | gs_ros2_simulator | 備考 |
|---------|-----------------|------------------|------|
| world.yaml | 生成（必須） | 読込（必須） | エントリーポイント |
| metadata.json | 生成（必須） | 読込（任意） | メタ情報 |
| gaussians/* | 生成（必須） | 読込（必須） | カメラレンダリング |
| geometry/heightmap.* | 生成（必須） | 読込（必須） | 接地・LiDAR |
| geometry/drivable.geojson | 生成（必須） | 読込（必須） | 走行制約 |
| geometry/static_mesh.glb | 生成（任意） | 読込（任意） | 高精度用 |
| sensors/calibration.yaml | 生成（必須） | 読込（必須） | センサ設定 |
| sensors/tf_static.json | 生成（必須） | 読込（必須） | TF |
| sim/timebase.yaml | 生成（必須） | 読込（必須） | 時刻設定 |
| preview/* | 生成（任意） | 読込（なし） | 確認用 |

---

## バージョニング戦略

### フォーマットバージョン

各ファイルに `version` フィールドを持たせ、独立してバージョン管理

- **Semantic Versioning** (MAJOR.MINOR.PATCH)
- MAJOR: 非互換な変更
- MINOR: 後方互換な機能追加
- PATCH: バグフィックス

### 互換性チェック

シミュレータ起動時に `world.yaml` の `version` をチェックし、
サポート外の場合はエラーを出す。

```python
SUPPORTED_VERSIONS = ["1.0.0", "1.1.0"]
if world_version not in SUPPORTED_VERSIONS:
    raise ValueError(f"Unsupported world version: {world_version}")
```

---

## 拡張ポイント

### 将来追加される可能性のあるディレクトリ/ファイル

```
worlds/<scene_id>/
├── actors/                 # 動的アクター（車両・歩行者）
│   └── trajectories.json
├── hdmap/                  # HDマップ
│   └── lanelet2.osm
├── traffic/                # 交通信号・標識
│   └── signals.json
└── scenarios/              # シナリオ定義
    └── scenario_001.yaml
```

これらは現時点では **スコープ外** だが、拡張時に追加可能な設計とする。

---

## データサイズの目安

| ファイル | 典型的なサイズ |
|---------|--------------|
| background.splat.ply | 50-500 MB |
| heightmap.bin (2048x2048) | 16 MB |
| static_mesh.glb | 10-100 MB (任意) |
| その他 (yaml, json) | < 1 MB |
| **合計** | **100-600 MB/scene** |

---

## 検証ツール

`gs_world_builder` は world_bundle 生成後に検証スクリプトを提供すべき：

```bash
python -m gs_world_builder.validate /path/to/world_bundle
```

チェック項目:
- [ ] 必須ファイルの存在
- [ ] YAML/JSON の妥当性
- [ ] calibration と tf_static の一貫性
- [ ] heightmap のサイズと解像度の整合性
- [ ] drivable 領域の妥当性（空でない、など）
- [ ] Gaussian データの読み込み可能性

---

## 変更履歴

| バージョン | 日付 | 変更内容 |
|-----------|------|---------|
| 1.0.0 | 2026-02-14 | 初版作成 |
