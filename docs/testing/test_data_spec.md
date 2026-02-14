# テストデータ仕様

**バージョン**: 1.0.0
**最終更新**: 2026-02-15

## 概要

このドキュメントは、GS-ROS2 Simulator のテストに使用するデータセットの仕様を定義します。

**目的**:
- 単体テスト・統合テスト用の再現可能なテストデータを提供
- エラーケースの検証用データを定義
- テストデータ生成方法を標準化

---

## 1. Minimal Test World Bundle

### 1-1. 概要

**目的**: 最小限の有効な world_bundle でテストを高速化

**仕様**:
- 場所: `worlds/minimal_test/`
- サイズ: 約10-20 MB
- 生成時間: < 5分

**用途**:
- world_bundle読み込みテスト
- ROS2起動テスト
- 基本的なシミュレーション動作テスト

---

### 1-2. ディレクトリ構造

```
worlds/minimal_test/
├── world.yaml
├── metadata.json
├── gaussians/
│   ├── background.splat.ply    # 100 Gaussians
│   └── render_config.json
├── geometry/
│   ├── heightmap.bin           # 128x128
│   ├── heightmap.yaml
│   └── drivable.geojson        # 1 rectangle
├── sensors/
│   ├── calibration.yaml        # 1 camera, 1 lidar
│   └── tf_static.json
└── sim/
    └── timebase.yaml
```

---

### 1-3. ファイル内容の詳細

#### world.yaml

```yaml
version: "1.0.0"
scene_id: "minimal_test"

metadata: "metadata.json"

gaussians:
  background: "gaussians/background.splat.ply"
  render_config: "gaussians/render_config.json"

geometry:
  heightmap_bin: "geometry/heightmap.bin"
  heightmap_yaml: "geometry/heightmap.yaml"
  drivable_geojson: "geometry/drivable.geojson"

sensors:
  calibration: "sensors/calibration.yaml"
  tf_static: "sensors/tf_static.json"

sim:
  timebase: "sim/timebase.yaml"
```

---

#### metadata.json

```json
{
  "version": "1.0.0",
  "scene_id": "minimal_test",
  "created_at": "2026-02-15T10:00:00Z",
  "builder_version": "0.1.0",

  "source": {
    "type": "synthetic",
    "description": "Minimal test world bundle for unit testing"
  },

  "coordinate_system": {
    "map_frame": "map",
    "odom_frame": "odom",
    "base_link_frame": "base_link",
    "convention": "ROS2 REP-103 (FLU)",
    "origin_lat": 35.681236,
    "origin_lon": 139.767125,
    "origin_alt": 0.0
  },

  "spatial_extent": {
    "min_x": -10.0,
    "max_x": 100.0,
    "min_y": -10.0,
    "max_y": 10.0,
    "min_z": -1.0,
    "max_z": 5.0,
    "units": "meters"
  },

  "sensors": {
    "cameras": ["front"],
    "lidars": ["top"]
  },

  "processing_info": {
    "gaussian_count": 100,
    "heightmap_resolution": 0.1,
    "drivable_area_m2": 2200.0
  }
}
```

---

#### gaussians/background.splat.ply

**仕様**:
- フォーマット: binary_little_endian
- Gaussian数: 100
- 配置: 平坦な地面上にランダム配置
- 色: グレー系（道路を模擬）

**生成スクリプト** (Python):

```python
import numpy as np
import struct

def generate_minimal_gaussians(output_path: str, count: int = 100):
    """最小限のGaussianデータを生成"""

    # ランダムシード固定（再現性）
    np.random.seed(42)

    # 位置: 地面上に配置 (x: 0-100, y: -10-10, z: 0)
    positions = np.random.uniform([0, -10, 0], [100, 10, 0.1], size=(count, 3))

    # スケール: 小さめ（0.5-1.0 m）
    scales = np.random.uniform(0.5, 1.0, size=(count, 3))

    # 回転: ほぼ単位クォータニオン
    rotations = np.tile([0, 0, 0, 1], (count, 1))

    # 不透明度: 0.8-1.0
    opacities = np.random.uniform(0.8, 1.0, size=(count, 1))

    # SH DC成分: グレー (0.4-0.6)
    sh_dc = np.random.uniform(0.4, 0.6, size=(count, 3))

    # SH高次成分: ゼロ (sh_degree=3 → 45個)
    sh_rest = np.zeros((count, 45), dtype=np.float32)

    # PLYヘッダ作成
    header = f"""ply
format binary_little_endian 1.0
element vertex {count}
property float x
property float y
property float z
property float scale_0
property float scale_1
property float scale_2
property float rot_0
property float rot_1
property float rot_2
property float rot_3
property float opacity
property float f_dc_0
property float f_dc_1
property float f_dc_2
"""

    # SH高次成分のプロパティ
    for i in range(45):
        header += f"property float f_rest_{i}\n"

    header += "end_header\n"

    # バイナリデータ作成
    with open(output_path, 'wb') as f:
        f.write(header.encode('ascii'))

        for i in range(count):
            # 各頂点データをバイナリで書き込み
            vertex_data = np.concatenate([
                positions[i],
                scales[i],
                rotations[i],
                opacities[i],
                sh_dc[i],
                sh_rest[i]
            ]).astype(np.float32)

            f.write(vertex_data.tobytes())

# 使用例
generate_minimal_gaussians("worlds/minimal_test/gaussians/background.splat.ply")
```

---

#### gaussians/render_config.json

```json
{
  "version": "1.0.0",
  "gaussian_format": "splat_ply",
  "sh_degree": 3,

  "color_correction": {
    "enabled": false
  },

  "background_mask": {
    "enabled": false
  }
}
```

---

#### geometry/heightmap.bin

**仕様**:
- サイズ: 128 x 128
- 解像度: 0.1 m/cell → 12.8m x 12.8m
- データ型: float32 (Little Endian)
- 高さ範囲: -1.0 ~ 0.1 m（ほぼ平坦な地面）

**生成スクリプト** (Python):

```python
import numpy as np

def generate_minimal_heightmap(output_bin: str, output_yaml: str):
    """最小限のheightmapを生成"""

    width, height = 128, 128
    resolution = 0.1  # meters/cell

    # ほぼ平坦な地面（z=0）、わずかな傾斜
    x = np.linspace(0, width-1, width)
    y = np.linspace(0, height-1, height)
    xx, yy = np.meshgrid(x, y)

    # わずかな傾斜: x方向に 0.0 → 0.1 m
    heightmap = (xx / width) * 0.1

    # ランダムノイズ追加 (±0.01m)
    np.random.seed(42)
    noise = np.random.uniform(-0.01, 0.01, size=(height, width))
    heightmap += noise

    # Little Endianで保存
    heightmap_flat = heightmap.astype(np.float32).flatten()

    with open(output_bin, 'wb') as f:
        f.write(heightmap_flat.tobytes())

    # YAMLファイル生成
    yaml_content = f"""# Heightmap metadata
width: {width}
height: {height}
resolution: {resolution}  # [m/cell]

# Origin: map frame の左下隅
origin:
  x: 0.0
  y: 0.0
  z: 0.0

# Data range
min_height: {heightmap.min():.6f}
max_height: {heightmap.max():.6f}

# Endianness
endian: little
"""

    with open(output_yaml, 'w') as f:
        f.write(yaml_content)

# 使用例
generate_minimal_heightmap(
    "worlds/minimal_test/geometry/heightmap.bin",
    "worlds/minimal_test/geometry/heightmap.yaml"
)
```

---

#### geometry/drivable.geojson

**仕様**:
- 走行可能領域: 1つの矩形ポリゴン
- 範囲: x: [0, 100], y: [-5, 5]

```json
{
  "type": "FeatureCollection",
  "crs": {
    "type": "name",
    "properties": {"name": "map_frame"}
  },
  "features": [
    {
      "type": "Feature",
      "geometry": {
        "type": "Polygon",
        "coordinates": [
          [
            [0.0, -5.0],
            [100.0, -5.0],
            [100.0, 5.0],
            [0.0, 5.0],
            [0.0, -5.0]
          ]
        ]
      },
      "properties": {
        "priority": 0,
        "area_m2": 1000.0
      }
    }
  ]
}
```

---

#### sensors/calibration.yaml

```yaml
# Sensor Calibration for minimal_test

cameras:
  front:
    frame_id: "camera_front"

    intrinsics:
      width: 640
      height: 480
      fx: 500.0
      fy: 500.0
      cx: 320.0
      cy: 240.0
      distortion_model: "radtan"
      k1: 0.0
      k2: 0.0
      p1: 0.0
      p2: 0.0

    extrinsics:
      translation: [1.5, 0.0, 1.2]  # base_link → camera
      rotation_quat: [0.0, 0.0, 0.0, 1.0]  # [x, y, z, w]

    rate_hz: 12.0

lidars:
  top:
    frame_id: "lidar_top"

    extrinsics:
      translation: [0.0, 0.0, 1.8]  # base_link → lidar
      rotation_quat: [0.0, 0.0, 0.0, 1.0]

    spec:
      model: "velodyne_vls128"
      channels: 32
      horizontal_resolution: 0.4  # [deg]
      vertical_fov: [-15.0, 15.0]  # [deg]
      max_range: 100.0  # [m]
      min_range: 0.5    # [m]

    rate_hz: 20.0
```

---

#### sensors/tf_static.json

```json
{
  "transforms": [
    {
      "parent_frame": "base_link",
      "child_frame": "camera_front",
      "translation": [1.5, 0.0, 1.2],
      "rotation_quat": [0.0, 0.0, 0.0, 1.0]
    },
    {
      "parent_frame": "base_link",
      "child_frame": "lidar_top",
      "translation": [0.0, 0.0, 1.8],
      "rotation_quat": [0.0, 0.0, 0.0, 1.0]
    }
  ]
}
```

---

#### sim/timebase.yaml

```yaml
# Simulation timebase configuration

dt: 0.01              # [s] シミュレーションタイムステップ (100Hz)
start_time: 0.0       # [s] 開始時刻

# センサ更新レート（calibration.yamlと一致）
sensor_rates:
  camera: 12.0          # [Hz]
  lidar: 20.0           # [Hz]

# 初期Ego位置
initial_pose:
  position: [10.0, 0.0, 0.0]
  orientation: [0.0, 0.0, 0.0, 1.0]  # quaternion [x,y,z,w]
  velocity: [0.0, 0.0, 0.0]
```

---

## 2. テスト用MCAP生成

### 2-1. 概要

**目的**: world_bundle生成テスト用の有効なMCAPファイルを提供

**仕様**:
- ファイル名: `test_data/minimal_test.mcap`
- 持続時間: 60秒
- センサ: 1カメラ、1 LiDAR、オドメトリ、TF

---

### 2-2. トピック仕様

| トピック名 | メッセージ型 | 周波数 | メッセージ数 |
|-----------|------------|--------|------------|
| `/camera/front/image_raw` | `sensor_msgs/Image` | 12Hz | 720 |
| `/camera/front/camera_info` | `sensor_msgs/CameraInfo` | 12Hz | 720 |
| `/lidar/top/points` | `sensor_msgs/PointCloud2` | 20Hz | 1200 |
| `/odom` | `nav_msgs/Odometry` | 100Hz | 6000 |
| `/tf` | `tf2_msgs/TFMessage` | 100Hz | 6000 |
| `/tf_static` | `tf2_msgs/TFMessage` | 1回 | 1 |

---

### 2-3. 生成スクリプト (Python + rosbag2)

```python
#!/usr/bin/env python3
"""
minimal_test.mcap 生成スクリプト

依存: ROS2, rosbag2, sensor_msgs, nav_msgs, tf2_msgs
"""

import rclpy
from rclpy.node import Node
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import numpy as np
from builtin_interfaces.msg import Time

class MinimalMCAPGenerator(Node):
    def __init__(self):
        super().__init__('minimal_mcap_generator')

        # rosbag2 writer setup
        storage_options = StorageOptions(
            uri='test_data/minimal_test',
            storage_id='mcap'
        )
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )

        self.writer = SequentialWriter()
        self.writer.open(storage_options, converter_options)

        # トピック登録
        self._create_topics()

    def _create_topics(self):
        """トピックを登録"""
        topics = [
            ('/camera/front/image_raw', 'sensor_msgs/msg/Image'),
            ('/camera/front/camera_info', 'sensor_msgs/msg/CameraInfo'),
            ('/lidar/top/points', 'sensor_msgs/msg/PointCloud2'),
            ('/odom', 'nav_msgs/msg/Odometry'),
            ('/tf', 'tf2_msgs/msg/TFMessage'),
            ('/tf_static', 'tf2_msgs/msg/TFMessage'),
        ]

        for topic_name, topic_type in topics:
            self.writer.create_topic({
                'name': topic_name,
                'type': topic_type,
                'serialization_format': 'cdr'
            })

    def generate_mcap(self, duration_sec: float = 60.0):
        """MCAPファイルを生成"""

        dt = 0.01  # 100Hz
        num_steps = int(duration_sec / dt)

        # tf_static を最初に書き込み
        self._write_tf_static(0.0)

        for i in range(num_steps):
            t = i * dt
            timestamp = self._to_ros_time(t)

            # Odometry + TF (100Hz)
            self._write_odom(timestamp, t)
            self._write_tf(timestamp, t)

            # Camera (12Hz)
            if i % 8 == 0:  # 100/12 ≈ 8
                self._write_camera(timestamp)

            # LiDAR (20Hz)
            if i % 5 == 0:  # 100/20 = 5
                self._write_lidar(timestamp)

        self.get_logger().info(f"Generated {num_steps} steps ({duration_sec}s)")

    def _to_ros_time(self, t_sec: float) -> Time:
        """秒をROS Timeに変換"""
        sec = int(t_sec)
        nanosec = int((t_sec - sec) * 1e9)
        return Time(sec=sec, nanosec=nanosec)

    def _write_tf_static(self, t: float):
        """tf_static を書き込み"""
        msg = TFMessage()

        # base_link → camera_front
        tf1 = TransformStamped()
        tf1.header.stamp = self._to_ros_time(t)
        tf1.header.frame_id = 'base_link'
        tf1.child_frame_id = 'camera_front'
        tf1.transform.translation.x = 1.5
        tf1.transform.translation.y = 0.0
        tf1.transform.translation.z = 1.2
        tf1.transform.rotation.w = 1.0

        # base_link → lidar_top
        tf2 = TransformStamped()
        tf2.header.stamp = self._to_ros_time(t)
        tf2.header.frame_id = 'base_link'
        tf2.child_frame_id = 'lidar_top'
        tf2.transform.translation.x = 0.0
        tf2.transform.translation.y = 0.0
        tf2.transform.translation.z = 1.8
        tf2.transform.rotation.w = 1.0

        msg.transforms = [tf1, tf2]

        self.writer.write('/tf_static', self.serialize_message(msg), int(t * 1e9))

    def _write_odom(self, timestamp: Time, t: float):
        """Odometry を書き込み（直進軌跡）"""
        msg = Odometry()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # 直進: x方向に 5 m/s
        x = 10.0 + 5.0 * t
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        msg.twist.twist.linear.x = 5.0

        self.writer.write('/odom', self.serialize_message(msg), timestamp.sec * int(1e9) + timestamp.nanosec)

    def _write_tf(self, timestamp: Time, t: float):
        """TF (map → odom → base_link) を書き込み"""
        msg = TFMessage()

        # map → odom (固定)
        tf1 = TransformStamped()
        tf1.header.stamp = timestamp
        tf1.header.frame_id = 'map'
        tf1.child_frame_id = 'odom'
        tf1.transform.rotation.w = 1.0

        # odom → base_link (odometryと同じ)
        tf2 = TransformStamped()
        tf2.header.stamp = timestamp
        tf2.header.frame_id = 'odom'
        tf2.child_frame_id = 'base_link'
        x = 10.0 + 5.0 * t
        tf2.transform.translation.x = x
        tf2.transform.rotation.w = 1.0

        msg.transforms = [tf1, tf2]

        self.writer.write('/tf', self.serialize_message(msg), timestamp.sec * int(1e9) + timestamp.nanosec)

    def _write_camera(self, timestamp: Time):
        """ダミーカメラ画像を書き込み"""
        # Image
        img = Image()
        img.header.stamp = timestamp
        img.header.frame_id = 'camera_front'
        img.width = 640
        img.height = 480
        img.encoding = 'rgb8'
        img.step = 640 * 3
        img.data = np.random.randint(0, 255, size=(480, 640, 3), dtype=np.uint8).tobytes()

        self.writer.write('/camera/front/image_raw', self.serialize_message(img), timestamp.sec * int(1e9) + timestamp.nanosec)

        # CameraInfo
        info = CameraInfo()
        info.header = img.header
        info.width = 640
        info.height = 480
        info.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.writer.write('/camera/front/camera_info', self.serialize_message(info), timestamp.sec * int(1e9) + timestamp.nanosec)

    def _write_lidar(self, timestamp: Time):
        """ダミーLiDAR点群を書き込み"""
        # 簡易的な点群（100点）
        points = np.random.uniform(-10, 10, size=(100, 3)).astype(np.float32)

        msg = PointCloud2()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'lidar_top'
        msg.height = 1
        msg.width = 100
        msg.is_dense = True
        msg.point_step = 12  # x,y,z (3 * 4 bytes)
        msg.row_step = 100 * 12
        msg.data = points.tobytes()

        self.writer.write('/lidar/top/points', self.serialize_message(msg), timestamp.sec * int(1e9) + timestamp.nanosec)

    def serialize_message(self, msg):
        """メッセージをシリアライズ"""
        from rclpy.serialization import serialize_message
        return serialize_message(msg)

def main():
    rclpy.init()
    generator = MinimalMCAPGenerator()
    generator.generate_mcap(duration_sec=60.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**使用方法**:
```bash
python3 generate_minimal_mcap.py
# 出力: test_data/minimal_test.mcap
```

---

## 3. エラーケース用テストデータ

### 3-1. 破損MCAP

**ファイル名**: `test_data/corrupted.mcap`

**生成方法**:
```bash
# 有効なMCAPを生成
cp test_data/minimal_test.mcap test_data/corrupted.mcap

# ファイルの途中を破損させる
dd if=/dev/zero of=test_data/corrupted.mcap bs=1024 count=10 seek=100 conv=notrunc
```

**期待される挙動**: 終了コード 2（MCAP読み込みエラー）

---

### 3-2. トピック不足MCAP

**ファイル名**: `test_data/missing_topics.mcap`

**内容**: カメラトピックのみ（LiDARなし）

**期待される挙動**: 終了コード 4（トピック不足エラー）

---

### 3-3. 不正なworld_bundle

#### a. 必須ファイル欠損

```bash
# heightmap.bin を削除
cp -r worlds/minimal_test worlds/invalid_missing_heightmap
rm worlds/invalid_missing_heightmap/geometry/heightmap.bin
```

**期待される挙動**: 起動時エラー（終了コード 2）

---

#### b. calibration と tf_static の不一致

`worlds/invalid_calibration_mismatch/sensors/calibration.yaml`:
```yaml
# camera_front の translation を変更
cameras:
  front:
    extrinsics:
      translation: [2.0, 0.5, 1.5]  # tf_static と異なる
```

**期待される挙動**: validation エラー

---

#### c. 空の drivable.geojson

```json
{
  "type": "FeatureCollection",
  "features": []
}
```

**期待される挙動**: 警告 + Degraded Mode（全エリアがoffroad）

---

## 4. テストデータ一覧

| データセット | 種別 | 用途 | 期待される結果 |
|------------|------|------|--------------|
| `worlds/minimal_test/` | 正常 | 基本動作テスト | 起動成功 |
| `test_data/minimal_test.mcap` | 正常 | world_bundle生成 | 生成成功 |
| `test_data/corrupted.mcap` | 異常 | エラーハンドリング | 終了コード 2 |
| `test_data/missing_topics.mcap` | 異常 | トピック検証 | 終了コード 4 |
| `worlds/invalid_missing_heightmap/` | 異常 | 必須ファイル検証 | 起動失敗 |
| `worlds/invalid_calibration_mismatch/` | 異常 | 一貫性検証 | validation エラー |
| `worlds/invalid_empty_drivable/` | 異常 | Degraded Mode | 警告 + 動作継続 |

---

## 5. データ生成の自動化

### 5-1. スクリプト構成

```
scripts/
├── generate_test_data.sh      # 全テストデータ生成（メインスクリプト）
├── generate_minimal_world.py  # minimal_test world_bundle 生成
├── generate_minimal_mcap.py   # minimal_test.mcap 生成
└── generate_error_cases.sh    # エラーケース用データ生成
```

---

### 5-2. generate_test_data.sh

```bash
#!/bin/bash
set -e

echo "=== GS-ROS2 Simulator テストデータ生成 ==="

# 1. minimal_test world_bundle
echo "[1/3] Generating minimal_test world_bundle..."
python3 scripts/generate_minimal_world.py

# 2. minimal_test.mcap
echo "[2/3] Generating minimal_test.mcap..."
python3 scripts/generate_minimal_mcap.py

# 3. エラーケース
echo "[3/3] Generating error case data..."
bash scripts/generate_error_cases.sh

echo "✅ テストデータ生成完了"
echo "  - worlds/minimal_test/"
echo "  - test_data/minimal_test.mcap"
echo "  - test_data/*（エラーケース）"
```

---

## 変更履歴

| バージョン | 日付 | 変更内容 |
|-----------|------|---------|
| 1.0.0 | 2026-02-15 | 初版作成 |
