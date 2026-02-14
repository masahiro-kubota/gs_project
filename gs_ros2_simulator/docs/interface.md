# gs_ros2_simulator 内部コンポーネント間インターフェース

**バージョン**: 1.0.1
**最終更新**: 2026-02-15

## 概要

このドキュメントは `gs_ros2_simulator` の内部コンポーネント間でやり取りされるすべてのデータの**厳密な仕様**を定義します。

**重要な区別**:
- **[architecture.md](architecture.md)** - 各コンポーネントの処理内容と実装方針
- **このドキュメント** - コンポーネント間のデータ契約（共通データ構造、データフォーマット）
- **[design.md](design.md)** - C++クラス・関数設計（インターフェース、実装詳細）

**準拠規約**: すべてのデータは [`../../docs/conventions.md`](../../docs/conventions.md) に従います。

---

## 共通データ構造

### 1. EgoPose（Ego位置姿勢）

**目的**: Ego車両の位置と姿勢を表現

**データ構造**:
```cpp
struct EgoPose {
  // Position [m] in map frame
  double x{0.0};
  double y{0.0};
  double z{0.0};

  // Orientation (quaternion: [x, y, z, w])
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};

  // Timestamp
  rclcpp::Time stamp;
};
```

**制約**:
- 座標系: map frame (ENU)
- クォータニオン正規化: `sqrt(qx² + qy² + qz² + qw²) = 1.0 ± 1e-6`
- `qw` の初期値は 1.0（単位クォータニオン）

**消費者**: すべてのコンポーネント

---

### 2. EgoTwist（Ego速度）

**目的**: Ego車両の並進・回転速度を表現

**データ構造**:
```cpp
struct EgoTwist {
  // Linear velocity [m/s] in base_link frame
  double vx{0.0};
  double vy{0.0};
  double vz{0.0};

  // Angular velocity [rad/s] in base_link frame
  double wx{0.0};
  double wy{0.0};
  double wz{0.0};
};
```

**制約**:
- 座標系: base_link frame (FLU)
- vx: 前進方向（正が前進）
- vy: 左方向（正が左）
- wz: ヨー角速度（正が左旋回）

**消費者**: VehicleDynamics, EgoStateCore, ROS2Bridge

---

### 3. EgoState（Ego状態）

**目的**: Ego車両の完全な状態を表現

**データ構造**:
```cpp
struct EgoState {
  EgoPose pose;
  EgoTwist twist;

  // Vehicle-specific state
  double steering_angle{0.0};  // [rad]
  double speed{0.0};            // [m/s]
};
```

**制約**:
- `steering_angle`: ステアリング角度（正が左）
- `speed`: 車速（スカラー、正が前進）
- `speed` と `twist.vx` は整合する必要がある

**消費者**: すべてのコンポーネント

---

### 4. ControlInput（制御入力）

**目的**: 車両制御入力を表現

**データ構造**:
```cpp
struct ControlInput {
  // Ackermann control
  double steering_angle{0.0};           // [rad]
  double steering_angle_velocity{0.0};  // [rad/s] (optional)
  double speed{0.0};                    // [m/s]
  double acceleration{0.0};             // [m/s^2]

  // Timestamp
  rclcpp::Time stamp;
};
```

**制約**:
- `steering_angle`: -max_steering_angle ~ +max_steering_angle
- `speed`: 0.0 ~ max_speed
- `stamp.nanoseconds() > 0` で有効
- タイムアウト判定: `(current_time - stamp).seconds() > timeout_sec`

**生成者**: ROS2Bridge（`/vehicle/control_cmd` から変換）
**消費者**: VehicleDynamics

---

### 5. WorldData（world_bundle データ）

**目的**: world_bundle の全情報を保持

**データ構造**:
```cpp
struct WorldData {
  // Metadata
  std::string scene_id;
  std::filesystem::path bundle_path;

  // File paths (relative to bundle_path)
  std::string gaussian_ply_path;
  std::string render_config_path;
  std::string heightmap_bin_path;
  std::string heightmap_yaml_path;
  std::string drivable_geojson_path;
  std::string calibration_yaml_path;
  std::string tf_static_json_path;
  std::string timebase_yaml_path;
  std::string static_mesh_glb_path;  // optional

  // Simulation config
  double dt{0.01};              // [s]
  double start_time{0.0};       // [s]
  EgoPose initial_pose;
  double initial_velocity{0.0}; // [m/s]
};
```

**制約**:
- すべてのファイルパスは `bundle_path` からの相対パス
- 必須ファイルは存在する必要がある
- `dt > 0.0`
- `static_mesh_glb_path` は任意

**生成者**: WorldLoader
**消費者**: すべてのコンポーネント

---

### 6. HeightmapData（heightmap データ）

**目的**: 地面高さマップデータを保持

**データ構造**:
```cpp
struct HeightmapData {
  int width{0};
  int height{0};
  double resolution{0.1};  // [m/cell]

  // Origin (map frame)
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_z{0.0};

  // Height range
  double min_height{0.0};
  double max_height{0.0};

  // Data (row-major: Y x X, Little Endian float32)
  std::vector<float> data;
};
```

**制約**:
- `data.size() == width * height`
- メモリレイアウト: row-major (Y x X)
- 無効領域: NaN (IEEE 754)
- データ範囲: min_height ≤ data[i] ≤ max_height (NaN除く)

**ファイルフォーマット**: [`../../docs/interfaces/world_bundle_schema.md#4-1`](../../docs/interfaces/world_bundle_schema.md#4-1) に準拠

**生成者**: GroundContact（ファイルから読み込み）
**消費者**: GroundContact, LiDARGenerator

---

### 7. DrivableArea（走行可能領域）

**目的**: 走行可能領域を表現

**データ構造**:
```cpp
struct DrivableArea {
  // GeoJSON polygons (in map frame, X-Y plane)
  std::vector<Eigen::MatrixX2d> polygons;

  // Priority (higher = more important)
  std::vector<int> priorities;
};
```

**制約**:
- 座標系: map frame の X-Y 平面
- 外周: 反時計回り (CCW)
- 穴: 時計回り (CW)
- `polygons.size() == priorities.size()`
- 最低1つのポリゴンが必要

**ファイルフォーマット**: [`../../docs/interfaces/world_bundle_schema.md#4-3`](../../docs/interfaces/world_bundle_schema.md#4-3) に準拠

**生成者**: EgoStateCore（ファイルから読み込み）
**消費者**: EgoStateCore

---

### 8. SimulationStatus（シミュレーション状態）

**目的**: シミュレーションの状態を報告

**データ構造**:
```cpp
struct SimulationStatus {
  bool is_collision{false};
  bool is_offroad{false};
  double elapsed_time{0.0};   // [s]
  std::string message;
};
```

**制約**:
- `is_collision`: 衝突検出（Phase 2 で実装）
- `is_offroad`: drivable 領域外
- `message`: 状態メッセージ（空文字列可）

**生成者**: EgoStateCore
**消費者**: ROS2Bridge

---

### 9. CameraIntrinsics（カメラ内部パラメータ）

**目的**: カメラの内部パラメータを保持

**データ構造**:
```cpp
struct CameraIntrinsics {
  std::string frame_id;
  int width{0};
  int height{0};

  // Pinhole model
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};

  // Distortion (radtan model)
  double k1{0.0};
  double k2{0.0};
  double p1{0.0};
  double p2{0.0};
};
```

**制約**:
- `width > 0 && height > 0`
- `fx > 0.0 && fy > 0.0`
- distortion coefficients は任意

**ファイルフォーマット**: [`../../docs/interfaces/world_bundle_schema.md#5-1`](../../docs/interfaces/world_bundle_schema.md#5-1) に準拠

**生成者**: CameraRenderer（calibration.yaml から読み込み）
**消費者**: CameraRenderer, ROS2Bridge

---

### 10. CameraExtrinsics（カメラ外部パラメータ）

**目的**: カメラの外部パラメータを保持

**データ構造**:
```cpp
struct CameraExtrinsics {
  // base_link -> camera transform
  Eigen::Isometry3d transform;
};
```

**制約**:
- `transform` は有効な剛体変換（回転 + 並進）
- 座標系: base_link → camera

**ファイルフォーマット**: [`../../docs/interfaces/world_bundle_schema.md#5-1`](../../docs/interfaces/world_bundle_schema.md#5-1) に準拠

**生成者**: CameraRenderer（calibration.yaml から読み込み）
**消費者**: CameraRenderer

---

### 11. RenderedImage（レンダリング画像）

**目的**: レンダリングされたカメラ画像を保持

**データ構造**:
```cpp
struct RenderedImage {
  std::string camera_id;
  int width{0};
  int height{0};
  std::vector<uint8_t> data;  // RGB8 format
  rclcpp::Time stamp;
};
```

**制約**:
- `data.size() == width * height * 3`
- RGB8: R, G, B の順で各チャンネル 0-255
- メモリレイアウト: row-major

**生成者**: CameraRenderer
**消費者**: ROS2Bridge

---

### 12. LiDARSpec（LiDAR仕様）

**目的**: LiDARの仕様を保持

**データ構造**:
```cpp
struct LiDARSpec {
  std::string frame_id;
  int channels{128};
  double horizontal_resolution{0.2};  // [deg]
  double vertical_fov_min{-25.0};     // [deg]
  double vertical_fov_max{15.0};      // [deg]
  double max_range{200.0};            // [m]
  double min_range{0.5};              // [m]
};
```

**制約**:
- `channels > 0`
- `horizontal_resolution > 0.0`
- `vertical_fov_min < vertical_fov_max`
- `min_range < max_range`

**ファイルフォーマット**: [`../../docs/interfaces/world_bundle_schema.md#5-1`](../../docs/interfaces/world_bundle_schema.md#5-1) に準拠

**生成者**: LiDARGenerator（calibration.yaml から読み込み）
**消費者**: LiDARGenerator

---

### 13. LiDARExtrinsics（LiDAR外部パラメータ）

**目的**: LiDARの外部パラメータを保持

**データ構造**:
```cpp
struct LiDARExtrinsics {
  // base_link -> lidar transform
  Eigen::Isometry3d transform;
};
```

**制約**:
- `transform` は有効な剛体変換（回転 + 並進）
- 座標系: base_link → lidar

**ファイルフォーマット**: [`../../docs/interfaces/world_bundle_schema.md#5-1`](../../docs/interfaces/world_bundle_schema.md#5-1) に準拠

**生成者**: LiDARGenerator（calibration.yaml から読み込み）
**消費者**: LiDARGenerator

---

### 14. GeneratedPointCloud（生成点群）

**目的**: 生成されたLiDAR点群を保持

**データ構造**:
```cpp
struct GeneratedPointCloud {
  std::string lidar_id;
  std::vector<Eigen::Vector4f> points;  // (x, y, z, intensity)
  rclcpp::Time stamp;
};
```

**制約**:
- `points` の各要素: (x, y, z, intensity)
  - x, y, z: [m], lidar frame
  - intensity: 0.0 ~ 255.0
- 座標系: lidar frame (FLU)
- 点数: 0 ~ 200,000

**生成者**: LiDARGenerator
**消費者**: ROS2Bridge

---

## センサトリガー仕様

### センサレート

各センサは `calibration.yaml` で定義されたレートで動作:

```yaml
sensors:
  cameras:
    front:
      rate_hz: 12.0   # 12Hz
  lidars:
    top:
      rate_hz: 20.0   # 20Hz
```

### トリガー判定

シミュレーション時刻 `t` でセンサがトリガーされるかどうかの判定:

```
period = 1.0 / rate_hz
next_trigger = floor(t / period) * period

should_trigger = |t - next_trigger| < dt / 2.0
```

**例**:
- Simulation: 100Hz (dt=10ms)
- Camera: 12Hz (83.3ms間隔)
- LiDAR: 20Hz (50ms間隔)

```
t=0ms:    sim, camera, lidar
t=10ms:   sim
t=20ms:   sim
...
t=50ms:   sim, lidar
...
t=83.3ms: sim, camera
```

---

## データ整合性ルール

### 1. クォータニオン正規化

すべてのクォータニオン（EgoPose 等）は正規化されている必要がある:

```cpp
bool is_normalized(double qx, double qy, double qz, double qw) {
  double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
  return std::abs(norm - 1.0) < 1e-6;
}
```

---

### 2. 座標系の一貫性

- **map frame**: ENU (East-North-Up)
- **base_link frame**: FLU (Forward-Left-Up)
- **camera/lidar frame**: カメラ/LiDAR固有（calibration.yaml で定義）

すべての変換は右手系を維持する必要がある。

---

### 3. タイムスタンプの一貫性

すべてのタイムスタンプは `/clock` と同期:

```cpp
bool is_sync_with_clock(const rclcpp::Time& stamp, const rclcpp::Time& clock_time) {
  return std::abs((stamp - clock_time).seconds()) < 0.001;  // 1ms以内
}
```

---

## エラー定義

### データ検証エラー

| エラー種別 | 説明 |
|----------|------|
| INVALID_QUATERNION | クォータニオンが正規化されていない |
| INVALID_TRANSFORM | 変換行列が有効な剛体変換でない |
| INVALID_TIMESTAMP | タイムスタンプが無効（0 または 負値） |
| INVALID_CONTROL_INPUT | 制御入力が範囲外 |
| INVALID_HEIGHTMAP_SIZE | heightmap のサイズが不一致 |
| INVALID_IMAGE_SIZE | 画像サイズが不一致 |
| INVALID_POINTCLOUD_SIZE | 点群サイズが異常 |

### エラーメッセージ形式

```
[COMPONENT_NAME] ERROR_TYPE: detail message
```

例:
```
[VehicleDynamics] INVALID_CONTROL_INPUT: steering_angle=0.6 exceeds max_steering_angle=0.52
```

---

## バージョン互換性

### インターフェースバージョニング

各データ構造に対応する world_bundle ファイルフォーマットは Semantic Versioning に従う:

- MAJOR: 非互換な変更（フィールド削除、型変更）
- MINOR: 後方互換な機能追加（新フィールド追加）
- PATCH: バグフィックス、ドキュメント修正

### 互換性マトリクス

| Interface Version | gs_ros2_simulator Version | world_bundle Version |
|------------------|--------------------------|---------------------|
| 1.0.0 | 0.1.x | 1.0.0 |

---

## 変更履歴

| バージョン | 日付 | 変更内容 |
|-----------|------|---------|
| 1.0.1 | 2026-02-15 | データ契約の明確化、architecture.md との整合性確保 |
| 1.0.0 | 2026-02-15 | 初版作成（interfaces.md から分割） |
