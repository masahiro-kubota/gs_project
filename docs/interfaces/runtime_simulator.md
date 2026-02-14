# ランタイムシミュレーション契約（world_bundle → ROS2）

**バージョン**: 1.0.2
**最終更新**: 2026-02-15

## 概要

`gs_ros2_simulator` は world_bundle を読み込み、ROS2 インターフェースで
カメラ・LiDAR・TF・Odom を配信し、制御入力を受け取って closed-loop を実現するシミュレータです。

**準拠規約**: この契約は [`conventions.md`](../conventions.md)、
[`world_bundle_schema.md`](world_bundle_schema.md)、および
[`ros2_messages.md`](ros2_messages.md) で定義された規約に従います。

**実装コンポーネント**: `gs_ros2_simulator`
**内部設計**: [`gs_ros2_simulator/docs/architecture.md`](../../gs_ros2_simulator/docs/architecture.md)

---

## 設計原則

1. **ROS2標準準拠**: ROS2 REP-103（座標系）、REP-105（tf）に準拠
2. **疎結合**: ADスタックと疎結合、標準メッセージで接続
3. **リアルタイム性**: use_sim_time=true 環境で安定動作
4. **再現性**: 同じ制御入力から同じ結果を生成
5. **拡張性**: センサ・dynamicsモデルの差し替えが容易

---

## 入力インターフェース

### 1. World Bundle（必須）

```
入力: world_bundle ディレクトリパス
フォーマット: world_bundle_schema.md に準拠
```

起動時パラメータで指定:
```bash
ros2 launch gs_ros2_simulator bringup.launch.xml world:=/path/to/world_bundle
```

---

### 2. ROS2 トピック（購読）

#### 制御入力

| トピック名 | メッセージ型 | 周波数 | QoS | 必須/任意 |
|-----------|------------|-------|-----|---------|
| `/vehicle/control_cmd` | `ackermann_msgs/AckermannDriveStamped` | ~10-100Hz | Reliable | 必須 |

**サポート対象メッセージ型**（パラメータで切替可能）:
- `ackermann_msgs/AckermannDriveStamped`（デフォルト）
- `geometry_msgs/TwistStamped`
- `gs_sim_msgs/VehicleControlCmd`

**メッセージ内容（Ackermann の場合）**:
```
std_msgs/Header header
ackermann_msgs/AckermannDrive drive
  float32 steering_angle          # [rad] ステア角度
  float32 steering_angle_velocity # [rad/s] ステア角速度（任意）
  float32 speed                   # [m/s] 目標速度
  float32 acceleration            # [m/s^2] 目標加速度
  float32 jerk                    # [m/s^3] 躍度（任意）
```

**タイムアウト**:
- 制御入力が途絶えた場合、車両は緊急停止（減速度 3.0 m/s²）（MUST）
- タイムアウト時間: デフォルト値 1.0秒（パラメータ `control_timeout_sec` で変更可能）（MUST）

---

### 3. ROS2 サービス（呼び出し可能）

| サービス名 | サービス型 | 説明 |
|-----------|-----------|------|
| `/sim/reset_world` | `gs_sim_msgs/srv/ResetWorld` | シミュレーションリセット |
| `/sim/set_ego_pose` | `gs_sim_msgs/srv/SetEgoPose` | Ego位置を強制設定（デバッグ用） |
| `/sim/pause` | `std_srvs/srv/Trigger` | 一時停止 |
| `/sim/resume` | `std_srvs/srv/Trigger` | 再開 |
| `/sim/step` | `std_srvs/srv/Trigger` | 1ステップだけ進める（一時停止中のみ） |

---

### 4. 起動時パラメータ

#### 必須パラメータ

```yaml
world: "/path/to/world_bundle"  # world_bundle のパス
```

#### 車両パラメータ

```yaml
vehicle:
  # 車両諸元
  wheelbase: 2.8                # [m] ホイールベース
  track_width: 1.6              # [m] トレッド幅
  mass: 1500.0                  # [kg] 車両重量
  inertia_z: 3000.0             # [kg*m^2] ヨー慣性モーメント

  # 制限値
  max_steering_angle: 0.52      # [rad] 最大ステア角（約30度）
  max_steering_rate: 0.5        # [rad/s] 最大ステア角速度
  max_speed: 30.0               # [m/s] 最大速度
  max_acceleration: 3.0         # [m/s^2] 最大加速度
  max_deceleration: 5.0         # [m/s^2] 最大減速度

  # ダイナミクスモデル
  dynamics_model: "kinematic_bicycle"  # "kinematic_bicycle" or "dynamic_bicycle"

  # タイヤパラメータ（dynamic_bicycle の場合のみ）
  tire:
    cornering_stiffness_front: 80000.0  # [N/rad]
    cornering_stiffness_rear: 80000.0   # [N/rad]
```

#### シミュレーションパラメータ

```yaml
simulation:
  # タイムステップ（world_bundle の timebase.yaml を上書き可能）
  dt: 0.01                      # [s] シミュレーションステップ（デフォルト値: 0.01 = 100Hz）

  # リアルタイム制御
  real_time_factor: 1.0         # 実時間の何倍で動かすか（デフォルト値: 1.0 = リアルタイム）
  max_step_time: 0.1            # [s] 1ステップの最大処理時間（デフォルト値: 0.1、超えたら警告）

  # 初期状態（world_bundle の timebase.yaml を上書き可能）
  initial_pose:
    x: 0.0
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  initial_velocity: 0.0         # [m/s]（デフォルト値: 0.0）

  # 制御
  control_timeout_sec: 1.0      # 制御入力タイムアウト（デフォルト値: 1.0）
  control_mode: "ackermann"     # デフォルト値: "ackermann"（"twist" または "custom" もサポート）
```

#### センサパラメータ

```yaml
sensors:
  # カメラ（world_bundle の calibration.yaml を基本とする）
  camera:
    enable: true  # デフォルト値: true
    rate_override: null         # [Hz] デフォルト値: null（null の場合は world_bundle の設定を使用）
    cameras: ["front"]          # 有効化するカメラ（省略時は全カメラ）

  # LiDAR
  lidar:
    enable: true  # デフォルト値: true
    rate_override: null  # デフォルト値: null
    lidars: ["top"]

    # Raycast設定
    raycast:
      use_heightmap: true       # デフォルト値: true
      use_static_mesh: false    # デフォルト値: false
      add_noise: false          # デフォルト値: false
      noise_std: 0.02           # [m] デフォルト値: 0.02
```

#### フレームID

```yaml
frames:
  map: "map"
  odom: "odom"
  base_link: "base_link"
  # カメラ・LiDARは world_bundle の calibration.yaml から自動設定
```

---

## 出力インターフェース

### 1. ROS2 トピック（配信）

#### Ego 状態

| トピック名 | メッセージ型 | 周波数 | QoS | 説明 |
|-----------|------------|-------|-----|------|
| `/tf` | `tf2_msgs/TFMessage` | 100Hz | Transient Local | 動的TF（map→odom→base_link） |
| `/tf_static` | `tf2_msgs/TFMessage` | latch | Transient Local | 静的TF（base_link→sensors） |
| `/odom` | `nav_msgs/Odometry` | 100Hz | Reliable | オドメトリ |
| `/vehicle/state` | `gs_sim_msgs/VehicleState` | 100Hz | Reliable | 詳細車両状態 |

---

#### シミュレーション状態

| トピック名 | メッセージ型 | 周波数 | QoS | 説明 |
|-----------|------------|-------|-----|------|
| `/clock` | `rosgraph_msgs/Clock` | 100Hz | Best Effort | シミュレーション時刻 |
| `/sim/status` | `gs_sim_msgs/SimulationStatus` | 10Hz | Reliable | シミュ状態・衝突・offroad等 |
| `/sim/world_info` | `gs_sim_msgs/WorldInfo` | latch | Transient Local | ワールド情報 |

---

#### センサ出力

##### カメラ

| トピック名 | メッセージ型 | 周波数 | QoS | 説明 |
|-----------|------------|-------|-----|------|
| `/camera/{camera_id}/image_raw` | `sensor_msgs/Image` | 12Hz* | Reliable | カメラ画像（RGB8） |
| `/camera/{camera_id}/camera_info` | `sensor_msgs/CameraInfo` | 12Hz* | Reliable | カメラ情報 |

*レートは world_bundle の `calibration.yaml` で設定

**Image メッセージ**:
```
std_msgs/Header header
  uint32 seq
  builtin_interfaces/Time stamp  # /clock と同期
  string frame_id                # "camera_front" etc

uint32 height                    # 画像高さ
uint32 width                     # 画像幅
string encoding                  # "rgb8"
uint8 is_bigendian
uint32 step
uint8[] data                     # 画像データ
```

---

##### LiDAR

| トピック名 | メッセージ型 | 周波数 | QoS | 説明 |
|-----------|------------|-------|-----|------|
| `/lidar/{lidar_id}/points` | `sensor_msgs/PointCloud2` | 20Hz* | Reliable | LiDAR点群 |

*レートは world_bundle の `calibration.yaml` で設定

**PointCloud2 フィールド**:
```
fields:
  - name: "x"
    offset: 0
    datatype: 7  # FLOAT32
    count: 1
  - name: "y"
    offset: 4
    datatype: 7
    count: 1
  - name: "z"
    offset: 8
    datatype: 7
    count: 1
  - name: "intensity"
    offset: 12
    datatype: 7
    count: 1
  - name: "ring"           # オプション
    offset: 16
    datatype: 4  # UINT16
    count: 1
```

---

### 2. TF ツリー

```
map
 └─ odom
     └─ base_link
         ├─ camera_front
         ├─ camera_left
         ├─ camera_right
         ├─ camera_rear
         └─ lidar_top
```

**TF 配信周波数**:
- `map` → `odom`: 100Hz（通常は単位行列、将来的にドリフトを模擬可能）
- `odom` → `base_link`: 100Hz（ego の pose/twist）
- `base_link` → センサ: latch（静的TF、world_bundle から読み込み）

---

## 起動インターフェース

### 1. Launch ファイル

#### bringup.launch.xml（デフォルト起動方法）

```bash
ros2 launch gs_ros2_simulator bringup.launch.xml \
    world:=/path/to/world_bundle \
    vehicle_config:=config/vehicle.yaml \
    sim_config:=config/simulation.yaml
```

**Launch 引数**:
| 引数 | 型 | デフォルト | 説明 |
|-----|---|-----------|------|
| `world` | string | MUST | world_bundle パス |
| `vehicle_config` | string | デフォルト値: `config/default_vehicle.yaml` | 車両パラメータ |
| `sim_config` | string | デフォルト値: `config/default_sim.yaml` | シミュレーションパラメータ |
| `sensor_config` | string | デフォルト値: `config/default_sensors.yaml` | センサパラメータ |
| `use_sim_time` | bool | デフォルト値: `true` | シミュレーション時刻使用 |
| `rviz` | bool | デフォルト値: `true` | RViz起動 |
| `log_level` | string | デフォルト値: `info` | ログレベル（debug/info/warn/error） |

---

---

### 2. ノード構成

#### 単一ノード構成（デフォルト推奨構成: 初期実装）

```
simulator_node
  ├─ WorldLoader
  ├─ SimClock
  ├─ EgoStateCore
  ├─ VehicleDynamics
  ├─ GroundContact
  ├─ CameraRenderer
  ├─ LiDARGenerator
  └─ ROS2Bridge
```

すべてを1つのプロセスに統合し、レイテンシを最小化。

---

#### マルチノード構成（将来的な拡張）

```
world_state_node          # Ego状態管理
vehicle_dynamics_node     # 車両運動
camera_renderer_node      # カメラレンダリング
lidar_generator_node      # LiDAR生成
sim_clock_node            # /clock配信
```

ノード間はトピックで疎結合、個別に差し替え可能。

---

## 動作モード

### 1. Normal モード（デフォルト動作モード）

```bash
ros2 launch gs_ros2_simulator bringup.launch.xml world:=worlds/scene_001
```

- リアルタイムで動作（デフォルト値: real_time_factor=1.0）
- 制御入力を受け付け、closed-loop で動作

---

### 2. Fast モード

```bash
ros2 launch gs_ros2_simulator bringup.launch.xml \
    world:=worlds/scene_001 \
    real_time_factor:=0.0  # 最速
```

- CPUが許す限り高速実行
- データ生成・テスト用

---

### 3. Step モード（デバッグ用）

```bash
# 起動
ros2 launch gs_ros2_simulator bringup.launch.xml \
    world:=worlds/scene_001 \
    start_paused:=true

# 1ステップずつ進める
ros2 service call /sim/step std_srvs/srv/Trigger
```

---

### 4. Replay モード（将来拡張）

事前記録した制御入力を再生:
```bash
ros2 launch gs_ros2_simulator bringup.launch.xml \
    world:=worlds/scene_001 \
    replay_control:=/path/to/control.mcap
```

---

## データフロー

### 周期的更新

```
Time: t0, t1, t2, ...

Each step (dt = 10ms):
  1. /clock publish (t)
  2. Read /vehicle/control_cmd
  3. Update vehicle dynamics (t → t+dt)
  4. Update ego pose/twist
  5. Ground contact & collision check
  6. Publish /tf, /odom, /vehicle/state
  7. If sensor_trigger(t):
       7a. Render camera image
       7b. Generate LiDAR points
       7c. Publish sensor topics
  8. Publish /sim/status
  9. Sleep to match real_time_factor
```

---

### センサトリガー

カメラ・LiDARは `calibration.yaml` のレートに従って更新:

```
Simulation: 100Hz (dt=10ms)
Camera: 12Hz (83.3ms間隔)
LiDAR: 20Hz (50ms間隔)

Timeline:
t=0ms:    sim, camera, lidar
t=10ms:   sim
t=20ms:   sim
...
t=50ms:   sim, lidar
...
t=83.3ms: sim, camera
```

---

## QoS プロファイル

| トピック種別 | Reliability | Durability | History | Depth |
|------------|-------------|-----------|---------|-------|
| センサ（Camera/LiDAR） | Reliable | Volatile | Keep Last | 5 |
| TF | Best Effort | Transient Local | Keep Last | 100 |
| TF Static | Reliable | Transient Local | Keep Last | 100 |
| Odom | Reliable | Volatile | Keep Last | 10 |
| Clock | Best Effort | Volatile | Keep Last | 1 |
| Control Input | Reliable | Volatile | Keep Last | 10 |
| Status | Reliable | Volatile | Keep Last | 10 |
| World Info | Reliable | Transient Local | Keep Last | 1 |

---

## エラーハンドリング

### 起動時エラー

| エラー | 終了コード | 対処 |
|-------|----------|------|
| World bundle not found | 1 | パス確認 |
| World bundle invalid | 2 | validate で検証 |
| Required file missing | 3 | world_bundle 再生成 |
| GPU not available | 4 | CUDA/Vulkan確認 |

---

### ランタイムエラー

| エラー | 挙動 | ログレベル |
|-------|-----|-----------|
| Control timeout | 緊急停止、警告ログ | WARN |
| Rendering failure | 前フレーム再利用、エラーログ | ERROR |
| Collision detected | `/sim/status` でフラグ設定 | INFO |
| Offroad detected | `/sim/status` でフラグ設定 | WARN |
| Step timeout | 警告ログ、real_time_factor低下 | WARN |

---

## パフォーマンス要件

### リアルタイム性

- **目標**: real_time_factor ≥ 1.0 で安定動作
- **必須システム要件**:
  - GPU: NVIDIA RTX 3060 以上（CUDA 11.8+）（MUST）
  - CPU: 4コア以上（MUST）
  - RAM: 8GB 以上（MUST）
  - ストレージ: SSD（HDDは性能要件を満たさない可能性あり）（MUST）

### 処理時間目安（RTX 4090の場合）

| 処理 | 時間 | 備考 |
|-----|------|------|
| Vehicle dynamics update | < 0.1ms | CPU |
| Ground contact check | < 0.5ms | CPU |
| Camera rendering (1920x1080) | < 5ms | GPU |
| LiDAR generation (128ch, raycast) | < 2ms | GPU |
| LiDAR generation (128ch, Gaussian)* | < 10ms | GPU（将来） |
| **Total per step** | **< 10ms** | **100Hz達成** |

---

## 依存パッケージ

### ROS2依存

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
<depend>geometry_msgs</depend>
<depend>tf2</depend>
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>
<depend>ackermann_msgs</depend>
<depend>rosgraph_msgs</depend>
<depend>std_srvs</depend>
<depend>gs_sim_msgs</depend>
```

### システム依存

```bash
# Ubuntu/Debian
apt-get install -y \
    ros-humble-desktop \
    ros-humble-ackermann-msgs \
    nvidia-cuda-toolkit \
    libyaml-cpp-dev \
    libeigen3-dev
```

### ビルド依存

```cmake
# CMakeLists.txt
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gs_sim_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# 3DGS rendering (選択)
find_package(CUDA REQUIRED)       # gsplat/CUDA backend
# OR
find_package(Vulkan REQUIRED)     # Vulkan backend
```

---

## テスト・検証

### 単体テスト（MUST）

```bash
colcon test --packages-select gs_ros2_simulator
```

テスト項目:
- [ ] World bundle 読み込み
- [ ] Vehicle dynamics（kinematic/dynamic）
- [ ] Ground contact & heightmap
- [ ] Drivable area check
- [ ] TF tree構築
- [ ] Sensor trigger timing

---

### 統合テスト

```bash
# ダミー制御でシミュレータ起動
ros2 launch gs_ros2_simulator test_dummy_control.launch.xml \
    world:=worlds/test_scene
```

検証項目:
- [ ] /clock が配信されている
- [ ] /tf, /odom が配信されている
- [ ] Camera image が配信されている
- [ ] LiDAR points が配信されている
- [ ] 制御入力に応じて ego が動く
- [ ] 衝突・offroad 検出が動作する

---

## 拡張ポイント

### Vehicle Dynamics の差し替え

```cpp
class VehicleDynamicsInterface {
public:
  virtual void update(double dt, const ControlInput& input) = 0;
  virtual EgoState getState() const = 0;
};

class KinematicBicycle : public VehicleDynamicsInterface { ... };
class DynamicBicycle : public VehicleDynamicsInterface { ... };
```

---

### LiDAR Generator の差し替え

```cpp
class LiDARGeneratorInterface {
public:
  virtual PointCloud2 generate(const EgoPose& pose) = 0;
};

class RaycastLiDAR : public LiDARGeneratorInterface { ... };
class GaussianLiDAR : public LiDARGeneratorInterface { ... };  // 将来
```

---

## 変更履歴

| バージョン | 日付 | 変更内容 |
|-----------|------|---------|
| 1.0.2 | 2026-02-15 | 曖昧な表現を明確化（推奨→デフォルト値、必須→MUST） |
| 1.0.1 | 2026-02-15 | 内部設計リンク追加、スキーマリファレンス修正 |
| 1.0.0 | 2026-02-14 | 初版作成 |
