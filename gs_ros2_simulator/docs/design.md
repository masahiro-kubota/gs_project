# gs_ros2_simulator 実装設計

**バージョン**: 1.0.1
**最終更新**: 2026-02-15

## 概要

このドキュメントは、`gs_ros2_simulator` の**実装レベルの設計**（C++クラス構成、関数シグネチャ）を定義します。

**ドキュメントの関係**:
- **[architecture.md](architecture.md)** - 各コンポーネントの責務・処理内容
- **[interface.md](interface.md)** - コンポーネント間のデータ契約
- **このドキュメント** - 実装設計（C++クラス・関数）

**言語**: C++17 以上（型ヒント、スマートポインタ必須）

---

## 設計原則

1. **抽象化**: インターフェースと実装を分離（差し替え可能）
2. **型安全性**: 強い型付けで実装ミスを防止
3. **テスタビリティ**: 各コンポーネントを独立してテスト可能
4. **ヘッダーオンリー**: 軽量なインターフェースはヘッダーのみで定義
5. **明確な所有権**: スマートポインタで所有権を明示

---

## コンポーネントインターフェース

### 1. WorldLoader

**責務**: world_bundle を読み込み、検証する

**ヘッダー**: `gs_sim/world_loader.hpp`

```cpp
namespace gs_sim {

class WorldLoaderInterface {
public:
  virtual ~WorldLoaderInterface() = default;

  /**
   * @brief Load and validate world_bundle
   * @param bundle_path Path to world_bundle directory
   * @return WorldData structure
   * @throws WorldLoadException if loading fails
   */
  virtual WorldData load(const std::filesystem::path& bundle_path) = 0;

  /**
   * @brief Validate world_bundle
   * @param world_data WorldData to validate
   * @return true if valid, false otherwise
   */
  virtual bool validate(const WorldData& world_data) const = 0;
};

// Factory
std::unique_ptr<WorldLoaderInterface> createWorldLoader();

}  // namespace gs_sim
```

**実装クラス**:
```cpp
class WorldLoader : public WorldLoaderInterface {
public:
  WorldData load(const std::filesystem::path& bundle_path) override;
  bool validate(const WorldData& world_data) const override;

private:
  void loadWorldYaml(const std::filesystem::path& yaml_path, WorldData& world_data);
  void loadMetadataJson(const std::filesystem::path& json_path, WorldData& world_data);
  void loadTimebaseYaml(const std::filesystem::path& yaml_path, WorldData& world_data);
  void checkFileExistence(const WorldData& world_data) const;
  void checkVersionCompatibility(const WorldData& world_data) const;
};
```

**実装ノート**:
- yaml-cpp ライブラリを使用してYAML読み込み
- nlohmann/json ライブラリを使用してJSON読み込み
- 検証失敗時は `WorldLoadException` をthrow

---

### 2. SimClock

**責務**: シミュレーション時刻を管理する

**ヘッダー**: `gs_sim/sim_clock.hpp`

```cpp
namespace gs_sim {

class SimClockInterface {
public:
  virtual ~SimClockInterface() = default;

  /**
   * @brief Initialize clock
   * @param start_time Initial simulation time [s]
   * @param dt Simulation time step [s]
   * @param real_time_factor Real-time factor (1.0 = real-time, 0.0 = fast)
   */
  virtual void initialize(double start_time, double dt, double real_time_factor) = 0;

  /**
   * @brief Advance simulation time by one step
   * @return Current simulation time
   */
  virtual rclcpp::Time step() = 0;

  /**
   * @brief Get current simulation time
   */
  virtual rclcpp::Time now() const = 0;

  /**
   * @brief Get time step
   */
  virtual double dt() const = 0;

  /**
   * @brief Pause simulation
   */
  virtual void pause() = 0;

  /**
   * @brief Resume simulation
   */
  virtual void resume() = 0;

  /**
   * @brief Check if simulation is paused
   */
  virtual bool isPaused() const = 0;

  /**
   * @brief Sleep to match real-time factor
   */
  virtual void sleep() = 0;
};

std::unique_ptr<SimClockInterface> createSimClock();

}  // namespace gs_sim
```

**実装ノート**:
- `std::chrono::high_resolution_clock` を使用
- `sleep()` は `std::this_thread::sleep_for()` を使用
- Paused 時は `step()` で時刻を進めない

---

### 3. VehicleDynamics

**責務**: 制御入力から車両運動を計算する

**ヘッダー**: `gs_sim/vehicle_dynamics.hpp`

```cpp
namespace gs_sim {

class VehicleDynamicsInterface {
public:
  virtual ~VehicleDynamicsInterface() = default;

  /**
   * @brief Initialize dynamics model
   * @param wheelbase Wheelbase [m]
   * @param max_steering_angle Maximum steering angle [rad]
   * @param max_speed Maximum speed [m/s]
   */
  virtual void initialize(double wheelbase, double max_steering_angle, double max_speed) = 0;

  /**
   * @brief Update vehicle state
   * @param dt Time step [s]
   * @param input Control input
   * @param current_state Current vehicle state (input/output)
   */
  virtual void update(double dt, const ControlInput& input, EgoState& current_state) = 0;

  /**
   * @brief Reset to initial state
   * @param initial_pose Initial pose
   * @param initial_velocity Initial velocity [m/s]
   */
  virtual void reset(const EgoPose& initial_pose, double initial_velocity) = 0;
};

// Factory
std::unique_ptr<VehicleDynamicsInterface> createKinematicBicycle();
std::unique_ptr<VehicleDynamicsInterface> createDynamicBicycle();

}  // namespace gs_sim
```

**実装クラス例（KinematicBicycle）**:
```cpp
class KinematicBicycle : public VehicleDynamicsInterface {
public:
  void initialize(double wheelbase, double max_steering_angle, double max_speed) override;
  void update(double dt, const ControlInput& input, EgoState& current_state) override;
  void reset(const EgoPose& initial_pose, double initial_velocity) override;

private:
  double wheelbase_{2.8};
  double max_steering_angle_{0.52};
  double max_speed_{30.0};

  double clampSteering(double steering) const;
  double clampSpeed(double speed) const;
  void integrateRK4(double dt, const ControlInput& input, EgoState& state);
};
```

**ダイナミクスモデル（Kinematic Bicycle）**:
```
dx/dt = v * cos(θ)
dy/dt = v * sin(θ)
dθ/dt = (v / L) * tan(δ)

where:
  v: speed
  θ: yaw angle
  δ: steering angle
  L: wheelbase
```

**実装ノート**:
- Runge-Kutta 4次（RK4）を推奨
- ステアリング・速度のクランプは必須
- タイムアウト時は緊急停止（減速度 3.0 m/s²）

---

### 4. GroundContact

**責務**: heightmap から接地点の高さを取得する

**ヘッダー**: `gs_sim/ground_contact.hpp`

```cpp
namespace gs_sim {

class GroundContactInterface {
public:
  virtual ~GroundContactInterface() = default;

  /**
   * @brief Load heightmap from file
   * @param bin_path Path to heightmap.bin
   * @param yaml_path Path to heightmap.yaml
   */
  virtual void loadHeightmap(const std::filesystem::path& bin_path,
                              const std::filesystem::path& yaml_path) = 0;

  /**
   * @brief Get ground height at (x, y)
   * @param x X coordinate in map frame [m]
   * @param y Y coordinate in map frame [m]
   * @return Ground height [m], or NaN if invalid
   */
  virtual double getGroundHeight(double x, double y) const = 0;

  /**
   * @brief Check if (x, y) is on valid ground
   */
  virtual bool isOnGround(double x, double y) const = 0;
};

std::unique_ptr<GroundContactInterface> createGroundContact();

}  // namespace gs_sim
```

**双線形補間**:
```cpp
double bilinearInterpolation(
    const HeightmapData& heightmap,
    double x, double y) {
  // グリッドセルの特定
  int grid_x = static_cast<int>((x - heightmap.origin_x) / heightmap.resolution);
  int grid_y = static_cast<int>((y - heightmap.origin_y) / heightmap.resolution);

  // 範囲外チェック
  if (grid_x < 0 || grid_x >= heightmap.width - 1 ||
      grid_y < 0 || grid_y >= heightmap.height - 1) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  // 4つの頂点
  float h00 = heightmap.data[grid_y * heightmap.width + grid_x];
  float h10 = heightmap.data[grid_y * heightmap.width + (grid_x + 1)];
  float h01 = heightmap.data[(grid_y + 1) * heightmap.width + grid_x];
  float h11 = heightmap.data[(grid_y + 1) * heightmap.width + (grid_x + 1)];

  // NaNチェック
  if (std::isnan(h00) || std::isnan(h10) || std::isnan(h01) || std::isnan(h11)) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  // 補間係数
  double fx = (x - (heightmap.origin_x + grid_x * heightmap.resolution)) / heightmap.resolution;
  double fy = (y - (heightmap.origin_y + grid_y * heightmap.resolution)) / heightmap.resolution;

  // 双線形補間
  double h0 = h00 * (1.0 - fx) + h10 * fx;
  double h1 = h01 * (1.0 - fx) + h11 * fx;
  return h0 * (1.0 - fy) + h1 * fy;
}
```

**実装ノート**:
- バイナリファイルの読み込みは Little Endian
- 無効領域は NaN で表現

---

### 5. EgoStateCore

**責務**: Ego状態を管理し、衝突・offroad判定を行う

**ヘッダー**: `gs_sim/ego_state_core.hpp`

```cpp
namespace gs_sim {

class EgoStateCoreInterface {
public:
  virtual ~EgoStateCoreInterface() = default;

  /**
   * @brief Initialize with drivable area
   * @param drivable_geojson_path Path to drivable.geojson
   */
  virtual void initialize(const std::filesystem::path& drivable_geojson_path) = 0;

  /**
   * @brief Update ego state
   * @param new_state New ego state from VehicleDynamics
   */
  virtual void updateState(const EgoState& new_state) = 0;

  /**
   * @brief Get current ego state
   */
  virtual const EgoState& getState() const = 0;

  /**
   * @brief Check collision and offroad
   * @return Simulation status
   */
  virtual SimulationStatus checkStatus() const = 0;

  /**
   * @brief Reset to initial state
   * @param initial_pose Initial pose
   * @param initial_velocity Initial velocity [m/s]
   */
  virtual void reset(const EgoPose& initial_pose, double initial_velocity) = 0;
};

std::unique_ptr<EgoStateCoreInterface> createEgoStateCore();

}  // namespace gs_sim
```

**実装ノート**:
- Shapely (Python) または GEOS (C++) ライブラリを使用してポリゴン内部判定
- Offroad検出時は WARN ログ

---

### 6. CameraRenderer

**責務**: 3DGS を使ってカメラ画像をレンダリングする

**ヘッダー**: `gs_sim/camera_renderer.hpp`

```cpp
namespace gs_sim {

class CameraRendererInterface {
public:
  virtual ~CameraRendererInterface() = default;

  /**
   * @brief Load Gaussian data and initialize renderer
   * @param gaussian_ply_path Path to background.splat.ply
   * @param render_config_path Path to render_config.json
   */
  virtual void loadGaussians(const std::filesystem::path& gaussian_ply_path,
                              const std::filesystem::path& render_config_path) = 0;

  /**
   * @brief Add camera
   * @param camera_id Camera ID (e.g., "front")
   * @param intrinsics Camera intrinsics
   * @param extrinsics Camera extrinsics (base_link -> camera)
   */
  virtual void addCamera(const std::string& camera_id,
                          const CameraIntrinsics& intrinsics,
                          const CameraExtrinsics& extrinsics) = 0;

  /**
   * @brief Render image from camera
   * @param camera_id Camera ID
   * @param ego_pose Current ego pose
   * @param stamp Timestamp
   * @return Rendered image (RGB8)
   */
  virtual RenderedImage render(const std::string& camera_id,
                                const EgoPose& ego_pose,
                                const rclcpp::Time& stamp) = 0;

  /**
   * @brief Get list of camera IDs
   */
  virtual std::vector<std::string> getCameraIds() const = 0;
};

std::unique_ptr<CameraRendererInterface> createCameraRenderer();

}  // namespace gs_sim
```

**実装ノート**:
- 3DGS レンダリングライブラリ候補:
  - gsplat（Python/CUDA）
  - nerfstudio-gsplat（C++/CUDA）
  - Vulkan backend（将来）
- GPU 上で高速レンダリング（目標: 5ms/frame）
- レンダリング失敗時は前フレーム再利用（ERROR ログ）

---

### 7. LiDARGenerator

**責務**: heightmap または static_mesh から LiDAR 点群を生成する

**ヘッダー**: `gs_sim/lidar_generator.hpp`

```cpp
namespace gs_sim {

class LiDARGeneratorInterface {
public:
  virtual ~LiDARGeneratorInterface() = default;

  /**
   * @brief Load heightmap for raycast
   * @param heightmap_bin_path Path to heightmap.bin
   * @param heightmap_yaml_path Path to heightmap.yaml
   */
  virtual void loadHeightmap(const std::filesystem::path& heightmap_bin_path,
                              const std::filesystem::path& heightmap_yaml_path) = 0;

  /**
   * @brief Load static mesh for raycast (optional)
   * @param static_mesh_glb_path Path to static_mesh.glb
   */
  virtual void loadStaticMesh(const std::filesystem::path& static_mesh_glb_path) = 0;

  /**
   * @brief Add LiDAR
   * @param lidar_id LiDAR ID (e.g., "top")
   * @param spec LiDAR specification
   * @param extrinsics LiDAR extrinsics (base_link -> lidar)
   */
  virtual void addLiDAR(const std::string& lidar_id,
                        const LiDARSpec& spec,
                        const LiDARExtrinsics& extrinsics) = 0;

  /**
   * @brief Generate point cloud
   * @param lidar_id LiDAR ID
   * @param ego_pose Current ego pose
   * @param stamp Timestamp
   * @return Generated point cloud
   */
  virtual GeneratedPointCloud generate(const std::string& lidar_id,
                                        const EgoPose& ego_pose,
                                        const rclcpp::Time& stamp) = 0;

  /**
   * @brief Get list of LiDAR IDs
   */
  virtual std::vector<std::string> getLiDARIds() const = 0;
};

std::unique_ptr<LiDARGeneratorInterface> createRaycastLiDAR();
std::unique_ptr<LiDARGeneratorInterface> createGaussianLiDAR();  // 将来

}  // namespace gs_sim
```

**実装ノート**:
- Raycast ライブラリ候補:
  - Embree（CPU/GPU、Intel製）
  - OptiX（GPU、NVIDIA製）
  - カスタム CUDA カーネル
- 初期実装: heightmap を使った 2.5D Raycast
- 将来拡張: static_mesh を使った 3D Raycast

---

### 8. ROS2Bridge

**責務**: ROS2 トピック・サービス・TFを管理する

**ヘッダー**: `gs_sim/ros2_bridge.hpp`

```cpp
namespace gs_sim {

class ROS2BridgeInterface {
public:
  virtual ~ROS2BridgeInterface() = default;

  /**
   * @brief Initialize ROS2 node
   * @param node_name Node name
   * @param world_data World data for configuration
   */
  virtual void initialize(const std::string& node_name, const WorldData& world_data) = 0;

  /**
   * @brief Publish /clock
   * @param sim_time Simulation time
   */
  virtual void publishClock(const rclcpp::Time& sim_time) = 0;

  /**
   * @brief Publish TF (map -> odom -> base_link)
   * @param ego_pose Ego pose
   * @param stamp Timestamp
   */
  virtual void publishTF(const EgoPose& ego_pose, const rclcpp::Time& stamp) = 0;

  /**
   * @brief Publish static TF (base_link -> sensors)
   */
  virtual void publishStaticTF() = 0;

  /**
   * @brief Publish /odom
   * @param ego_state Ego state
   */
  virtual void publishOdom(const EgoState& ego_state) = 0;

  /**
   * @brief Publish /vehicle/state
   * @param ego_state Ego state
   */
  virtual void publishVehicleState(const EgoState& ego_state) = 0;

  /**
   * @brief Publish /sim/status
   * @param status Simulation status
   * @param stamp Timestamp
   */
  virtual void publishSimStatus(const SimulationStatus& status, const rclcpp::Time& stamp) = 0;

  /**
   * @brief Publish camera image
   * @param image Rendered image
   * @param intrinsics Camera intrinsics
   */
  virtual void publishCameraImage(const RenderedImage& image,
                                   const CameraIntrinsics& intrinsics) = 0;

  /**
   * @brief Publish LiDAR point cloud
   * @param pointcloud Generated point cloud
   */
  virtual void publishLiDARPointCloud(const GeneratedPointCloud& pointcloud) = 0;

  /**
   * @brief Get latest control input
   * @return Control input, or invalid ControlInput if none received
   */
  virtual ControlInput getLatestControlInput() const = 0;

  /**
   * @brief Spin once (process callbacks)
   */
  virtual void spinOnce() = 0;
};

std::unique_ptr<ROS2BridgeInterface> createROS2Bridge(rclcpp::Node::SharedPtr node);

}  // namespace gs_sim
```

**実装ノート**:
- rclcpp の Node を継承
- executor は SingleThreadedExecutor を推奨（初期実装）
- QoS プロファイル設定は [runtime_simulator.md](../../docs/interfaces/runtime_simulator.md) 参照

---

## ユーティリティクラス

### SensorTrigger

**責務**: センサレートに応じてトリガーを判定

**ヘッダー**: `gs_sim/sensor_trigger.hpp`

```cpp
namespace gs_sim {

class SensorTrigger {
public:
  SensorTrigger(double rate_hz, double dt)
    : period_(1.0 / rate_hz), dt_(dt), next_trigger_(0.0) {}

  /**
   * @brief Check if sensor should trigger at current time
   * @param current_time Current simulation time [s]
   * @return true if sensor should trigger
   */
  bool shouldTrigger(double current_time) {
    if (current_time >= next_trigger_ - dt_ / 2.0) {
      next_trigger_ += period_;
      return true;
    }
    return false;
  }

  void reset() {
    next_trigger_ = 0.0;
  }

private:
  double period_;
  double dt_;
  double next_trigger_;
};

}  // namespace gs_sim
```

---

## エラーハンドリング

### 例外クラス

**ヘッダー**: `gs_sim/exceptions.hpp`

```cpp
namespace gs_sim {

class SimulatorException : public std::runtime_error {
public:
  explicit SimulatorException(const std::string& message)
    : std::runtime_error(message) {}
};

class WorldLoadException : public SimulatorException {
public:
  explicit WorldLoadException(const std::string& message)
    : SimulatorException("WorldLoader: " + message) {}
};

class RenderException : public SimulatorException {
public:
  explicit RenderException(const std::string& message)
    : SimulatorException("CameraRenderer: " + message) {}
};

class LiDARException : public SimulatorException {
public:
  explicit LiDARException(const std::string& message)
    : SimulatorException("LiDARGenerator: " + message) {}
};

class DynamicsException : public SimulatorException {
public:
  explicit DynamicsException(const std::string& message)
    : SimulatorException("VehicleDynamics: " + message) {}
};

}  // namespace gs_sim
```

---

## 使用例

### シミュレータノードの構成

```cpp
#include "gs_sim/world_loader.hpp"
#include "gs_sim/sim_clock.hpp"
#include "gs_sim/vehicle_dynamics.hpp"
#include "gs_sim/ground_contact.hpp"
#include "gs_sim/ego_state_core.hpp"
#include "gs_sim/camera_renderer.hpp"
#include "gs_sim/lidar_generator.hpp"
#include "gs_sim/ros2_bridge.hpp"
#include "gs_sim/sensor_trigger.hpp"

class SimulatorNode : public rclcpp::Node {
public:
  SimulatorNode(const std::string& world_bundle_path)
    : Node("simulator_node") {

    // 1. Load world_bundle
    world_loader_ = gs_sim::createWorldLoader();
    world_data_ = world_loader_->load(world_bundle_path);

    if (!world_loader_->validate(world_data_)) {
      throw gs_sim::WorldLoadException("Invalid world_bundle");
    }

    // 2. Initialize components
    sim_clock_ = gs_sim::createSimClock();
    sim_clock_->initialize(world_data_.start_time, world_data_.dt, 1.0);

    vehicle_dynamics_ = gs_sim::createKinematicBicycle();
    vehicle_dynamics_->initialize(2.8, 0.52, 30.0);

    ground_contact_ = gs_sim::createGroundContact();
    ground_contact_->loadHeightmap(
      world_data_.bundle_path / world_data_.heightmap_bin_path,
      world_data_.bundle_path / world_data_.heightmap_yaml_path
    );

    ego_state_core_ = gs_sim::createEgoStateCore();
    ego_state_core_->initialize(
      world_data_.bundle_path / world_data_.drivable_geojson_path
    );

    camera_renderer_ = gs_sim::createCameraRenderer();
    camera_renderer_->loadGaussians(
      world_data_.bundle_path / world_data_.gaussian_ply_path,
      world_data_.bundle_path / world_data_.render_config_path
    );

    lidar_generator_ = gs_sim::createRaycastLiDAR();
    lidar_generator_->loadHeightmap(
      world_data_.bundle_path / world_data_.heightmap_bin_path,
      world_data_.bundle_path / world_data_.heightmap_yaml_path
    );

    ros2_bridge_ = gs_sim::createROS2Bridge(shared_from_this());
    ros2_bridge_->initialize("simulator_node", world_data_);

    // 3. Start simulation loop
    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(world_data_.dt * 1000)),
      std::bind(&SimulatorNode::step, this)
    );
  }

private:
  void step() {
    // 1. Advance time
    auto sim_time = sim_clock_->step();
    ros2_bridge_->publishClock(sim_time);

    // 2. Get control input
    auto control_input = ros2_bridge_->getLatestControlInput();

    // 3. Update vehicle dynamics
    auto ego_state = ego_state_core_->getState();
    vehicle_dynamics_->update(world_data_.dt, control_input, ego_state);

    // 4. Update ground contact
    ego_state.pose.z = ground_contact_->getGroundHeight(ego_state.pose.x, ego_state.pose.y);

    // 5. Update ego state
    ego_state_core_->updateState(ego_state);

    // 6. Publish ego state
    ros2_bridge_->publishTF(ego_state.pose, sim_time);
    ros2_bridge_->publishOdom(ego_state);
    ros2_bridge_->publishVehicleState(ego_state);

    // 7. Check simulation status
    auto status = ego_state_core_->checkStatus();
    ros2_bridge_->publishSimStatus(status, sim_time);

    // 8. Render sensors
    if (camera_trigger_.shouldTrigger(sim_time.seconds())) {
      for (const auto& camera_id : camera_renderer_->getCameraIds()) {
        auto image = camera_renderer_->render(camera_id, ego_state.pose, sim_time);
        ros2_bridge_->publishCameraImage(image, camera_intrinsics_[camera_id]);
      }
    }

    if (lidar_trigger_.shouldTrigger(sim_time.seconds())) {
      for (const auto& lidar_id : lidar_generator_->getLiDARIds()) {
        auto pointcloud = lidar_generator_->generate(lidar_id, ego_state.pose, sim_time);
        ros2_bridge_->publishLiDARPointCloud(pointcloud);
      }
    }

    // 9. Sleep to match real-time factor
    sim_clock_->sleep();
  }

  // Components
  std::unique_ptr<gs_sim::WorldLoaderInterface> world_loader_;
  std::unique_ptr<gs_sim::SimClockInterface> sim_clock_;
  std::unique_ptr<gs_sim::VehicleDynamicsInterface> vehicle_dynamics_;
  std::unique_ptr<gs_sim::GroundContactInterface> ground_contact_;
  std::unique_ptr<gs_sim::EgoStateCoreInterface> ego_state_core_;
  std::unique_ptr<gs_sim::CameraRendererInterface> camera_renderer_;
  std::unique_ptr<gs_sim::LiDARGeneratorInterface> lidar_generator_;
  std::unique_ptr<gs_sim::ROS2BridgeInterface> ros2_bridge_;

  gs_sim::WorldData world_data_;
  gs_sim::SensorTrigger camera_trigger_{12.0, 0.01};
  gs_sim::SensorTrigger lidar_trigger_{20.0, 0.01};

  std::map<std::string, gs_sim::CameraIntrinsics> camera_intrinsics_;

  rclcpp::TimerBase::SharedPtr timer_;
};
```

---

## 依存関係

### 必須ライブラリ

```cmake
# CMakeLists.txt

# ROS2
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED)

# YAML
find_package(yaml-cpp REQUIRED)

# JSON
# nlohmann/json (header-only)

# 3DGS rendering (example: gsplat)
# Custom or third-party

# Raycast (example: Embree)
find_package(embree REQUIRED)

# GIS (example: GEOS)
find_package(GEOS REQUIRED)
```

---

## 変更履歴

| バージョン | 日付 | 変更内容 |
|-----------|------|---------|
| 1.0.1 | 2026-02-15 | architecture.md との整合性確保、実装例の明確化 |
| 1.0.0 | 2026-02-15 | 初版作成（interfaces.md から分割） |
