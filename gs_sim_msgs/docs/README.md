# gs_sim_msgs

**バージョン**: 1.0.0

## 概要

`gs_sim_msgs` は GS-ROS2 Simulator で使用する ROS2 メッセージ・サービス定義を提供するパッケージです。

このパッケージには内部コンポーネントは存在せず、純粋なメッセージ定義のみを提供します。

---

## ドキュメント

すべてのメッセージ・サービス定義は以下の外部契約に記載されています：

📄 **[../../docs/interfaces/ros2_messages.md](../../docs/interfaces/ros2_messages.md)**

このドキュメントには以下が定義されています：
- カスタムメッセージ定義（VehicleControlCmd, VehicleState, SimulationStatus 等）
- サービス定義（ResetWorld, SetEgoPose 等）
- 標準メッセージの使用方針
- QoS プロファイル推奨設定

---

## パッケージ構成

```
gs_sim_msgs/
├── msg/                    # メッセージ定義 (.msg)
│   ├── VehicleControlCmd.msg
│   ├── VehicleState.msg
│   ├── SimulationStatus.msg
│   └── WorldInfo.msg
├── srv/                    # サービス定義 (.srv)
│   ├── ResetWorld.srv
│   ├── SetEgoPose.srv
│   ├── Pause.srv
│   ├── Resume.srv
│   └── Step.srv
├── CMakeLists.txt
├── package.xml
└── docs/
    └── README.md           # このファイル
```

---

## 使用方法

### C++ から使用

```cpp
#include <gs_sim_msgs/msg/vehicle_state.hpp>
#include <gs_sim_msgs/srv/reset_world.hpp>

// メッセージの使用例
gs_sim_msgs::msg::VehicleState state;
state.header.stamp = this->now();
state.steering_angle = 0.1;

// サービスの使用例
auto client = create_client<gs_sim_msgs::srv::ResetWorld>("/sim/reset_world");
auto request = std::make_shared<gs_sim_msgs::srv::ResetWorld::Request>();
request->world_path = "worlds/scene_001";
```

### Python から使用

```python
from gs_sim_msgs.msg import VehicleState
from gs_sim_msgs.srv import ResetWorld

# メッセージの使用例
state = VehicleState()
state.header.stamp = self.get_clock().now().to_msg()
state.steering_angle = 0.1

# サービスの使用例
client = self.create_client(ResetWorld, '/sim/reset_world')
request = ResetWorld.Request()
request.world_path = 'worlds/scene_001'
```

---

## 関連パッケージ

- **gs_ros2_simulator**: このメッセージを使用するシミュレータ本体
- **gs_world_builder**: world_bundle を生成（メッセージは使用しない）

---

## 変更履歴

| バージョン | 日付 | 変更内容 |
|-----------|------|---------|
| 1.0.0 | 2026-02-14 | 初版作成 |
