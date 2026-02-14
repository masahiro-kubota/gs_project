# ROS2 メッセージ定義契約

**バージョン**: 1.0.2
**最終更新**: 2026-02-15

## 概要

`gs_sim_msgs` は GS-ROS2 Simulator で使用する ROS2 メッセージ、サービス、アクションの定義を提供します。
標準 ROS2 メッセージ（`sensor_msgs`, `nav_msgs`, `geometry_msgs` など）で対応できるものは極力使用し、
シミュレータ固有の情報のみをカスタムメッセージとして定義します。

**準拠規約**: このパッケージは [`conventions.md`](../conventions.md) で定義された
共通規約（座標系、単位系、命名規則等）に従います。

---

## 設計原則

1. **標準メッセージ優先**: 可能な限り ROS2 標準メッセージを使用
2. **最小限の定義**: 本当に必要なものだけを定義
3. **バージョニング**: メッセージ構造変更時は後方互換性を考慮
4. **明確な命名**: 用途が明確にわかる名前を使用

---

## メッセージ定義

### 1. 制御入力（Control Input）

#### デフォルト推奨: 標準メッセージを使用
```
# 制御入力には標準メッセージを使用することをデフォルトとして推奨
ackermann_msgs/AckermannDriveStamped  # ステア + 速度/加速度制御
geometry_msgs/TwistStamped             # 簡易的な速度制御
```

#### カスタムメッセージ（Optional: 必要な場合のみ）
```
gs_sim_msgs/VehicleControlCmd
---
std_msgs/Header header

# 操舵
float32 steering_angle        # [rad] 正=左（必須）
float32 steering_angle_rate   # [rad/s] Optional（省略可能）

# 駆動/制動
float32 throttle              # [0.0-1.0] アクセル開度（必須）
float32 brake                 # [0.0-1.0] ブレーキ圧（必須）
float32 target_velocity       # [m/s] 目標速度（速度制御モード、必須）
float32 target_acceleration   # [m/s^2] 目標加速度（加速度制御モード、必須）

# ギア・モード
uint8 gear                    # 0=P, 1=R, 2=N, 3=D
uint8 control_mode            # 0=throttle/brake, 1=velocity, 2=acceleration

uint8 GEAR_PARK = 0
uint8 GEAR_REVERSE = 1
uint8 GEAR_NEUTRAL = 2
uint8 GEAR_DRIVE = 3

uint8 MODE_DIRECT = 0
uint8 MODE_VELOCITY = 1
uint8 MODE_ACCELERATION = 2
```

---

### 2. 車両状態（Vehicle State）

```
gs_sim_msgs/VehicleState
---
std_msgs/Header header

# 位置姿勢（map frame）
geometry_msgs/Pose pose
geometry_msgs/Twist twist       # linear/angular velocity
geometry_msgs/Accel accel       # linear/angular acceleration

# 車両固有状態
float32 steering_angle          # [rad] 実ステア角
float32 steering_angle_rate     # [rad/s]
float32 wheel_speed_fl          # [m/s] 前左
float32 wheel_speed_fr          # [m/s] 前右
float32 wheel_speed_rl          # [m/s] 後左
float32 wheel_speed_rr          # [m/s] 後右

uint8 gear                      # 現在のギア
```

---

### 3. シミュレーション状態（Simulation Status）

```
gs_sim_msgs/SimulationStatus
---
std_msgs/Header header

# シミュレーション時刻
builtin_interfaces/Time sim_time    # シミュレーション時刻
float64 sim_dt                      # [s] シミュレーションタイムステップ
uint64 step_count                   # ステップカウント

# リアルタイム性能
float64 real_time_factor            # 実時間比率（1.0=リアルタイム）
float64 step_duration_ms            # [ms] 前ステップの処理時間

# 状態フラグ
bool is_collision                   # 衝突発生
bool is_offroad                     # 走行可能領域外
bool is_paused                      # 一時停止中

# エラー・警告
string[] warnings                   # 警告メッセージ
string[] errors                     # エラーメッセージ
```

---

### 4. ワールド情報（World Info）

```
gs_sim_msgs/WorldInfo
---
std_msgs/Header header

# ワールドメタデータ
string world_id                     # ワールド識別子（例: "nuscenes_scene_0103"）
string world_bundle_path            # world_bundle のパス
string source_mcap                  # 生成元 mcap ファイル名
string world_version                # world_bundle フォーマットバージョン

# 座標系情報
string map_frame                    # 通常 "map"
string odom_frame                   # 通常 "odom"
string base_link_frame              # 通常 "base_link"

# 空間範囲
geometry_msgs/Point world_origin    # ワールド原点（LLA等）
float64 world_extent_x              # [m] X方向範囲
float64 world_extent_y              # [m] Y方向範囲
float64 world_extent_z              # [m] Z方向範囲

# 利用可能なセンサ
string[] available_cameras          # カメラリスト（例: ["front", "left", "right"]）
string[] available_lidars           # LiDARリスト（例: ["top"]）
```

---

## サービス定義

### 1. ワールドリセット

```
gs_sim_msgs/srv/ResetWorld
---
# Request
geometry_msgs/Pose initial_pose     # 初期位置（Optional: 省略時はデフォルト値を使用）
bool reset_time                     # シミュレーション時刻もリセットするか（必須）

---
# Response
bool success
string message
```

---

### 2. Ego Pose 設定（デバッグ用）

```
gs_sim_msgs/srv/SetEgoPose
---
# Request
geometry_msgs/Pose pose  # 必須
geometry_msgs/Twist twist           # Optional（省略可能）: 初期速度

---
# Response
bool success
string message
```

---

### 3. ワールド読み込み

```
gs_sim_msgs/srv/LoadWorld
---
# Request
string world_bundle_path            # world_bundle のパス

---
# Response
bool success
string message
gs_sim_msgs/WorldInfo world_info    # 読み込んだワールド情報
```

---

### 4. シミュレーション制御

```
gs_sim_msgs/srv/SimulationControl
---
# Request
uint8 command  # 必須
bool step_once                      # Optional: step モードの場合、1ステップだけ進める

uint8 CMD_PAUSE = 0
uint8 CMD_RESUME = 1
uint8 CMD_STEP = 2
uint8 CMD_RESET = 3

---
# Response
bool success
string message
```

---

## アクション定義

### 1. シナリオ実行（将来拡張用）

```
gs_sim_msgs/action/ExecuteScenario
---
# Goal
string scenario_file                # シナリオファイルパス
float64 timeout_sec                 # タイムアウト [s]

---
# Result
bool success
string message
float64 completion_time             # 実行時間 [s]

---
# Feedback
float64 progress                    # 進捗 [0.0-1.0]
string current_phase                # 現在のフェーズ
```

---

## トピック命名規約

### シミュレータが配信するトピック（出力）

| トピック名 | メッセージ型 | 周波数 | 説明 |
|-----------|------------|--------|------|
| `/clock` | `rosgraph_msgs/Clock` | sim依存 | シミュレーション時刻 |
| `/tf` | `tf2_msgs/TFMessage` | sim依存 | 動的TF |
| `/tf_static` | `tf2_msgs/TFMessage` | latch | 静的TF |
| `/odom` | `nav_msgs/Odometry` | 100Hz | Ego オドメトリ |
| `/vehicle/state` | `gs_sim_msgs/VehicleState` | 100Hz | 詳細車両状態 |
| `/sim/status` | `gs_sim_msgs/SimulationStatus` | 10Hz | シミュレーション状態 |
| `/sim/world_info` | `gs_sim_msgs/WorldInfo` | latch | ワールド情報 |
| `/camera/{camera_id}/image_raw` | `sensor_msgs/Image` | 12Hz* | カメラ画像 |
| `/camera/{camera_id}/camera_info` | `sensor_msgs/CameraInfo` | 12Hz* | カメラ情報 |
| `/lidar/{lidar_id}/points` | `sensor_msgs/PointCloud2` | 20Hz* | LiDAR点群 |

*センサレートは `timebase.yaml` で設定可能

### シミュレータが購読するトピック（入力）

| トピック名 | メッセージ型 | 説明 |
|-----------|------------|------|
| `/vehicle/control_cmd` | `ackermann_msgs/AckermannDriveStamped` または `gs_sim_msgs/VehicleControlCmd` | 車両制御指令 |

---

## フレーム ID 規約

| フレームID | 説明 |
|-----------|------|
| `map` | 固定ワールド座標系 |
| `odom` | オドメトリ座標系（通常はmapと一致） |
| `base_link` | 車両中心 |
| `camera_{id}` | カメラフレーム（例: `camera_front`） |
| `lidar_{id}` | LiDARフレーム（例: `lidar_top`） |

TF ツリー:
```
map
 └─ odom
     └─ base_link
         ├─ camera_front
         ├─ camera_left
         ├─ camera_right
         └─ lidar_top
```

---

## バージョニング戦略

- メッセージ定義の変更は慎重に行う
- 非互換な変更が必要な場合は、メッセージ名にバージョンを含める（例: `VehicleStateV2`）
- 新フィールド追加は末尾に追加し、デフォルト値を持たせる
- 削除が必要な場合は deprecated とマークし、1バージョン猶予を設ける

---

## 依存パッケージ

```xml
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
<depend>tf2_msgs</depend>
<depend>ackermann_msgs</depend>
<depend>builtin_interfaces</depend>
<depend>rosgraph_msgs</depend>
```

---

## 実装ノート

### 標準メッセージとの使い分け

- **制御入力**: できるだけ `ackermann_msgs/AckermannDriveStamped` を使用
- **オドメトリ**: `nav_msgs/Odometry` で統一
- **カスタムメッセージ**: 標準で表現できない「車両固有状態」や「シミュ固有情報」のみ

### 拡張性

- 将来的な拡張（動的アクター、交通信号等）を考慮し、名前空間を適切に分離
- シナリオ実行など、現時点で不要な定義は最小限に留める

---

## 変更履歴

| バージョン | 日付 | 変更内容 |
|-----------|------|---------|
| 1.0.2 | 2026-02-15 | 曖昧な表現を明確化（オプション→Optional、必須の明記） |
| 1.0.1 | 2026-02-15 | （スキップ） |
| 1.0.0 | 2026-02-14 | 初版作成 |
