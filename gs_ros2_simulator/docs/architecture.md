# gs_ros2_simulator 内部コンポーネント設計

**バージョン**: 1.0.1
**最終更新**: 2026-02-15

## 概要

このドキュメントは `gs_ros2_simulator` の**内部コンポーネント分割と実装設計**を定義します。

**重要な区別**:
- **[docs/interfaces/runtime_simulator.md](../../docs/interfaces/runtime_simulator.md)** - 外部インターフェース契約（ROS2トピック、サービス、パラメータ）
- **このドキュメント** - 内部実装の設計指針（コンポーネント分割、データフロー）

**関連ドキュメント**:
- **[interface.md](interface.md)** - コンポーネント間のデータ契約
- **[design.md](design.md)** - C++クラス・関数設計

---

## 設計原則

1. **リアルタイム性**: 100Hz (dt=10ms) で安定動作
2. **モジュール分離**: 各コンポーネントは独立してテスト・差し替え可能
3. **疎結合**: ROS2標準メッセージを介して接続
4. **再現性**: 同じ制御入力から同じ結果を生成
5. **段階的実装**: 最小構成から段階的に機能追加

---

## 全体アーキテクチャ

### 単一ノード構成（Phase 1: 初期実装）

```
simulator_node
  ├─ WorldLoader          # world_bundle読み込み
  ├─ SimClock             # /clock配信・時刻管理
  ├─ EgoStateCore         # Ego状態管理
  ├─ VehicleDynamics      # 車両運動モデル
  ├─ GroundContact        # 接地・heightmap
  ├─ CameraRenderer       # カメラレンダリング (3DGS)
  ├─ LiDARGenerator       # LiDAR点群生成
  └─ ROS2Bridge           # ROS2 pub/sub管理
```

すべてを1つのプロセスに統合し、レイテンシを最小化。

### マルチノード構成（Phase 2: 将来拡張）

```
world_state_node          # Ego状態管理
vehicle_dynamics_node     # 車両運動
camera_renderer_node      # カメラレンダリング
lidar_generator_node      # LiDAR生成
sim_clock_node            # /clock配信
```

ノード間はトピックで疎結合、個別に差し替え可能。

---

## コンポーネント詳細

### 1. WorldLoader（world_bundle読み込み）

**責務**: world_bundle を読み込み、各コンポーネントにデータを提供

**入力**:
- world_bundle ディレクトリパス（起動パラメータ）

**処理内容**:
1. `world.yaml` を読み込み、ファイルパスを取得
2. `metadata.json` を読み込み、シーン情報を取得
3. `sensors/calibration.yaml` を読み込み、センサ設定を取得
4. `sensors/tf_static.json` を読み込み、静的TFを取得
5. `sim/timebase.yaml` を読み込み、シミュレーション設定を取得
6. `gaussians/background.splat.ply` のパスを取得（CameraRendererに渡す）
7. `geometry/heightmap.*` のパスを取得（GroundContactに渡す）
8. `geometry/drivable.geojson` を読み込み（EgoStateCoreに渡す）
9. **最終検証**:
   - 必須ファイルの存在確認
   - バージョン互換性チェック
   - calibration と tf_static の一貫性チェック

**出力**:
- `WorldData` 構造体（各コンポーネントで共有）

**実装ノート**:
- 起動時に1回だけ実行
- 検証失敗時はエラーで終了（exit code 2）
- world_bundle の再読み込みは `/sim/reset_world` サービスで実現

---

### 2. SimClock（シミュレーション時刻管理）

**責務**: シミュレーション時刻を管理し、`/clock` を配信

**入力**:
- `sim/timebase.yaml` の `dt`, `start_time`
- 起動パラメータ `real_time_factor`

**処理内容**:
1. シミュレーション時刻 `t` を管理（初期値: `start_time`）
2. 毎ステップ `t += dt`
3. `/clock` トピックに `rosgraph_msgs/Clock` を配信（100Hz）
4. `real_time_factor` に応じてスリープ
   - `real_time_factor = 1.0`: リアルタイム（dt=10ms なら 10ms スリープ）
   - `real_time_factor = 0.0`: 最速（スリープなし）
5. 一時停止・再開・ステップ実行モードのサポート

**出力**:
- `/clock` (rosgraph_msgs/Clock, 100Hz)
- 現在のシミュレーション時刻 `t` （各コンポーネントで共有）

**実装ノート**:
- use_sim_time=true 環境で必須
- ステップ実行モードでは `/sim/step` サービスで1ステップずつ進める
- 処理時間が `dt` を超えた場合は警告ログ（WARN）

---

### 3. EgoStateCore（Ego状態管理）

**責務**: Ego の状態（pose/twist）を一元管理し、衝突・offroad 判定を行う

**入力**:
- VehicleDynamics で計算された新しい状態
- `geometry/drivable.geojson` （走行可能領域）

**処理内容**:
1. **Ego の状態を保持**（Source of Truth）
   - 現在の pose/twist を常に保持
   - VehicleDynamics に現在の状態を提供
   - VehicleDynamics から更新された状態を受け取る
2. **衝突判定**（Phase 2 で実装）
3. **Offroad 判定**（drivable 領域外チェック）
4. **ROS2 メッセージ配信**
   - `/odom` (nav_msgs/Odometry, 100Hz)
   - `/vehicle/state` (gs_sim_msgs/VehicleState, 100Hz)
   - `/sim/status` (gs_sim_msgs/SimulationStatus, 10Hz)

**出力**:
- `/odom` (nav_msgs/Odometry, 100Hz)
- `/vehicle/state` (gs_sim_msgs/VehicleState, 100Hz)
- `/sim/status` (gs_sim_msgs/SimulationStatus, 10Hz)
- 現在の Ego state（各コンポーネントで参照）

**実装ノート**:
- **状態の一元管理**: EgoStateCore が唯一の状態保持者（Single Source of Truth）
- drivable 領域の判定には GEOS (C++) または Shapely (Python) を使用
- offroad 検出時は `/sim/status` でフラグを立てる（WARN ログ）

---

### 4. VehicleDynamics（車両運動モデル）

**責務**: 制御入力から Ego の運動を計算する（状態は保持しない）

**入力**:
- 制御入力（ROS2Bridge から取得）
- 現在の Ego 状態（EgoStateCore から取得）
- 起動パラメータ（wheelbase, max_steering_angle, max_speed 等）

**処理内容**:
1. 制御入力の検証
   - タイムアウトチェック（1.0秒、パラメータで変更可能）
   - 制限値のクランプ（max_steering_angle, max_speed 等）
2. ダイナミクスモデルで運動を計算
   - `kinematic_bicycle`: キネマティック・バイシクルモデル（初期実装）
   - `dynamic_bicycle`: ダイナミック・バイシクルモデル（将来拡張）
3. 新しい pose/twist を計算（dt=10ms ごと）
4. 計算結果を返す（状態更新は EgoStateCore が実施）

**出力**:
- 更新された pose/twist（EgoStateCore に渡す）

**実装ノート**:
- **ステートレス**: VehicleDynamics は状態を保持せず、純粋な計算のみ実施
- ダイナミクスモデルはインターフェースで抽象化（差し替え可能）
- タイムアウト時は緊急停止（減速度 3.0 m/s²）
- 積分法は Runge-Kutta 4次（RK4）を推奨
- 詳細は **[design.md](design.md#3-vehicledynamics)** 参照

---

### 5. GroundContact（接地・heightmap）

**責務**: heightmap から接地点の高さを取得（Pull パターン）

**入力**:
- `geometry/heightmap.bin`, `geometry/heightmap.yaml`（起動時読み込み）
- クエリ: 現在の Ego position (x, y)

**処理内容**:
1. heightmap を読み込み（起動時1回）
2. リクエストされた (x, y) から対応するグリッドセルを特定
3. 双線形補間で高さ z を計算
4. 無効領域（NaN）の場合はエラーログ

**出力**:
- 接地点の高さ z [m]（またはNaN）

**データフロー**:
- VehicleDynamics や EgoStateCore が `getGroundHeight(x, y)` を**呼び出す**
- GroundContact は受動的に高さを返す（Push ではなく Pull パターン）

**実装ノート**:
- heightmap の読み込みは Eigen（C++）または NumPy（Python）を推奨
- 双線形補間で滑らかな高さ変化を実現
- 将来的には `static_mesh.glb` を使った高精度接地も可能
- 詳細は **[design.md](design.md#4-groundcontact)** 参照

---

### 6. CameraRenderer（カメラレンダリング）

**責務**: 3DGS を使ってカメラ画像をレンダリング

**入力**:
- `gaussians/background.splat.ply` （Gaussianデータ）
- `gaussians/render_config.json` （レンダリング設定）
- `sensors/calibration.yaml` （カメラ intrinsics/extrinsics）
- 現在の Ego pose

**処理内容**:
1. Gaussian データを GPU にロード（起動時1回）
2. 各カメラの intrinsics/extrinsics を読み込み
3. センサトリガーのタイミングで画像をレンダリング
   - カメラレート（例: 12Hz）に従って実行
   - Ego pose と extrinsics から world-to-camera 変換を計算
   - 3DGS レンダリング（GPU）
4. RGB8 形式で `/camera/{camera_id}/image_raw` を配信
5. `/camera/{camera_id}/camera_info` を配信

**出力**:
- `/camera/{camera_id}/image_raw` (sensor_msgs/Image, 12Hz*)
- `/camera/{camera_id}/camera_info` (sensor_msgs/CameraInfo, 12Hz*)

*レートは calibration.yaml で設定

**実装ノート**:
- 3DGS レンダリングライブラリの候補:
  - gsplat（Python/CUDA）
  - nerfstudio-gsplat（C++/CUDA）
  - Vulkan backend（将来）
- レンダリング時間が目標値（5ms）を超えた場合は警告ログ
- 初期実装では背景のみ、将来的には動的物体も統合

---

### 7. LiDARGenerator（LiDAR点群生成）

**責務**: heightmap または static_mesh から LiDAR 点群を生成

**入力**:
- `geometry/heightmap.bin`, `geometry/heightmap.yaml` （Raycast方式1）
- `geometry/static_mesh.glb` （Raycast方式2、任意）
- `sensors/calibration.yaml` （LiDAR spec/extrinsics）
- 現在の Ego pose

**処理内容**:
1. LiDAR spec を読み込み（channels, horizontal_resolution, vertical_fov 等）
2. センサトリガーのタイミングで点群を生成
   - LiDARレート（例: 20Hz）に従って実行
   - Ego pose と extrinsics から world-to-lidar 変換を計算
   - Raycast 実行（GPUを推奨）
     - **方式1**: heightmap を使った 2.5D Raycast（初期実装）
     - **方式2**: static_mesh を使った 3D Raycast（将来拡張）
     - **方式3**: Gaussian を使った Raycast（将来研究課題）
3. PointCloud2 形式で `/lidar/{lidar_id}/points` を配信

**出力**:
- `/lidar/{lidar_id}/points` (sensor_msgs/PointCloud2, 20Hz*)

*レートは calibration.yaml で設定

**実装ノート**:
- Raycast ライブラリの候補:
  - Embree（CPU/GPU、Intel製）
  - OptiX（GPU、NVIDIA製）
  - カスタム CUDA カーネル
- heightmap Raycast は単純だが精度が低い（2.5Dのため）
- static_mesh Raycast は精度が高いが重い
- ノイズ追加は任意（パラメータで有効化）
- 詳細は **[design.md](design.md#7-lidargenerator)** 参照

---

### 8. ROS2Bridge（ROS2 pub/sub管理）

**責務**: すべての ROS2 トピック・サービス・TFを管理

**入力**:
- 各コンポーネントからの配信データ
- `/vehicle/control_cmd` （購読）
- サービスリクエスト

**処理内容**:
1. **Publisher 初期化**:
   - `/clock`, `/tf`, `/tf_static`, `/odom`, `/vehicle/state`, `/sim/status`
   - `/camera/{camera_id}/image_raw`, `/camera/{camera_id}/camera_info`
   - `/lidar/{lidar_id}/points`
2. **Subscriber 初期化**:
   - `/vehicle/control_cmd`
3. **Service Server 初期化**:
   - `/sim/reset_world`, `/sim/set_ego_pose`, `/sim/pause`, `/sim/resume`, `/sim/step`
4. **TF Broadcaster 初期化**:
   - 動的TF: `map` → `odom` → `base_link`
   - 静的TF: `base_link` → センサフレーム
5. **QoS プロファイル設定**:
   - センサ: Reliable, Volatile, Keep Last 5
   - TF: Best Effort, Transient Local, Keep Last 100
   - 詳細は [runtime_simulator.md](../../docs/interfaces/runtime_simulator.md) 参照
6. **データ配信**:
   - 各コンポーネントからデータを受け取り、適切なトピックに配信

**出力**:
- すべての ROS2 トピック・サービス・TF

**実装ノート**:
- rclcpp の Node を継承
- executor は SingleThreadedExecutor を推奨（初期実装）
- MultiThreadedExecutor は Phase 2 で検討

---

## データフロー

### 周期的更新（100Hz）

**高レベルフロー**:
```
1. 時刻更新: SimClock が t += dt, /clock 配信

2. 制御入力取得: ROS2Bridge が /vehicle/control_cmd を読み込み

3. 車両運動計算:
   - EgoStateCore から現在の状態を取得
   - VehicleDynamics で新しい状態を計算
   - GroundContact で接地点高さを取得（z座標更新）
   - EgoStateCore に新しい状態を設定

4. 状態配信:
   - EgoStateCore が /tf, /odom, /vehicle/state を配信
   - 衝突・offroad 判定を実施
   - /sim/status を配信（10Hz）

5. センサレンダリング（トリガー時のみ）:
   - Camera: 12Hz でトリガー時に画像レンダリング
   - LiDAR: 20Hz でトリガー時に点群生成

6. 時刻同期: SimClock が real_time_factor に応じてスリープ
```

**詳細な実装例**: **[design.md](design.md#使用例)** の SimulatorNode::step() を参照

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

**実装詳細**: **[interface.md](interface.md#センサトリガー仕様)** および **[design.md](design.md#sensortrigger)** を参照

---

## コンポーネント間の依存関係

```mermaid
graph TD
    A[WorldLoader] --> B[SimClock]
    A --> C[EgoStateCore]
    A --> D[VehicleDynamics]
    A --> E[GroundContact]
    A --> F[CameraRenderer]
    A --> G[LiDARGenerator]
    B --> H[ROS2Bridge]
    C --> H
    D --> C
    E --> C
    E --> D
    F --> H
    G --> H
    H --> I[/vehicle/control_cmd]
    I --> D
```

---

## 実装優先順位

### Phase 1: 最小構成（必須）

実装順序:
1. ✅ WorldLoader（world_bundle 読み込み）
2. ✅ SimClock（/clock 配信）
3. ✅ ROS2Bridge（pub/sub 基盤）
4. ✅ EgoStateCore（pose/twist 管理）
5. ✅ VehicleDynamics（kinematic_bicycle モデル）
6. ✅ GroundContact（heightmap 接地）
7. ✅ CameraRenderer（3DGS レンダリング）
8. ✅ LiDARGenerator（heightmap Raycast）

**目標**: Closed-loop シミュレータが動作し、カメラ・LiDAR を配信

---

### Phase 2: 精度向上（任意）

9. DynamicBicycle モデル（タイヤ・スリップ）
10. static_mesh を使った高精度 LiDAR Raycast
11. 衝突判定（static_mesh 利用）
12. マルチノード構成への分割

---

### Phase 3: 将来拡張

13. 動的物体（アクター）のサポート
14. Gaussian LiDAR（研究課題）
15. HDMap 統合
16. シナリオ実行エンジン

---

## テスト戦略

### 単体テスト

各コンポーネントを独立してテスト:

```bash
colcon test --packages-select gs_ros2_simulator
```

**テスト内容**:
- WorldLoader: world_bundle 読み込み、検証
- VehicleDynamics: 各モデルの運動計算
- GroundContact: heightmap 読み込み、双線形補間
- EgoStateCore: offroad 判定
- SimClock: 時刻管理、real_time_factor

---

### 統合テスト

minimal world_bundle でシミュレータ全体をテスト:

```bash
ros2 launch gs_ros2_simulator bringup.launch.xml \
    world:=tests/data/minimal_world
```

**検証項目**:
- [ ] /clock が配信されている
- [ ] /tf, /odom が配信されている
- [ ] Camera image が配信されている
- [ ] LiDAR points が配信されている
- [ ] 制御入力に応じて ego が動く
- [ ] Offroad 検出が動作する

---

### パフォーマンステスト

リアルタイム性の検証:

```bash
ros2 launch gs_ros2_simulator bringup.launch.xml \
    world:=tests/data/benchmark_world \
    real_time_factor:=1.0
```

**目標**:
- 各ステップが 10ms 以内に完了（100Hz 達成）
- Camera rendering: < 5ms
- LiDAR generation: < 2ms

---

## エラーハンドリング

### 各コンポーネントの責務

- **入力検証**: world_bundle の妥当性チェック
- **エラー報告**: 明確なエラーメッセージ（コンポーネント名、原因）
- **Graceful degradation**: 一部のセンサが失敗しても継続可能

### エラー種別

| 種別 | 終了コード | 対処 |
|------|----------|------|
| World bundle not found | 1 | パス確認 |
| World bundle invalid | 2 | validate で検証 |
| GPU not available | 4 | CUDA/Vulkan確認 |
| Control timeout | - | 緊急停止（WARN ログ） |
| Rendering failure | - | 前フレーム再利用（ERROR ログ） |

---

## パフォーマンス最適化

### ボトルネック

1. **CameraRenderer**: GPU レンダリング（目標: 5ms）
2. **LiDARGenerator**: Raycast（目標: 2ms）
3. **GroundContact**: heightmap 読み込み（双線形補間は軽い）

### 最適化戦略

- GPU 利用（3DGS レンダリング、Raycast）
- マルチスレッド（複数カメラの並列レンダリング）
- メモリ効率化（Gaussian データの GPU キャッシュ）

---

## 変更履歴

| バージョン | 日付 | 変更内容 |
|-----------|------|---------|
| 1.0.1 | 2026-02-15 | 責務明確化、重複削除、design.mdへの参照追加 |
| 1.0.0 | 2026-02-14 | 初版作成 |
