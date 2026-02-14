# テストシナリオ

**バージョン**: 1.0.1
**最終更新**: 2026-02-15

## 概要

このドキュメントは、GS-ROS2 Simulator の各種テストシナリオを定義します。

**テストレベル**:
1. **単体テスト（Unit Test）**: 個別コンポーネントのテスト
2. **統合テスト（Integration Test）**: コンポーネント間のインターフェーステスト
3. **性能テスト（Performance Test）**: リアルタイム性能・リソース使用量のテスト

---

## 0. Smoke Test（基本動作確認）

**目的**: 30秒以内で基本動作を確認（CI/CDのfast feedbackループ用）

### TEST-SMOKE-001: world_bundle最小限起動テスト

**優先度**: P0（Critical）

**目的**: minimal_test が起動することを確認

**入力**: worlds/minimal_test/

**手順**: ros2 launch で30秒起動

**期待される出力**: エラーなく起動、/clock配信

**実行時間**: < 30秒

---

## 1. 単体テスト（Unit Test）

### 1-1. gs_world_builder コンポーネント

#### TEST-WB-001: MCAP読み込み（正常系）

**優先度**: P1（High）

**目的**: 有効なMCAPファイルを正しく読み込めることを確認

**入力**:
- `test_data/minimal_test.mcap`

**手順**:
```bash
python3 -m gs_world_builder.mcap_ingest \
    --input test_data/minimal_test.mcap \
    --output /tmp/test_extract
```

**期待される出力**:
- 終了コード: 0
- `/tmp/test_extract/poses.csv` が存在
- `/tmp/test_extract/images/` にフレームが存在
- `/tmp/test_extract/lidar/` にフレームが存在
- `timeline.json` が生成

**検証項目**:
- [ ] `poses.csv` の行数 = 6000（100Hz × 60秒）
- [ ] `images/front/` のファイル数 = 720（12Hz × 60秒）
- [ ] `lidar/` のファイル数 = 1200（20Hz × 60秒）
- [ ] すべてのタイムスタンプが昇順

---

#### TEST-WB-002: MCAP読み込み（破損ファイル）

**優先度**: P1（High）

**目的**: 破損したMCAPファイルのエラーハンドリングを確認

**入力**:
- `test_data/corrupted.mcap`

**手順**:
```bash
python3 -m gs_world_builder.mcap_ingest \
    --input test_data/corrupted.mcap \
    --output /tmp/test_extract
```

**期待される出力**:
- 終了コード: 2（MCAP読み込みエラー）
- stderr: "ERROR: Failed to read MCAP file" を含む

---

#### TEST-WB-003: Heightmap生成

**優先度**: P1（High）

**目的**: 点群からheightmapを正しく生成できることを確認

**入力**:
- テスト用点群: 平坦な地面（z=0）+ 100,000点

**手順**:
```python
from gs_world_builder.geometry_builder import GroundSurfaceBuilder

builder = GroundSurfaceBuilder(resolution=0.1)
heightmap = builder.build(point_cloud)
```

**期待される出力**:
- heightmap.shape = (128, 128)
- heightmap の平均値 ≈ 0.0 ± 0.05
- heightmap の標準偏差 < 0.1
- NaN が含まれない（有効領域）

---

#### TEST-WB-004: Drivable領域生成

**優先度**: P1（High）

**目的**: Ego軌跡からdrivable領域を生成できることを確認

**入力**:
- 軌跡: 直線（x: 0→100, y: 0, z: 0）

**手順**:
```python
from gs_world_builder.geometry_builder import DrivableAreaBuilder

builder = DrivableAreaBuilder(buffer_width=5.0)
drivable = builder.build(poses)
```

**期待される出力**:
- GeoJSON Polygon 1つ
- 面積 ≈ 1000 m²（100m × 10m）
- すべての軌跡点がポリゴン内に含まれる

---

#### TEST-WB-005: Validation エラー（エラーコード5）

**優先度**: P1（High）

**目的**: calibration.yaml と tf_static.json の不一致検出

**入力**: worlds/invalid_calibration_mismatch/

**手順**: gs-world-builder validate

**期待される出力**:
- 終了コード: 5（Validation エラー）
- エラーメッセージ: "calibration mismatch detected"

---

#### TEST-WB-006: リソース不足エラー（エラーコード6）

**優先度**: P2（Medium）

**目的**: メモリ不足時のエラーハンドリング

**入力**: 巨大なMCAP（10GB超）、メモリ制限: 1GB

**手順**:
```bash
ulimit -v 1048576  # 1GB制限
gs-world-builder build --input huge.mcap
```

**期待される出力**:
- 終了コード: 6（リソース不足エラー）
- エラーメッセージ: "Insufficient memory"

---

### 1-2. gs_ros2_simulator コンポーネント

#### TEST-SIM-001: world_bundle読み込み（正常系）

**優先度**: P1（High）

**目的**: 有効なworld_bundleを読み込めることを確認

**入力**:
- `worlds/minimal_test/`

**手順**:
```python
from gs_ros2_simulator.world_loader import WorldLoader

loader = WorldLoader()
world = loader.load("worlds/minimal_test")
```

**期待される出力**:
- 例外が発生しない
- `world.scene_id == "minimal_test"`
- `world.gaussians` が読み込まれている
- `world.heightmap.shape == (128, 128)`
- `world.drivable_polygons` が1つ以上

---

#### TEST-SIM-002: world_bundle読み込み（不正ファイル）

**優先度**: P1（High）

**目的**: 不正なworld_bundleのエラーハンドリングを確認

**入力**:
- `worlds/invalid_missing_heightmap/`

**手順**:
```python
loader = WorldLoader()
try:
    world = loader.load("worlds/invalid_missing_heightmap")
except Exception as e:
    print(f"Error: {e}")
```

**期待される出力**:
- 例外が発生: `FileNotFoundError` または `ValidationError`
- エラーメッセージ: "heightmap.bin not found" を含む

---

#### TEST-SIM-003: Vehicle Dynamics（直進）

**優先度**: P1（High）

**目的**: 直進時の車両運動を確認

**入力**:
- 制御入力: `steering_angle = 0.0`, `speed = 5.0 m/s`
- 初期状態: `(x=0, y=0, yaw=0)`, `v=0`
- シミュレーション時間: 10秒

**手順**:
```python
from gs_ros2_simulator.vehicle_dynamics import KinematicBicycle

vehicle = KinematicBicycle(wheelbase=2.5)
vehicle.reset(x=0, y=0, yaw=0, v=0)

for _ in range(1000):  # 10秒 @ 100Hz
    vehicle.update(steering=0.0, speed=5.0, dt=0.01)
```

**期待される出力**:
- 最終位置: `x ≈ 50.0 ± 0.1`, `y ≈ 0.0 ± 0.01`
- 最終速度: `v ≈ 5.0 ± 0.01`
- 最終ヨー角: `yaw ≈ 0.0 ± 0.001`

---

#### TEST-SIM-004: Vehicle Dynamics（左旋回）

**優先度**: P1（High）

**目的**: 旋回時の車両運動を確認

**入力**:
- 制御入力: `steering_angle = 0.1 rad`, `speed = 5.0 m/s`
- 初期状態: `(x=0, y=0, yaw=0)`, `v=0`
- シミュレーション時間: 5秒

**手順**:
```python
vehicle = KinematicBicycle(wheelbase=2.5)
vehicle.reset(x=0, y=0, yaw=0, v=0)

for _ in range(500):  # 5秒 @ 100Hz
    vehicle.update(steering=0.1, speed=5.0, dt=0.01)
```

**期待される出力**:
- 最終ヨー角: `yaw > 0.0`（左旋回）
- 最終y座標: `y > 0.0`（左方向に移動）
- 旋回半径: `R ≈ wheelbase / tan(steering_angle) ≈ 25.0 m`

---

#### TEST-SIM-005: Ground Contact（heightmap補間）

**優先度**: P2（Medium）

**目的**: heightmapからの接地高さ補間を確認

**入力**:
- heightmap: 128x128、resolution 0.1 m
- テスト点: `(x=5.0, y=5.0)`

**手順**:
```python
from gs_ros2_simulator.ground_contact import HeightmapInterpolator

interpolator = HeightmapInterpolator(heightmap, resolution=0.1)
z = interpolator.get_height(x=5.0, y=5.0)
```

**期待される出力**:
- `z` が有効な値（NaNでない）
- 周囲のheightmap値と整合（双線形補間）

---

#### TEST-SIM-006: Drivable検出（正常領域）

**優先度**: P2（Medium）

**目的**: drivable領域内の判定を確認

**入力**:
- drivable: 矩形 [0, 100] × [-5, 5]
- テスト点: `(x=50.0, y=0.0)`

**手順**:
```python
from gs_ros2_simulator.ground_contact import DrivableChecker

checker = DrivableChecker(drivable_polygons)
is_offroad = checker.is_offroad(x=50.0, y=0.0)
```

**期待される出力**:
- `is_offroad == False`

---

#### TEST-SIM-007: Drivable検出（offroad領域）

**優先度**: P2（Medium）

**目的**: drivable領域外の判定を確認

**入力**:
- drivable: 矩形 [0, 100] × [-5, 5]
- テスト点: `(x=50.0, y=10.0)`

**手順**:
```python
is_offroad = checker.is_offroad(x=50.0, y=10.0)
```

**期待される出力**:
- `is_offroad == True`

---

#### TEST-SIM-008: Sensor Trigger（カメラ12Hz）

**優先度**: P1（High）

**目的**: カメラトリガータイミングの精度を確認

**入力**:
- sim_dt: 0.01秒（100Hz）
- camera_rate: 12Hz

**手順**:
```python
from gs_ros2_simulator.sim_clock import SensorTrigger

trigger = SensorTrigger(rate_hz=12.0, sim_dt=0.01)

timestamps = []
for i in range(1000):  # 10秒
    t = i * 0.01
    if trigger.should_trigger(t):
        timestamps.append(t)
```

**期待される出力**:
- トリガー回数: 120回（12Hz × 10秒）
- 平均間隔: `1/12 ≈ 0.0833秒 ± 0.005秒`
- 誤差: すべてのトリガーで ±5ms 以内

---

#### TEST-SIM-009: 制御入力タイムアウト

**優先度**: P1（High）

**目的**: 制御入力タイムアウト時の緊急停止を確認

**入力**:
- 初期速度: `v = 10.0 m/s`
- 制御入力: なし（タイムアウト）
- タイムアウト時間: 1.0秒

**手順**:
```python
from gs_ros2_simulator.vehicle_dynamics import VehicleController

controller = VehicleController(timeout_sec=1.0)
controller.set_initial_speed(10.0)

# 1.5秒経過（制御入力なし）
for _ in range(150):
    controller.update(dt=0.01, has_input=False)
```

**期待される出力**:
- タイムアウト後、減速開始
- 減速度: `3.0 m/s²`
- 最終速度: `v = 10.0 - 3.0 * 0.5 = 8.5 m/s`（1.5秒後）

---

#### TEST-SIM-010: Degraded Mode（heightmap読み込み失敗）

**優先度**: P2（Medium）

**目的**: heightmap不正時のDegraded Mode動作確認

**入力**: heightmap.binが破損したworld_bundle

**手順**: シミュレータ起動

**期待される出力**:
- 警告ログ: "Failed to load heightmap, using flat ground"
- 動作継続（z=0の平坦地面として動作）
- /simulation_status で degraded=true

---

#### TEST-SIM-011: Degraded Mode（static_mesh欠損）

**優先度**: P3（Low）

**目的**: static_mesh.glb 欠損時の動作確認

**入力**: static_mesh.glbのないworld_bundle

**手順**: シミュレータ起動

**期待される出力**:
- 警告ログ: "static_mesh.glb not found, using heightmap only"
- 動作継続（heightmapのみ使用）

---

## 2. 統合テスト（Integration Test）

### 2-1. End-to-End テスト

#### TEST-E2E-001: MCAP → world_bundle 生成

**優先度**: P0（Critical）

**目的**: MCAPから完全なworld_bundleを生成できることを確認

**入力**:
- `test_data/minimal_test.mcap`
- `config/builder_config.yaml`

**手順**:
```bash
gs-world-builder build \
    --input test_data/minimal_test.mcap \
    --config config/builder_config.yaml \
    --output /tmp/test_world
```

**期待される出力**:
- 終了コード: 0
- `/tmp/test_world/world.yaml` が存在
- すべての必須ファイルが存在
- `build_report.json` の `status == "success"`

**検証項目**:
- [ ] `world.yaml` がスキーマに準拠
- [ ] `metadata.json` に必須フィールドがすべて存在
- [ ] `background.splat.ply` のGaussian数 > 0
- [ ] `heightmap.bin` のサイズが正しい
- [ ] `drivable.geojson` に最低1つのPolygon
- [ ] validation実行: `gs-world-builder validate /tmp/test_world` が成功

---

#### TEST-E2E-002: world_bundle → ROS2シミュレーション

**優先度**: P0（Critical）

**目的**: world_bundleからROS2シミュレーションを起動できることを確認

**入力**:
- `worlds/minimal_test/`

**手順**:
```bash
ros2 launch gs_ros2_simulator bringup.launch.xml \
    world:=worlds/minimal_test
```

**期待される出力**:
- ノードが起動（終了しない）
- 以下のトピックが配信される:
  - `/clock` @ 100Hz
  - `/tf` @ 100Hz
  - `/odom` @ 100Hz
  - `/camera/front/image_raw` @ 12Hz
  - `/lidar/top/points` @ 20Hz

**検証項目**（30秒実行後）:
```bash
# トピック確認
ros2 topic list | grep -E "/clock|/tf|/odom|/camera|/lidar"

# 周波数確認
ros2 topic hz /clock  # ≈ 100 Hz
ros2 topic hz /camera/front/image_raw  # ≈ 12 Hz
ros2 topic hz /lidar/top/points  # ≈ 20 Hz

# メッセージ確認
ros2 topic echo /odom --once
```

- [ ] すべてのトピックが存在
- [ ] 周波数が仕様通り（±10%以内）
- [ ] メッセージの内容が妥当（NaN/Infなし）

---

#### TEST-E2E-003: ダミー制御での走行テスト

**優先度**: P0（Critical）

**目的**: 制御入力に応じてEgoが移動することを確認

**入力**:
- world_bundle: `worlds/minimal_test/`
- 制御入力: 直進（steering=0, speed=5 m/s）

**手順**:
```bash
# シミュレータ起動
ros2 launch gs_ros2_simulator bringup.launch.xml world:=worlds/minimal_test

# 別ターミナルで制御入力を送信
ros2 topic pub /vehicle/control_cmd ackermann_msgs/AckermannDriveStamped \
    "{ drive: { steering_angle: 0.0, speed: 5.0 } }" --rate 10
```

**期待される出力**（10秒後）:
- `/odom` の `pose.position.x` が増加（約50m）
- `/odom` の `pose.position.y` ≈ 0（直進）
- `/tf` の `base_link` が移動

**検証項目**:
```bash
# 初期位置記録
ros2 topic echo /odom --once > /tmp/odom_start.txt

# 10秒待機
sleep 10

# 最終位置記録
ros2 topic echo /odom --once > /tmp/odom_end.txt

# 差分確認
# Δx ≈ 50 m
# Δy ≈ 0 m
```

---

#### TEST-E2E-004: 旋回動作テスト

**優先度**: P1（High）

**目的**: 旋回制御が正しく動作することを確認

**入力**:
- 制御入力: 左旋回（steering=0.1 rad, speed=5 m/s）

**手順**:
```bash
ros2 topic pub /vehicle/control_cmd ackermann_msgs/AckermannDriveStamped \
    "{ drive: { steering_angle: 0.1, speed: 5.0 } }" --rate 10
```

**期待される出力**（5秒後）:
- ヨー角が増加（左旋回）
- y座標が正の方向に移動

**検証項目**:
- [ ] ヨー角 > 0
- [ ] y座標 > 初期値
- [ ] 旋回半径が妥当（≈ 25m）

---

### 2-2. インターフェース統合テスト

#### TEST-INT-001: calibration.yaml ⇄ tf_static.json 整合性

**優先度**: P1（High）

**目的**: calibration.yaml と tf_static.json の整合性を確認

**入力**:
- `worlds/minimal_test/sensors/calibration.yaml`
- `worlds/minimal_test/sensors/tf_static.json`

**手順**:
```python
from gs_world_builder.world_bundle_packer import validate_calibration_consistency

result = validate_calibration_consistency(
    "worlds/minimal_test/sensors/calibration.yaml",
    "worlds/minimal_test/sensors/tf_static.json"
)
```

**期待される出力**:
- `result.is_valid == True`
- すべてのセンサextrinsicsが一致

---

#### TEST-INT-002: ROS2メッセージ型検証

**優先度**: P1（High）

**目的**: 配信されるROS2メッセージが仕様に準拠することを確認

**入力**:
- 実行中のシミュレータ

**手順**:
```python
import rclpy
from sensor_msgs.msg import Image

def callback(msg):
    assert msg.width == 640
    assert msg.height == 480
    assert msg.encoding == 'rgb8'
    assert len(msg.data) == 640 * 480 * 3

node.create_subscription(Image, '/camera/front/image_raw', callback, 10)
```

**期待される出力**:
- すべてのアサーションがパス

---

## 3. 性能テスト（Performance Test）

### 3-1. リアルタイム性能テスト

#### TEST-PERF-001: Real-time Factor測定

**優先度**: P0（Critical）

**目的**: シミュレーションがリアルタイムで動作することを確認

**入力**:
- world_bundle: `worlds/minimal_test/`
- シミュレーション時間: 60秒

**手順**:
```bash
# シミュレータ起動
ros2 launch gs_ros2_simulator bringup.launch.xml world:=worlds/minimal_test

# RTF測定
python3 scripts/measure_rtf.py --duration 60
```

**測定スクリプト**:
```python
import time
import rclpy
from rosgraph_msgs.msg import Clock

start_wall = time.time()
start_sim = None
end_sim = None

def clock_callback(msg):
    global start_sim, end_sim
    if start_sim is None:
        start_sim = msg.clock.sec + msg.clock.nanosec * 1e-9
    end_sim = msg.clock.sec + msg.clock.nanosec * 1e-9

# ... 60秒待機 ...

end_wall = time.time()
rtf = (end_sim - start_sim) / (end_wall - start_wall)
print(f"Real-time Factor: {rtf:.3f}")
```

**期待される出力**:
- Real-time Factor ≥ 0.95（95%以上）

---

#### TEST-PERF-002: ステップ処理時間測定

**優先度**: P1（High）

**目的**: 各ステップの処理時間が許容範囲内であることを確認

**測定項目**:
- Vehicle dynamics update
- Ground contact check
- Camera rendering
- LiDAR generation

**期待される処理時間**:
| 処理 | 目標時間 |
|-----|---------|
| Vehicle dynamics | < 0.1 ms |
| Ground contact | < 0.5 ms |
| Camera rendering | < 20 ms |
| LiDAR raycast | < 5 ms |

**検証方法**:
```python
import time

start = time.perf_counter()
vehicle.update(...)
elapsed_ms = (time.perf_counter() - start) * 1000
assert elapsed_ms < 0.1
```

---

#### TEST-PERF-003: メモリ使用量測定

**優先度**: P1（High）

**目的**: メモリリークがないことを確認

**手順**:
```bash
# シミュレータ起動
ros2 launch gs_ros2_simulator bringup.launch.xml world:=worlds/minimal_test &
PID=$!

# メモリ使用量を10分間監視
for i in {1..600}; do
    ps -o rss= -p $PID >> /tmp/memory_usage.txt
    sleep 1
done

# 増加傾向を確認
python3 scripts/check_memory_leak.py /tmp/memory_usage.txt
```

**期待される出力**:
- メモリ使用量が定常状態に収束（増加し続けない）
- 最大メモリ: < 4 GB

---

### 3-2. スケーラビリティテスト

#### TEST-PERF-004: 大規模world_bundle

**優先度**: P2（Medium）

**目的**: 最大サイズworld_bundleでの性能確認

**入力**:
- Gaussian数: 5,000,000（最大）
- heightmap: 4096×4096（最大）
- カメラ: 4台
- LiDAR: 2台

**期待される性能**:
- 起動時間: < 60秒
- Real-time Factor: ≥ 0.8
- メモリ使用量: < 4 GB

---

#### TEST-PERF-005: 最小world_bundle

**優先度**: P1（High）

**目的**: 最小構成での性能ベースライン確認

**入力**:
- Gaussian数: 100（最小）
- heightmap: 128×128（最小）
- カメラ: 1台
- LiDAR: 1台

**期待される性能**:
- 起動時間: < 5秒
- Real-time Factor: ≥ 1.0（リアルタイム超過）
- メモリ使用量: < 500 MB

---

### 3-3. 並列実行ガイドライン

### 並列実行可能なテスト

**単体テスト**: すべて並列実行可能
- ファイルI/Oは一意のディレクトリを使用（UUID生成）
- ROS2ノードは使用しない

**統合テスト**: 条件付きで並列実行可能
- ROS2ノード名に UUID を付与
- ポート番号の競合を回避（DDS設定）

### 並列実行時の注意事項

**ファイルシステムの競合回避**:
```python
import uuid
import tempfile

# 一意のディレクトリ生成
test_dir = tempfile.mkdtemp(prefix=f"test_{uuid.uuid4().hex[:8]}_")
```

**ROS2ノード名の競合回避**:
```bash
# ノード名にUUIDを付与
ros2 launch gs_ros2_simulator bringup.launch.xml \
    world:=worlds/minimal_test \
    node_name:=sim_node_${RANDOM}
```

**並列実行コマンド**:
```bash
# pytestで並列実行（-n auto で自動並列数決定）
pytest tests/unit/ -n auto

# 統合テストは順次実行推奨
pytest tests/integration/ -n 1
```

---

## 4. テスト実行管理

### 4-1. テストコマンド

```bash
# すべての単体テスト実行
colcon test --packages-select gs_world_builder gs_ros2_simulator

# 統合テスト実行
pytest tests/integration/

# 性能テスト実行
pytest tests/performance/ --benchmark-only
```

---

### 4-2. CI/CD統合

**GitHub Actions ワークフロー例**:

```yaml
name: Test Suite

on: [push, pull_request]

jobs:
  unit-tests:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3
      - name: Install dependencies
        run: |
          sudo apt-get install ros-humble-desktop
          pip install -r requirements.txt
      - name: Run unit tests
        run: colcon test

  integration-tests:
    runs-on: ubuntu-22.04
    needs: unit-tests
    steps:
      - name: Generate test data
        run: bash scripts/generate_test_data.sh
      - name: Run E2E tests
        run: pytest tests/integration/

  performance-tests:
    runs-on: ubuntu-22.04
    needs: integration-tests
    steps:
      - name: Run performance benchmarks
        run: pytest tests/performance/ --benchmark-only
```

---

## 変更履歴

| バージョン | 日付 | 変更内容 |
|-----------|------|---------|
| 1.0.1 | 2026-02-15 | Smoke Test追加、エラーコード5,6テスト追加、Degraded Modeテスト追加、境界値テスト追加、優先度追加、並列実行ガイドライン追加 |
| 1.0.0 | 2026-02-15 | 初版作成 |
