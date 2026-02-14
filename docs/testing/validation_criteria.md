# 検証基準

**バージョン**: 1.0.0
**最終更新**: 2026-02-15

## 概要

このドキュメントは、GS-ROS2 Simulator の各種検証基準を定義します。

**内容**:
1. **ROS2メッセージの検証基準**: 各フィールドの期待値・制約
2. **許容誤差の定義**: 位置、角度、時刻等の許容範囲
3. **パフォーマンス測定基準**: リアルタイム性能・リソース使用量の基準
4. **成功/失敗の判定基準**: テスト合格ラインの定義

---

## 1. ROS2メッセージの検証基準

### 1-1. Image (sensor_msgs/Image)

#### 必須フィールド

| フィールド | 型 | 期待値 | 制約 |
|----------|----|---------| -----|
| `header.stamp` | Time | シミュレーション時刻 | 単調増加（MUST） |
| `header.frame_id` | string | `"camera_front"` 等 | 空でない（MUST） |
| `width` | uint32 | 640, 1920 等 | > 0（MUST） |
| `height` | uint32 | 480, 1080 等 | > 0（MUST） |
| `encoding` | string | `"rgb8"` | 有効なencoding（MUST） |
| `is_bigendian` | uint8 | 0 | Little Endian |
| `step` | uint32 | `width * 3` | `width * channels`（MUST） |
| `data` | uint8[] | 画像データ | `len(data) == height * step`（MUST） |

#### サポートされるencoding

| encoding | channels | bytes/pixel |
|----------|---------|-------------|
| `rgb8` | 3 | 3 |
| `bgr8` | 3 | 3 |
| `mono8` | 1 | 1 |

#### 検証スクリプト（Python）

```python
def validate_image(msg: Image):
    """Image メッセージの検証"""

    # 必須フィールドチェック
    assert msg.width > 0, "width must be > 0"
    assert msg.height > 0, "height must be > 0"
    assert msg.encoding in ["rgb8", "bgr8", "mono8"], f"Invalid encoding: {msg.encoding}"
    assert msg.header.frame_id != "", "frame_id must not be empty"

    # サイズ整合性
    if msg.encoding == "rgb8":
        expected_step = msg.width * 3
        expected_size = msg.height * expected_step
    elif msg.encoding == "mono8":
        expected_step = msg.width
        expected_size = msg.height * expected_step

    assert msg.step == expected_step, f"step mismatch: {msg.step} != {expected_step}"
    assert len(msg.data) == expected_size, f"data size mismatch: {len(msg.data)} != {expected_size}"

    # タイムスタンプ
    assert msg.header.stamp.sec >= 0, "timestamp must be >= 0"

    print("✅ Image validation passed")
```

---

### 1-2. CameraInfo (sensor_msgs/CameraInfo)

#### 必須フィールド

| フィールド | 期待値 | 制約 |
|----------|--------|------|
| `header` | Image と同じ | frame_id 一致（MUST） |
| `width` | Image と同じ | 一致（MUST） |
| `height` | Image と同じ | 一致（MUST） |
| `distortion_model` | `"plumb_bob"` or `"equidistant"` | サポート対象モデル |
| `d` | [k1, k2, p1, p2, k3] | 長さ 5（plumb_bob） |
| `k` | [fx, 0, cx, 0, fy, cy, 0, 0, 1] | 長さ 9、fx/fy > 0 |
| `r` | 単位行列 or 回転行列 | 長さ 9 |
| `p` | [fx', 0, cx', Tx, 0, fy', cy', Ty, 0, 0, 1, 0] | 長さ 12 |

#### 検証スクリプト

```python
def validate_camera_info(msg: CameraInfo):
    """CameraInfo メッセージの検証"""

    # サイズ
    assert msg.width > 0 and msg.height > 0

    # Intrinsics (K matrix)
    assert len(msg.k) == 9
    fx, fy = msg.k[0], msg.k[4]
    cx, cy = msg.k[2], msg.k[5]
    assert fx > 0 and fy > 0, "fx, fy must be > 0"
    assert 0 < cx < msg.width, f"cx out of range: {cx}"
    assert 0 < cy < msg.height, f"cy out of range: {cy}"

    # Distortion
    assert msg.distortion_model in ["plumb_bob", "equidistant"]
    if msg.distortion_model == "plumb_bob":
        assert len(msg.d) == 5

    # R matrix
    assert len(msg.r) == 9

    # P matrix
    assert len(msg.p) == 12

    print("✅ CameraInfo validation passed")
```

---

### 1-3. PointCloud2 (sensor_msgs/PointCloud2)

#### 必須フィールド

| フィールド | 期待値 | 制約 |
|----------|--------|------|
| `header.stamp` | シミュレーション時刻 | 単調増加 |
| `header.frame_id` | `"lidar_top"` 等 | 空でない |
| `height` | 1 | Unorganized cloud |
| `width` | 点数 | > 0 |
| `fields` | [x, y, z, intensity, ring...] | 最低4つ（x,y,z,intensity） |
| `is_bigendian` | False | Little Endian |
| `point_step` | 16 or 20 | フィールドサイズの合計 |
| `row_step` | `width * point_step` | 一致（MUST） |
| `data` | 点群データ | `len(data) == row_step`（MUST） |
| `is_dense` | True or False | NaN/Inf の有無 |

#### サポートされるfield構成

| 構成 | fields | point_step |
|-----|--------|------------|
| XYZI | x, y, z, intensity | 16 bytes |
| XYZIR | x, y, z, intensity, ring | 20 bytes |

#### 検証スクリプト

```python
def validate_pointcloud2(msg: PointCloud2):
    """PointCloud2 メッセージの検証"""

    # 基本フィールド
    assert msg.height == 1, "height must be 1 (unorganized)"
    assert msg.width > 0, "width must be > 0"
    assert msg.header.frame_id != ""

    # Fields
    field_names = [f.name for f in msg.fields]
    assert 'x' in field_names and 'y' in field_names and 'z' in field_names
    assert 'intensity' in field_names

    # データサイズ
    expected_size = msg.width * msg.point_step
    assert len(msg.data) == expected_size, f"data size mismatch: {len(msg.data)} != {expected_size}"

    # 点数範囲
    assert 0 <= msg.width <= 200000, f"point count out of range: {msg.width}"

    print("✅ PointCloud2 validation passed")
```

---

### 1-4. Odometry (nav_msgs/Odometry)

#### 必須フィールド

| フィールド | 期待値 | 制約 |
|----------|--------|------|
| `header.stamp` | シミュレーション時刻 | 単調増加 |
| `header.frame_id` | `"odom"` | 固定 |
| `child_frame_id` | `"base_link"` | 固定 |
| `pose.pose.position` | (x, y, z) | world内に存在 |
| `pose.pose.orientation` | quaternion [x,y,z,w] | 正規化（MUST） |
| `twist.twist.linear` | (vx, vy, vz) | 速度制限内 |
| `twist.twist.angular` | (wx, wy, wz) | 角速度制限内 |

#### 検証スクリプト

```python
import numpy as np

def validate_odometry(msg: Odometry):
    """Odometry メッセージの検証"""

    # Frame IDs
    assert msg.header.frame_id == "odom"
    assert msg.child_frame_id == "base_link"

    # クォータニオン正規化
    q = msg.pose.pose.orientation
    norm = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    assert abs(norm - 1.0) < 1e-6, f"quaternion not normalized: {norm}"

    # 速度範囲（例: max 30 m/s）
    vx = msg.twist.twist.linear.x
    assert abs(vx) <= 30.0, f"linear velocity out of range: {vx}"

    # 角速度範囲（例: max 1 rad/s）
    wz = msg.twist.twist.angular.z
    assert abs(wz) <= 1.0, f"angular velocity out of range: {wz}"

    print("✅ Odometry validation passed")
```

---

### 1-5. TFMessage (tf2_msgs/TFMessage)

#### 必須フィールド

| フィールド | 期待値 | 制約 |
|----------|--------|------|
| `transforms` | TransformStamped[] | 長さ > 0 |
| `transforms[].header.stamp` | シミュレーション時刻 | 単調増加 |
| `transforms[].header.frame_id` | 親フレーム | 空でない |
| `transforms[].child_frame_id` | 子フレーム | 空でない、親と異なる |
| `transforms[].transform.rotation` | quaternion | 正規化（MUST） |

#### 検証スクリプト

```python
def validate_tf(msg: TFMessage):
    """TFMessage の検証"""

    assert len(msg.transforms) > 0, "transforms must not be empty"

    for tf in msg.transforms:
        # Frame IDs
        assert tf.header.frame_id != ""
        assert tf.child_frame_id != ""
        assert tf.header.frame_id != tf.child_frame_id

        # クォータニオン正規化
        q = tf.transform.rotation
        norm = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        assert abs(norm - 1.0) < 1e-6, f"quaternion not normalized: {norm}"

    print("✅ TFMessage validation passed")
```

---

## 2. 許容誤差の定義

### 2-1. 幾何精度

| 項目 | 許容誤差 | 根拠 |
|-----|---------|------|
| **位置精度（x, y）** | ±0.01 m | シミュレーション精度 |
| **高さ精度（z）** | ±0.05 m | heightmap解像度依存 |
| **ヨー角精度** | ±0.01 rad（≈0.57°） | Vehicle dynamics精度 |
| **ロール・ピッチ** | ±0.001 rad（≈0.057°） | 平坦地面想定 |
| **クォータニオン正規化** | ±1e-6 | 浮動小数点精度 |

#### 検証例

```python
def assert_position_close(actual, expected, atol=0.01):
    """位置の近似比較"""
    assert abs(actual.x - expected.x) < atol, f"x mismatch: {actual.x} vs {expected.x}"
    assert abs(actual.y - expected.y) < atol, f"y mismatch: {actual.y} vs {expected.y}"
    assert abs(actual.z - expected.z) < 0.05, f"z mismatch: {actual.z} vs {expected.z}"

def assert_yaw_close(actual, expected, atol=0.01):
    """ヨー角の近似比較"""
    diff = (actual - expected + np.pi) % (2 * np.pi) - np.pi  # [-pi, pi]
    assert abs(diff) < atol, f"yaw mismatch: {actual} vs {expected}"
```

---

### 2-2. 時刻精度

| 項目 | 許容誤差 | 根拠 |
|-----|---------|------|
| **シミュレーション時刻の進行** | ±1 ms | タイマー精度 |
| **センサトリガータイミング** | ±5 ms | sim_dtの半分 |
| **メッセージタイムスタンプ** | ±1 ms | ROS2時刻精度 |
| **TF時刻同期** | ±1 ms | 同一ステップ内 |

#### 検証例

```python
def assert_timestamp_close(t1, t2, atol_ms=1.0):
    """タイムスタンプの近似比較"""
    t1_sec = t1.sec + t1.nanosec * 1e-9
    t2_sec = t2.sec + t2.nanosec * 1e-9
    diff_ms = abs(t1_sec - t2_sec) * 1000
    assert diff_ms < atol_ms, f"timestamp mismatch: {diff_ms} ms"
```

---

### 2-3. 速度・加速度精度

| 項目 | 許容誤差 | 根拠 |
|-----|---------|------|
| **線速度** | ±0.01 m/s | 数値積分誤差 |
| **角速度** | ±0.001 rad/s | 数値積分誤差 |
| **加速度** | ±0.1 m/s² | 制御入力の離散化 |

---

## 3. パフォーマンス測定基準

### 3-1. リアルタイム性能

#### Real-time Factor（RTF）

**定義**:
```
RTF = シミュレーション時間 / 実時間
```

**基準**:
| 条件 | RTF要件 | 判定 |
|-----|---------|------|
| **minimal_test** | ≥ 0.95 | 必須（MUST） |
| **標準world_bundle** | ≥ 0.90 | 推奨 |
| **大規模world_bundle** | ≥ 0.80 | 許容 |

**測定方法**:
```python
def measure_rtf(duration_sec=60):
    """RTFを測定"""
    import time

    start_wall = time.time()
    start_sim = get_sim_time()  # /clock から取得

    time.sleep(duration_sec)

    end_wall = time.time()
    end_sim = get_sim_time()

    rtf = (end_sim - start_sim) / (end_wall - start_wall)
    return rtf

# 検証
rtf = measure_rtf(60)
assert rtf >= 0.95, f"RTF too low: {rtf}"
```

---

#### ステップ処理時間

**基準**:
| 処理 | 目標時間 | 最大許容時間 |
|-----|---------|-------------|
| **Vehicle dynamics** | < 0.1 ms | 1 ms |
| **Ground contact** | < 0.5 ms | 2 ms |
| **Sensor trigger** | < 0.1 ms | 1 ms |
| **Camera rendering** | < 20 ms | 30 ms |
| **LiDAR raycast** | < 5 ms | 10 ms |
| **ROS2 publish** | < 1 ms | 5 ms |
| **合計（100Hz）** | < 8 ms | 10 ms |

**測定方法**:
```python
import time

def benchmark_step(func, num_iterations=1000):
    """処理時間のベンチマーク"""
    times = []

    for _ in range(num_iterations):
        start = time.perf_counter()
        func()
        end = time.perf_counter()
        times.append((end - start) * 1000)  # ms

    avg_ms = np.mean(times)
    max_ms = np.max(times)

    return {
        'avg_ms': avg_ms,
        'max_ms': max_ms,
        'p95_ms': np.percentile(times, 95),
        'p99_ms': np.percentile(times, 99)
    }

# 検証
result = benchmark_step(vehicle.update)
assert result['p95_ms'] < 0.1, f"Vehicle dynamics too slow: {result}"
```

---

### 3-2. リソース使用量

#### メモリ使用量

**基準**:
| 項目 | 上限 | 判定 |
|-----|------|------|
| **world_bundle読み込み後** | 2 GB | 必須 |
| **シミュレーション実行中** | 4 GB | 必須 |
| **メモリリーク** | なし（定常） | 必須 |

**測定方法**:
```python
import psutil
import os

def measure_memory_usage(duration_sec=600):
    """メモリ使用量を測定"""
    pid = os.getpid()
    process = psutil.Process(pid)

    memory_usage = []

    for _ in range(duration_sec):
        mem_mb = process.memory_info().rss / 1024 / 1024
        memory_usage.append(mem_mb)
        time.sleep(1)

    # トレンド分析（線形回帰）
    from scipy.stats import linregress
    x = np.arange(len(memory_usage))
    slope, _, _, _, _ = linregress(x, memory_usage)

    # メモリリークの判定: 傾き > 0.1 MB/sec
    has_leak = slope > 0.1

    return {
        'initial_mb': memory_usage[0],
        'final_mb': memory_usage[-1],
        'max_mb': max(memory_usage),
        'slope_mb_per_sec': slope,
        'has_leak': has_leak
    }

# 検証
result = measure_memory_usage(600)
assert result['max_mb'] < 4096, f"Memory usage too high: {result['max_mb']} MB"
assert not result['has_leak'], f"Memory leak detected: {result['slope_mb_per_sec']} MB/s"
```

---

#### GPU使用率

**基準**:
| 項目 | 期待値 | 判定 |
|-----|-------|------|
| **GPU使用率（Camera有効時）** | 30-80% | 正常 |
| **GPU メモリ使用量** | < 4 GB | 必須 |

**測定方法**:
```python
import pynvml

def measure_gpu_usage():
    """GPU使用率を測定"""
    pynvml.nvmlInit()
    handle = pynvml.nvmlDeviceGetHandleByIndex(0)

    util = pynvml.nvmlDeviceGetUtilizationRates(handle)
    mem_info = pynvml.nvmlDeviceGetMemoryInfo(handle)

    return {
        'gpu_util_percent': util.gpu,
        'memory_used_mb': mem_info.used / 1024 / 1024
    }

# 検証
result = measure_gpu_usage()
assert result['memory_used_mb'] < 4096, f"GPU memory too high: {result['memory_used_mb']} MB"
```

---

## 4. 成功/失敗の判定基準

### 4-1. 単体テストの合格基準

| 項目 | 合格基準 |
|-----|---------|
| **すべてのアサーションがパス** | 必須 |
| **例外が発生しない** | 正常系では必須 |
| **終了コードが0** | 正常系では必須 |
| **処理時間が基準内** | 性能テストでは必須 |

---

### 4-2. 統合テストの合格基準

| 項目 | 合格基準 |
|-----|---------|
| **E2E処理が完了** | 必須 |
| **すべての必須ファイルが生成** | 必須 |
| **validation が成功** | 必須 |
| **ROS2トピックが配信される** | 必須 |
| **メッセージが検証基準を満たす** | 必須 |

---

### 4-3. 性能テストの合格基準

| 項目 | 合格基準 |
|-----|---------|
| **RTF ≥ 0.95** | minimal_testでは必須 |
| **ステップ処理時間 < 10 ms (p95)** | 必須 |
| **メモリ使用量 < 4 GB** | 必須 |
| **メモリリークなし** | 必須 |

---

## 5. テスト結果の記録

### 5-1. 結果フォーマット（JSON）

```json
{
  "test_name": "TEST-SIM-001",
  "test_type": "unit",
  "timestamp": "2026-02-15T10:30:00Z",
  "status": "passed",
  "duration_sec": 1.234,

  "metrics": {
    "rtf": 0.98,
    "memory_mb": 1024,
    "step_time_ms": 7.5
  },

  "assertions": {
    "total": 10,
    "passed": 10,
    "failed": 0
  },

  "errors": []
}
```

---

### 5-2. 継続的監視

**メトリクス収集**:
- すべてのテスト実行で性能メトリクスを記録
- 時系列データとして保存
- 回帰検出（性能劣化）

**アラート条件**:
- RTF < 0.9（2回連続）
- メモリ使用量 > 3.5 GB
- テスト失敗率 > 5%

---

## 変更履歴

| バージョン | 日付 | 変更内容 |
|-----------|------|---------|
| 1.0.0 | 2026-02-15 | 初版作成 |
