"""
テスト共通設定

このモジュールは、すべてのテストで使用する共通の設定値を集約します。
仕様変更時は、このファイルのみを修正すればすべてのテストに反映されます。
"""

# ==============================================================================
# カメラ設定
# ==============================================================================

CAMERA_MINIMAL_WIDTH = 640
CAMERA_MINIMAL_HEIGHT = 480
CAMERA_MINIMAL_FX = 500.0
CAMERA_MINIMAL_FY = 500.0
CAMERA_MINIMAL_CX = 320.0
CAMERA_MINIMAL_CY = 240.0

CAMERA_STANDARD_WIDTH = 1920
CAMERA_STANDARD_HEIGHT = 1080
CAMERA_STANDARD_FX = 1500.0
CAMERA_STANDARD_FY = 1500.0
CAMERA_STANDARD_CX = 960.0
CAMERA_STANDARD_CY = 540.0

CAMERA_ENCODING_RGB8 = "rgb8"
CAMERA_ENCODING_BGR8 = "bgr8"
CAMERA_ENCODING_MONO8 = "mono8"

CAMERA_RATE_HZ = 12.0

# ==============================================================================
# LiDAR設定
# ==============================================================================

LIDAR_MINIMAL_CHANNELS = 32
LIDAR_STANDARD_CHANNELS = 128
LIDAR_MAX_RANGE = 100.0
LIDAR_MIN_RANGE = 0.5
LIDAR_HORIZONTAL_RESOLUTION = 0.4  # [deg]
LIDAR_VERTICAL_FOV = (-15.0, 15.0)  # [deg]
LIDAR_RATE_HZ = 20.0

LIDAR_MAX_POINTS = 200000

# ==============================================================================
# Vehicle Dynamics設定
# ==============================================================================

VEHICLE_WHEELBASE = 2.5  # [m]
VEHICLE_MAX_SPEED = 30.0  # [m/s]
VEHICLE_MAX_STEERING_ANGLE = 0.52  # [rad] ≈ 30 deg

# ==============================================================================
# シミュレーション設定
# ==============================================================================

SIM_DT = 0.01  # [s] 100Hz
SIM_RATE_HZ = 100.0  # [Hz]

CONTROL_TIMEOUT_SEC = 1.0  # [s]
EMERGENCY_DECELERATION = 3.0  # [m/s²]

# ==============================================================================
# Heightmap設定
# ==============================================================================

HEIGHTMAP_MINIMAL_SIZE = 128
HEIGHTMAP_STANDARD_SIZE = 1024
HEIGHTMAP_LARGE_SIZE = 2048
HEIGHTMAP_MAX_SIZE = 4096

HEIGHTMAP_RESOLUTION = 0.1  # [m/cell]

# ==============================================================================
# Gaussian設定
# ==============================================================================

GAUSSIAN_MINIMAL_COUNT = 100
GAUSSIAN_STANDARD_COUNT = 100000
GAUSSIAN_LARGE_COUNT = 1000000
GAUSSIAN_MAX_COUNT = 5000000

GAUSSIAN_SH_DEGREE = 3

# ==============================================================================
# Drivable Area設定
# ==============================================================================

DRIVABLE_BUFFER_WIDTH = 5.0  # [m]

# ==============================================================================
# 許容誤差
# ==============================================================================

# 幾何精度
TOLERANCE_POSITION_XY = 0.01  # [m]
TOLERANCE_POSITION_Z = 0.05   # [m]
TOLERANCE_YAW = 0.01          # [rad]
TOLERANCE_ROLL_PITCH = 0.001  # [rad]
TOLERANCE_QUATERNION_NORM = 1e-6

# 速度精度
TOLERANCE_LINEAR_VELOCITY = 0.01   # [m/s]
TOLERANCE_ANGULAR_VELOCITY = 0.001 # [rad/s]
TOLERANCE_ACCELERATION = 0.1       # [m/s²]

# 時刻精度
TOLERANCE_TIMESTAMP_MS = 1.0       # [ms]
TOLERANCE_SENSOR_TRIGGER_MS = 5.0  # [ms]

# ==============================================================================
# パフォーマンス基準
# ==============================================================================

# Real-time Factor
RTF_THRESHOLD_MINIMAL = 0.95
RTF_THRESHOLD_STANDARD = 0.90
RTF_THRESHOLD_LARGE = 0.80

# ステップ処理時間 [ms]
STEP_TIME_VEHICLE_DYNAMICS_MAX = 0.1
STEP_TIME_GROUND_CONTACT_MAX = 0.5
STEP_TIME_SENSOR_TRIGGER_MAX = 0.1
STEP_TIME_CAMERA_RENDERING_MAX = 20.0
STEP_TIME_LIDAR_RAYCAST_MAX = 5.0
STEP_TIME_ROS2_PUBLISH_MAX = 1.0
STEP_TIME_TOTAL_MAX = 10.0  # 100Hz

# メモリ使用量 [MB]
MEMORY_USAGE_AFTER_LOAD_MAX = 2048
MEMORY_USAGE_RUNTIME_MAX = 4096
MEMORY_LEAK_THRESHOLD_MB_PER_SEC = 0.1

# GPU使用量
GPU_MEMORY_MAX_MB = 4096

# ==============================================================================
# テストデータパス
# ==============================================================================

import os

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

WORLDS_DIR = os.path.join(PROJECT_ROOT, "worlds")
TEST_DATA_DIR = os.path.join(PROJECT_ROOT, "test_data")

# world_bundle
WORLD_MINIMAL_TEST = os.path.join(WORLDS_DIR, "minimal_test")
WORLD_INVALID_MISSING_HEIGHTMAP = os.path.join(WORLDS_DIR, "invalid_missing_heightmap")
WORLD_INVALID_CALIBRATION_MISMATCH = os.path.join(WORLDS_DIR, "invalid_calibration_mismatch")
WORLD_INVALID_EMPTY_DRIVABLE = os.path.join(WORLDS_DIR, "invalid_empty_drivable")

# MCAP
MCAP_MINIMAL_TEST = os.path.join(TEST_DATA_DIR, "minimal_test.mcap")
MCAP_CORRUPTED = os.path.join(TEST_DATA_DIR, "corrupted.mcap")
MCAP_MISSING_TOPICS = os.path.join(TEST_DATA_DIR, "missing_topics.mcap")

# ==============================================================================
# ROS2フレーム名
# ==============================================================================

FRAME_MAP = "map"
FRAME_ODOM = "odom"
FRAME_BASE_LINK = "base_link"
FRAME_CAMERA_FRONT = "camera_front"
FRAME_LIDAR_TOP = "lidar_top"

# ==============================================================================
# エラーコード
# ==============================================================================

EXIT_CODE_SUCCESS = 0
EXIT_CODE_CONFIG_ERROR = 1
EXIT_CODE_INPUT_ERROR = 2
EXIT_CODE_PROCESSING_ERROR = 3
EXIT_CODE_TOPIC_MISSING = 4
EXIT_CODE_VALIDATION_ERROR = 5
EXIT_CODE_RESOURCE_ERROR = 6

# ==============================================================================
# テスト優先度
# ==============================================================================

PRIORITY_CRITICAL = "P0"  # 必須機能、リリースブロッカー
PRIORITY_HIGH = "P1"      # 重要機能
PRIORITY_MEDIUM = "P2"    # 通常機能
PRIORITY_LOW = "P3"       # エッジケース

# ==============================================================================
# ヘルパー関数
# ==============================================================================

def assert_position_close(actual, expected, atol_xy=None, atol_z=None):
    """位置の近似比較"""
    if atol_xy is None:
        atol_xy = TOLERANCE_POSITION_XY
    if atol_z is None:
        atol_z = TOLERANCE_POSITION_Z

    assert abs(actual.x - expected.x) < atol_xy, \
        f"x mismatch: {actual.x} vs {expected.x} (tolerance: {atol_xy})"
    assert abs(actual.y - expected.y) < atol_xy, \
        f"y mismatch: {actual.y} vs {expected.y} (tolerance: {atol_xy})"
    assert abs(actual.z - expected.z) < atol_z, \
        f"z mismatch: {actual.z} vs {expected.z} (tolerance: {atol_z})"


def assert_yaw_close(actual, expected, atol=None):
    """ヨー角の近似比較（-π ~ π で正規化）"""
    import numpy as np

    if atol is None:
        atol = TOLERANCE_YAW

    diff = (actual - expected + np.pi) % (2 * np.pi) - np.pi
    assert abs(diff) < atol, \
        f"yaw mismatch: {actual} vs {expected} (diff: {diff}, tolerance: {atol})"


def assert_quaternion_normalized(q, atol=None):
    """クォータニオンの正規化チェック"""
    import numpy as np

    if atol is None:
        atol = TOLERANCE_QUATERNION_NORM

    norm = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    assert abs(norm - 1.0) < atol, \
        f"quaternion not normalized: {norm} (tolerance: {atol})"


def assert_timestamp_close(t1, t2, atol_ms=None):
    """タイムスタンプの近似比較"""
    if atol_ms is None:
        atol_ms = TOLERANCE_TIMESTAMP_MS

    t1_sec = t1.sec + t1.nanosec * 1e-9
    t2_sec = t2.sec + t2.nanosec * 1e-9
    diff_ms = abs(t1_sec - t2_sec) * 1000

    assert diff_ms < atol_ms, \
        f"timestamp mismatch: {diff_ms} ms (tolerance: {atol_ms} ms)"
