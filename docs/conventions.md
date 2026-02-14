# GS-ROS2 Simulator 共通規約

**バージョン**: 1.0.2
**最終更新**: 2026-02-15

## 概要

このドキュメントは、GS-ROS2 Simulatorの全コンポーネントで共通して使用される
規約・データフォーマット・座標系などを定義します。

**すべてのコンポーネントはこの規約に従わなければなりません。**

---

## 座標系規約

### 基本原則

**ROS2 REP-103 準拠**: https://www.ros.org/reps/rep-0103.html

### フレーム定義

#### map フレーム
- **用途**: 固定ワールド座標系
- **原点**: シーン依存（metadata.jsonで定義）
- **軸方向**:
  - **X軸**: 東 (East)
  - **Y軸**: 北 (North)
  - **Z軸**: 上 (Up)
- **略称**: ENU (East-North-Up)
- **単位**: メートル [m]
- **右手座標系**

#### odom フレーム
- **用途**: オドメトリ座標系（デフォルトでは map と一致）
- **原点**: ego車両の初期位置、または map と同一
- **軸方向**: map と同一（ENU）
- **ドリフト**: 現在のスコープではドリフトなし（map と固定）

#### base_link フレーム
- **用途**: 車両中心座標系
- **原点**: 車両後輪軸中心（kinematic bicycle model の基準点）
- **軸方向**:
  - **X軸**: 前方 (Forward)
  - **Y軸**: 左 (Left)
  - **Z軸**: 上 (Up)
- **略称**: FLU (Forward-Left-Up)
- **右手座標系**

#### センサフレーム
- **camera_{id}**, **lidar_{id}** など
- **原点**: 各センサの光学中心
- **軸方向**: センサ依存（calibration.yamlで定義）
- **カメラの標準**:
  - X軸: 右、Y軸: 下、Z軸: 前方（光軸）（OpenCV準拠）
  - または X軸: 右、Y軸: 上、Z軸: 後方（ROS準拠）
  - **重要**: `calibration.yaml` の `camera_convention` で明示すること

---

## 単位系

| 物理量 | 単位 | 記号 |
|-------|------|------|
| 長さ・距離 | メートル | [m] |
| 時間 | 秒 | [s] |
| 速度 | メートル毎秒 | [m/s] |
| 加速度 | メートル毎秒毎秒 | [m/s²] |
| 角度 | ラジアン | [rad] |
| 角速度 | ラジアン毎秒 | [rad/s] |
| 角加速度 | ラジアン毎秒毎秒 | [rad/s²] |
| 質量 | キログラム | [kg] |
| 周波数 | ヘルツ | [Hz] |

**注意**:
- 設定ファイルで度数法（degree）を使う場合は、明示的に `_deg` サフィックスを付ける
- 内部計算は常にラジアン

---

## 回転表現

### クォータニオン

**順序**: **[x, y, z, w]** （ROS2標準）

```yaml
# 例: 90度ヨー回転（Z軸周り）
rotation_quat: [0.0, 0.0, 0.707107, 0.707107]  # [x, y, z, w]
```

**正規化**: 常に単位クォータニオン（||q|| = 1）

**恒等回転**: `[0, 0, 0, 1]`

### オイラー角（補助的）

**順序**: **Roll-Pitch-Yaw (intrinsic, Z-Y-X)**

- **Roll**: X軸周りの回転 [rad]
- **Pitch**: Y軸周りの回転 [rad]
- **Yaw**: Z軸周りの回転 [rad]

**注意**: オイラー角はジンバルロックがあるため、内部表現にはクォータニオンを使用

---

## 時刻・タイムスタンプ

### 時刻表現

**ROS2 標準**: `builtin_interfaces/Time`
```
int32 sec    # 秒
uint32 nanosec  # ナノ秒
```

### シミュレーション時刻

- **基準**: シミュレーション開始を `t=0` とする
- **/clock トピック**: `rosgraph_msgs/Clock` で配信
- **use_sim_time**: 常に `true` で運用

### センサ同期

**同一タイムステップのセンサは同一タイムスタンプを持つ**

例:
```
t=0.0000s: Camera (12Hz trigger), LiDAR (20Hz trigger) → 両方とも stamp=0.0000
t=0.0500s: LiDAR (20Hz trigger) → stamp=0.0500
t=0.0833s: Camera (12Hz trigger) → stamp=0.0833
```

**非整数倍の扱い**: センサレートに最も近いシミュステップでトリガー

---

## バイナリデータフォーマット

### エンディアン

**統一規約**: **Little Endian**

- すべてのバイナリファイル（heightmap.bin等）は Little Endian
- x86/x64, ARM の標準に従う

### 浮動小数点

**IEEE 754 準拠**

- `float32`: IEEE 754 単精度浮動小数点（4 bytes）
- `float64`: IEEE 754 倍精度浮動小数点（8 bytes）

### NaN (Not a Number) 表現

**無効値の表現**: `NaN` （IEEE 754 NaN）

```python
# Python例
import numpy as np
invalid_value = np.nan  # float32の場合も np.float32(np.nan)
```

**用途**: heightmap で地面が存在しない領域

---

## ファイル命名・パス規約

### パス表現

**world_bundle 内**: **すべて相対パス**

- `world.yaml` を起点とした相対パス
- 例: `gaussians/background.splat.ply`
- スラッシュ `/` を使用（Windows でも `/`）

**絶対パス禁止**: 可搬性のため

### シンボリックリンク

**サポート**: world_bundle 内のシンボリックリンクはサポート対象
- ただし、リンク先も world_bundle 内に存在すること（外部参照禁止）

### ファイル名規則

- **小文字 + アンダースコア**: `background.splat.ply`, `heightmap.bin`
- **拡張子必須**: `.yaml`, `.json`, `.bin`, `.ply`, `.geojson` など

---

## センサID命名規則

### カメラ

**形式**: `{位置}_{サブ位置}` （オプション）

**ID例**:
- `front`, `left`, `right`, `rear`（単一カメラ）
- `front_left`, `front_right`, `rear_left`, `rear_right`（複数カメラ）

**禁則**:
- スペース、特殊文字禁止
- 数字のみ（`cam_0`）は避けること（意味が不明確）

### LiDAR

**形式**: `{位置}`

**ID例**:
- `top`, `front`, `rear`

---

## データの一貫性規則

### 二重定義の優先順位

#### tf_static.json vs calibration.yaml

**正**: **calibration.yaml が正**

- `calibration.yaml` で定義された extrinsics が正式な値
- `tf_static.json` は `calibration.yaml` から生成される派生物
- 食い違った場合は **エラーとして検出**（validation で弾く）

#### world.yaml vs 個別設定ファイル

**正**: **world.yaml が正**

- `world.yaml` がエントリーポイント
- 個別ファイルへのパスが `world.yaml` に記載されていない場合、そのファイルは無視される

---

## バージョニング規約

### Semantic Versioning

**形式**: `MAJOR.MINOR.PATCH`

- **MAJOR**: 非互換な変更（例: ファイル構造変更）
- **MINOR**: 後方互換な機能追加（例: 新フィールド追加）
- **PATCH**: バグフィックス、ドキュメント修正

### バージョンフィールド

**すべての設定ファイルに version フィールドを含める**

```yaml
version: "1.0.0"
```

```json
{
  "version": "1.0.0",
  ...
}
```

### 互換性チェック

**読み込み側の責務**:
```python
SUPPORTED_MAJOR = 1
SUPPORTED_MINOR_MIN = 0

major, minor, patch = parse_version(data['version'])
if major != SUPPORTED_MAJOR or minor < SUPPORTED_MINOR_MIN:
    raise VersionError(f"Unsupported version: {data['version']}")
```

---

## 制御入力の補間規則

### 基本方針

**Zero-Order Hold (ZOH)**

- 制御入力は次の更新まで保持される
- 線形補間は行わない（ステップ入力を保つ）

### タイムアウト

**制御入力タイムアウト**: デフォルト 1.0秒

- タイムアウト後は **緊急停止**
  - ステア: 現在値保持
  - スロットル: 0.0
  - ブレーキ: 最大（減速度 3.0 m/s²）

---

## センサトリガータイミング

### 非整数倍レートの扱い

**最近傍トリガー**

```
sim_dt = 10ms (100Hz)
camera_rate = 12Hz → 期待間隔 83.33ms

実際のトリガー:
t=0ms (0*10)     → trigger (誤差 0.00ms)
t=80ms (8*10)    → trigger (誤差 -3.33ms)
t=170ms (17*10)  → trigger (誤差 +3.33ms)
t=250ms (25*10)  → trigger (誤差 -0.00ms)
...
```

**許容誤差**: 最大 ±5ms（sim_dt の半分）

### センサ間同期

**同一タイムステップでトリガーされたセンサは同一タイムスタンプを持つ**

---

## Gaussian形式の詳細仕様

**標準フォーマット**: PLY形式（binary_little_endian）

**ファイル名**: `background.splat.ply`

**PLYヘッダ仕様**:
```
ply
format binary_little_endian 1.0
element vertex <N>
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
property float f_rest_0
...
property float f_rest_44
end_header
<binary data>
```

**必須プロパティ順序**:
- 回転: クォータニオン `[x, y, z, w]` → `rot_0, rot_1, rot_2, rot_3`
- SH係数: DC成分（3個）+ 高次成分（sh_degree=3の場合 45個）

**SH次数**: `sh_degree` は `render_config.json` で定義（MUST）

**制約**:
- ASCII形式（`format ascii 1.0`）は**サポート対象外**
- NPZ形式は**サポート対象外**（開発用ツールでの変換は可）

---

## drivable.geojson の詳細仕様

### 基本構造

**座標系**: map frame の X-Y 平面（Z座標は無視）

**MultiPolygon サポート**: 複数の走行可能領域がある場合に使用可能

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
          [[x0, y0], [x1, y1], ..., [x0, y0]]  // 外周（反時計回り）
          // [[...], [...]]  // 穴（時計回り）オプション
        ]
      },
      "properties": {
        "type": "drivable",  // 必須
        "priority": 0        // オプション: 小さいほど優先（デフォルト0）
      }
    }
  ]
}
```

### 複数ポリゴンの優先順位

**priority フィールド**: 数値が小さいほど優先（デフォルト値: 0）

- 同一地点が複数ポリゴンに含まれる場合、priority の小さい方を採用

### 穴あきポリゴン

**対応**: Polygon with holes をサポート

- 外周: 反時計回り (CCW)
- 穴: 時計回り (CW)

### 空の場合の挙動

**drivable.geojson が空（features: []）の場合**: **全域をoffroadとして扱う**

- シミュレータは警告を出すが、起動は継続
- `/sim/status` で常に `is_offroad=true`

---

## エラー処理の統一規約

### 起動時エラー（致命的）

**即座に終了**: 以下の場合は起動を中止

- 必須ファイルの欠損
- フォーマット不正（YAML/JSONパースエラー）
- バージョン非互換
- tf_static と calibration の不一致

### ランタイムエラー（継続可能）

**Degraded Mode**: 以下の場合は警告を出して継続

- センサレンダリング失敗 → 前フレームを再利用
- LiDAR生成失敗 → 空の点群を出力
- 制御入力タイムアウト → 緊急停止

---

## リソース制約（推奨値）

### world_bundle サイズ

| 項目 | 推奨上限 | 理由 |
|-----|---------|------|
| 総サイズ | 2 GB | メモリマッピングの効率 |
| Gaussian数 | 5M points | GPU メモリ（8GB GPU想定） |
| heightmap解像度 | 4096x4096 | メモリ使用量（64MB/map） |
| static_mesh 三角形数 | 10M triangles | レンダリング性能 |

**超過時の対処**: 警告を出すが、処理は継続

---

## 並行アクセス規約

### 読み込み専用

**world_bundle は読み込み専用として扱う**

- 複数プロセスからの同時読み込み: **サポート対象**
- ファイルロック: **不要**
- 書き込み: **禁止**（起動中の world_bundle への書き込みは未定義動作）

---

## テストデータ規約

### Minimal World Bundle

**各コンポーネントは minimal world_bundle でテスト可能であること**

**最小構成**:
```
worlds/minimal_test/
  world.yaml
  metadata.json
  gaussians/
    background.splat.ply   # 100 Gaussians程度
    render_config.json
  geometry/
    heightmap.bin          # 128x128
    heightmap.yaml
    drivable.geojson       # 1 polygon
  sensors/
    calibration.yaml       # 1 camera, 1 lidar
    tf_static.json
  sim/
    timebase.yaml
```

**配布**: リポジトリに `worlds/minimal_test/` として含める

---

## 変更履歴

| バージョン | 日付 | 変更内容 |
|-----------|------|---------|
| 1.0.0 | 2026-02-14 | 初版作成 |
