以下は、このプロジェクト全体を俯瞰できる **README（簡潔版）** です。
研究プロジェクトとしても公開可能な体裁を意識しています。

---

# GS-ROS2 Simulator

**3D Gaussian Splatting (3DGS) ベースの ROS2 ネイティブ自動運転シミュレータ**

mcap 形式の実走行ログから背景 3DGS を生成し、
そのフォトリアル空間上で ego 車両を **closed-loop（カメラ・LiDAR入力）** で走行させるシステム。

---

## 🎯 目的

* 実走行データ（mcap）からフォトリアルな背景 3DGS を生成
* ROS2 対応の自動運転スタックをそのまま接続可能
* Camera + LiDAR 入力を生成
* Ego 車両を自由に走行可能
* 将来的なセンサ忠実度向上（rolling shutter / Gaussian LiDAR）にも拡張可能

---

# 🏗 プロジェクト構成

```
# ROS2パッケージ
gs_world_builder/     # mcap → world_bundle 生成
gs_ros2_simulator/    # ROS2 closed-loop simulator
gs_sim_msgs/          # ROS2 msg / srv 定義

# データ
worlds/               # 生成された world_bundle
```

---

# 📦 World Bundle

シミュレータは `world_bundle` を読み込んで起動します。

```
worlds/<scene_id>/
  world.yaml
  gaussians/
    background.splat.ply
  geometry/
    heightmap.bin
    drivable.geojson
    static_mesh.glb        # optional
  sensors/
    calibration.yaml
    tf_static.json
  sim/
    timebase.yaml
  metadata.json
```

### 設計思想

* **3DGSは“見え”**
* **走行・衝突・LiDARは幾何（heightmap / mesh）**
* DriveStudio依存は builder 側で閉じる
* Runtimeは world_bundle のみを参照

---

# 🔁 システムフロー

## Offline

```
mcap
  ↓
gs_world_builder
  ↓
world_bundle
```

### 主な処理

* MCAP取り込み
* キャリブレーション整理
* DriveStudio用変換
* 背景3DGS生成
* 走行用heightmap / drivable生成
* world_bundle出力

---

## Runtime（ROS2）

```
world_bundle
  ↓
gs_ros2_simulator
  ↓
/camera/image
/lidar/points
/tf
/odom
/clock
```

### 構成モジュール

* Simulation Clock
* Ego Vehicle Dynamics（kinematic bicycle）
* Collision & Drivable
* 3DGS Camera Renderer
* LiDAR Raycast Generator
* ROS2 Bridge

---

# 🚗 Closed-Loop 実行

```
AD Stack
   ↑           ↓
 camera / lidar
   ↑           ↓
gs_ros2_simulator
```

* AD から制御入力を受け取る
* Ego状態を更新
* 次フレームのセンサ出力を生成
* ループ継続

---

# 🔧 起動例

```bash
ros2 launch gs_ros2_simulator bringup.launch.xml \
    world:=worlds/nuscenes_scene_001
```

---

# 🧠 設計原則

1. **World生成とRuntimeを分離**
2. **GSはレンダ専用、物理は幾何専用**
3. **ROS2インタフェースを先に固定**
4. **段階的拡張（Raycast → Gaussian LiDAR）**

---

# 🚀 将来拡張

* SplatAD / neurad-studio による Gaussian LiDAR
* rolling shutter camera
* 動的アクター追加
* HDMap統合
* 交通シナリオエンジン

---

# 📌 現在のスコープ

* 背景3DGS生成
* Egoのみのclosed-loop
* Camera + LiDAR
* ROS2統合

---

# 📚 開発者向けドキュメント

## 必読: インターフェース契約

各コンポーネントを開発する前に、以下のインターフェース定義を必ず確認してください：

### 共通規約（最重要）
* **[docs/conventions.md](docs/conventions.md)** - 座標系、単位系、バイナリフォーマット等の共通規約

### コンポーネント間インターフェース契約
* **[docs/interfaces/](docs/interfaces/)** - インターフェース契約の概要
  * **[ros2_messages.md](docs/interfaces/ros2_messages.md)** - ROS2メッセージ・サービス定義
  * **[world_bundle_schema.md](docs/interfaces/world_bundle_schema.md)** - World Bundle データフォーマット
  * **[offline_builder.md](docs/interfaces/offline_builder.md)** - MCAP → world_bundle 変換契約
  * **[runtime_simulator.md](docs/interfaces/runtime_simulator.md)** - world_bundle → ROS2 契約

### アーキテクチャ設計
* **[docs/architecture.md](docs/architecture.md)** - 詳細なコンポーネント分割設計

## 開発の進め方

1. **共通規約の理解**: `docs/conventions.md` を熟読
2. **インターフェース確認**: 開発するコンポーネントの `docs/interface.md` を確認
3. **独立開発**: インターフェース契約に従えば、各コンポーネントは並行開発可能
4. **統合**: インターフェースが守られていれば、統合時の問題を最小化