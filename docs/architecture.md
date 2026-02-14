# アーキテクチャ設計（詳細コンポーネント分割）

**注意**: このドキュメントは設計議論とコンポーネント分割の指針を示すものです。
**正式なインターフェース契約**は以下を参照してください：

- **[conventions.md](conventions.md)** - 共通規約
- **[interfaces/](interfaces/)** - コンポーネント間インターフェース契約
  - **[ros2_messages.md](interfaces/ros2_messages.md)** - ROS2メッセージ定義
  - **[world_bundle_schema.md](interfaces/world_bundle_schema.md)** - World Bundle スキーマ
  - **[offline_builder.md](interfaces/offline_builder.md)** - オフライン処理契約
  - **[runtime_simulator.md](interfaces/runtime_simulator.md)** - ランタイム契約

---

プロジェクトは3つのROS2パッケージ（`gs_world_builder / gs_ros2_simulator / gs_sim_msgs`）とデータディレクトリ（`worlds/`）で構成されます。
**各パッケージ内をさらに自然に分割**します。
（依存関係が見えるように、各サブコンポーネントの **入力/出力** も併記します）

---

# 0) 全体の依存関係（トップレベル）

* `gs_sim_msgs`：ROS2 msg/srv/action 定義（**他から参照されるだけ**）
* `gs_world_builder`：入力 `mcap` → 出力 `world_bundle`（`worlds/` に保存）
* `gs_ros2_simulator`：入力 `world_bundle` → ROS2トピックでセンサ/TF/odomを出す & 制御を受ける
* `worlds/`：生成物（データのみ）

依存の向きはこれが理想です（循環させない）：

* `gs_world_builder` →（生成）→ `worlds/`
* `gs_ros2_simulator` →（読込）→ `worlds/`
* `gs_ros2_simulator` ⇄ `gs_sim_msgs`（メッセージ参照）
* `gs_world_builder` ⇄ `gs_sim_msgs` は基本無し（必要なら最小限の“スキーマ共有”のみ）

---

# 1) gs_world_builder（mcap → world_bundle を作る）

目的：**world_bundleを“再現可能に”生成するパイプライン**
中は大きく「取り込み」「変換」「3DGS」「走行幾何」「バンドル化」に分けるのが自然です。

## 1-1. mcap_ingest（MCAP取り込み・正規化）

* 入力：`*.mcap`
* 出力：

  * `raw_extract/images/` - カメラ画像（デコード済み、必要なら）
  * `raw_extract/lidar/` - LiDAR点群（フレーム単位、PCD/npz形式）
  * `raw_extract/poses.csv` - Ego軌跡（timestamp, x, y, z, qx, qy, qz, qw）
  * `raw_extract/tf/` - TF情報
  * `timeline.json` - 全センサの生タイムスタンプ一覧
  * `frameset.json` - シミュレータ用リサンプリング対応表（カメラ12Hz, LiDAR 20Hz）
  * `timebase.yaml` - センサレート・sim_dt の初期設定
* 役割：

  * MCAP からセンサデータを抽出・デコード
  * topicマッピング（どのtopicがCAM_FRONTか等）
  * `/tf` と `/tf_static` の分離・整理
  * タイムスタンプ整理と時間同期の基盤
  * シミュレーション用タイムベース設定の生成

## 1-2. calibration_builder（キャリブ/座標系の確定）

* 入力：
  * `raw_extract/tf/` - MCAP由来のTF情報
  * MCAP内の camera_info
  * センサ仕様（config.yaml で指定）
* 出力：

  * `calibration.yaml` - センサキャリブレーション（camera intrinsics/extrinsics, lidar extrinsics）
  * `tf_static.json` - 静的TF（calibration.yaml から派生生成）
  * `metadata.json`（部分）- 座標系情報（coordinate_system セクション）
* 役割：

  * カメラintrinsicsの確定（リサイズやundistort対応含む）
  * センサextrinsicsの確定（base_link→センサ）
  * tf_static.json の生成（calibration.yaml と一貫性を保証）
  * 座標系情報の記録（ENU/FLU、原点など）

## 1-3. drivesim_dataset_converter（DriveStudio入力生成）

※あなたが言っている「mcap_gs変換」に相当

* 入力：`raw_extract/ + calibration + frameset`
* 出力：DriveStudio dataset（画像列、pose列、必要な補助情報）
* 役割：

  * keyframe/補間の扱い（camera 12Hz / lidar 20Hz など）
  * 露出揺れがある場合のメタ情報保持（任意）

## 1-4. gs_train_orchestrator（3DGS学習実行・管理）

* 入力：DriveStudio dataset
* 出力：

  * DriveStudioの成果物（checkpoints、gaussiansなど）
  * `training_report.json`（設定・収束指標・最良iter）
* 役割：

  * 再現可能な設定固定（configの凍結、seed、commit hash記録）
  * “背景のみ”のモードを確実に回す（あなたの用途）

## 1-5. gs_exporter（ランタイム用Gaussianフォーマットへ変換）

* 入力：DriveStudio成果物
* 出力：

  * `gaussians/background.splat.ply`（または `background.npz`）
  * `render_config.json`（色補正や露出係数、背景マスク情報など）
* 役割：

  * **runtimeがDriveStudioに依存しない**ための最重要コンポーネント

## 1-6. geometry_builder（走行用幾何を作る）

※あなたが言っている「gs_mesh変換」に相当（ただしGSから直接作るより点群/地図由来が現実的）
中をさらに段階的に分けます。

### 1-6a. pointcloud_integrator（点群統合）

* 入力：
  * `raw_extract/lidar/` - フレーム単位のLiDAR点群
  * `raw_extract/poses.csv` - Ego軌跡
* 出力：
  * `point_cloud_map.pcd` - 統合静的点群地図
* 役割：
  * フレーム単位のLiDAR点群をEgo軌跡を使ってmap frameに変換・統合
  * 動的物体のフィルタリング（オプション、簡易的にはvoxel grid filter）
  * ground_surface_builder と static_mesh_builder の入力として使用

### 1-6b. ground_surface_builder（地面モデル）

* 入力：
  * `point_cloud_map.pcd` - 統合静的点群
  * `raw_extract/poses.csv` - Ego軌跡（範囲決定用）
  * geometry_config.yaml（解像度、範囲）
* 出力：

  * `geometry/heightmap.bin`（2.5D、Little Endian float32配列）
  * `geometry/heightmap.yaml`（heightmapの原点と解像度）
* 役割：
  * 点群から地面高さマップを生成（2.5D投影）
  * メディアンフィルタ・外れ値除去
  * 接地、簡易衝突、LiDAR生成の基盤

### 1-6c. drivable_area_builder（走行可能領域）

* 入力：
  * `raw_extract/poses.csv` - Ego軌跡
  * geometry_config.yaml（バッファ幅）
  * （任意）HDMap（Lanelet2等）
* 出力：
  * `geometry/drivable.geojson`（GeoJSON Polygon）
* 役割：
  * Ego軌跡の周辺をバッファして走行可能領域を生成（最初はこれで十分）
  * ポリゴン簡略化（Douglas-Peucker等）
  * offroad判定、コース制約の基盤

### 1-6d. static_mesh_builder（任意：精密メッシュ）

* 入力：
  * `point_cloud_map.pcd` - 統合点群
  * geometry_config.yaml（voxel_size、method）
* 出力：
  * `geometry/static_mesh.glb`（glTF 2.0 Binary）
* 役割：
  * Poisson再構成またはTSDFで精密メッシュ生成
  * 高精度LiDAR raycast、壁/縁石の衝突判定用
  * **初期実装では省略推奨**（heightmapで代用）

## 1-7. world_bundle_packer（world_bundle生成）

* 入力：
  * `gaussians/background.splat.ply` + `render_config.json`
  * `geometry/heightmap.*` + `drivable.geojson` + （任意）`static_mesh.glb`
  * `calibration.yaml` + `tf_static.json`
  * `timebase.yaml`
  * `metadata.json`（各ステップで部分的に生成されたもの）
* 出力：
  * `worlds/<scene_id>/` - 最終world_bundle（world_bundle_schema.md準拠）
    * `world.yaml` - エントリーポイント
    * `metadata.json`（完全版）- 生成元mcap、処理情報、バージョン
    * その他すべてのファイル
  * `build_report.json` - ビルド処理のサマリ
* 役割：
  * すべての中間生成物を world_bundle 構造に配置
  * `world.yaml` の生成（各ファイルへの相対パスを集約）
  * `metadata.json` の最終化（処理情報、統計情報を追記）
  * 最終検証（schema 準拠チェック、ファイル存在確認、一貫性チェック）
  * `build_report.json` の生成（処理時間、統計、警告・エラー）

---

# 2) gs_ros2_simulator（world_bundleを読んで閉ループを回す）

目的：**ROS2インタフェースの契約を固定して、中身を差し替え可能に**する
中は「起動/設定」「状態推定」「運動」「センサ」「判定」「ログ」に分けるのが自然です。

## 2-1. world_loader（world_bundle読み込み）

* 入力：`world:=worlds/<scene_id>`
* 出力：メモリ上のWorldConfig（gaussians/geometry/calib/timebase）
* 役割：

  * フォーマット互換性チェック（versioning）
  * lazy load（大きいGaussianを必要時だけ読むなど）

## 2-2. sim_clock（/clockとステップ管理）

* 入力：timebase.yaml（sim_dt、レート）
* 出力：`/clock`
* 役割：センサ更新（12Hz/20Hz）とシミュ更新（例100Hz）を分離

## 2-3. ego_state_core（ego状態の正）

* 入力：vehicle dynamicsの更新結果
* 出力：

  * `/tf`（map/odom/base_link）
  * `/odom`
  * `/vehicle/state`（任意）
* 役割：他ノードが参照する唯一のego状態

## 2-4. vehicle_dynamics（運動モデル）

* 入力：`/vehicle/control_cmd`（Ackermann推奨）
* パラメータ：mass、wheelbase、制限値等（起動時）
* 出力：ego状態更新（pose/twist）
* 役割：まずkinematic bicycle、後でdynamicへ差し替え

## 2-5. ground_contact & constraints（接地・制約）

* 入力：heightmap/drivable + ego pose
* 出力：

  * 接地補正（z、roll/pitchを簡易反映するかは任意）
  * `offroad` / `collision` フラグ（最初は簡易）
* 役割：走行が成立する最低限の物理制約

## 2-6. sensor_camera_gs（カメラ生成：3DGSレンダ）

* 入力：ego pose + camera calib + background gaussians
* 出力：

  * `/camera/*/image_raw`
  * `/camera/*/camera_info`
* 役割：CAM_FRONTから開始、後でsurroundへ拡張
* 実装差し替え点：

  * gsplat（CUDA）/ Vulkan / 自前レンダラ

## 2-7. sensor_lidar（LiDAR生成：まずraycast）

* 入力：ego pose + lidar calib + geometry（heightmap/mesh） + lidar spec
* 出力：`/lidar/points`
* 役割：閉ループ成立のためのLiDARを確実に出す
* 将来差し替え点：

  * SplatAD/neurad-studio方式に置換（同じtopic契約のまま）

## 2-8. ros2_bridge（ADスタックとのI/O契約面）

* 入力：ADのcontrol cmd
* 出力：ADが必要とするtopic一式（上で生成したセンサ/TF/clock）
* 役割：topic名やframe_idを“ADが期待する形”に揃える（ここに寄せると後が楽）

## 2-9. recorder & debug（ログ・可視化）

* 出力：mcap/rosbag2
* 役割：

  * 走行結果を再学習に回せるように保存
  * デバッグ用のoverlay（軌跡、drivable境界、heightmap等）

---

# 3) gs_sim_msgs（ROS2 msg/srv/action の定義）

目的：**シミュ固有の契約を標準メッセージで賄えない部分だけ補う**
最初は少なく、必要に応じて増やすのが良いです。

## 3-1. control（制御入力）

* 可能なら標準（AckermannDriveStamped）を使い、独自msgは避ける
* 独自にするなら：

  * `VehicleControlCmd.msg`（steer, throttle, brake, gear, stamp）

## 3-2. state/telemetry（状態）

* `VehicleState.msg`（pose/twist/steer_angleなど）
* `SimulationStatus.msg`（collision/offroad/step_timeなど）

## 3-3. services（運用系）

* `ResetWorld.srv`（初期位置に戻す）
* `SetEgoPose.srv`（デバッグ用）
* `LoadWorld.srv`（world切り替え：必要なら）

> ただし“まず動かす”なら service は後回しでOK。最初はlaunch引数＋再起動で十分。

---

# 4) worlds/（データ：world_bundleの集合）

目的：**差し替え可能な「シーン単位アセット」置き場**

推奨：

* `worlds/<scene_id>/world.yaml` を入口にして、各ファイルへの相対パスを持つ

例：

* `worlds/nuscenes_scene-0103/`
* `worlds/my_mcap_run_20260214/`

---

# 5) 依存関係の“きれいな”保ち方（重要）

* `gs_ros2_simulator` は **DriveStudioを一切importしない**
* `gs_ros2_simulator` は `world.yaml` だけ読めば動く
* `gs_world_builder` は runtimeの実装詳細を知らない（“world schema”だけ共有）
* 共有するのは `world schema（YAML/JSONの仕様）` と `gs_sim_msgs` だけ

