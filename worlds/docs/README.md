# worlds/

**データディレクトリ**: world_bundle の保存場所

## 概要

`worlds/` ディレクトリには、`gs_world_builder` で生成された world_bundle が格納されます。

各 world_bundle は自己完結型のシーンデータで、以下を含みます：
- 3D Gaussian Splatting データ（背景）
- 幾何データ（heightmap, drivable領域, メッシュ）
- センサキャリブレーション
- シミュレーション設定

---

## ディレクトリ構造

```
worlds/
├── <scene_id_1>/           # World bundle（例: nuscenes_scene_001）
│   ├── world.yaml          # World bundle のエントリーポイント
│   ├── metadata.json       # シーン情報
│   ├── gaussians/          # 3DGS データ
│   ├── geometry/           # 幾何データ
│   ├── sensors/            # センサ設定
│   └── sim/                # シミュレーション設定
├── <scene_id_2>/
├── <scene_id_3>/
└── docs/
    └── README.md           # このファイル
```

---

## World Bundle フォーマット

**完全な仕様**: 📄 **[../../docs/interfaces/world_bundle_schema.md](../../docs/interfaces/world_bundle_schema.md)**

### 必須ファイル

| ファイル | 説明 |
|---------|------|
| `world.yaml` | World bundle のエントリーポイント（全ファイルへの参照） |
| `metadata.json` | シーン情報（座標系、範囲、統計情報等） |
| `gaussians/background.splat.ply` | 3DGS データ（背景） |
| `gaussians/render_config.json` | レンダリング設定 |
| `geometry/heightmap.bin` | 地面高さマップ（2.5D） |
| `geometry/heightmap.yaml` | Heightmap メタ情報 |
| `geometry/drivable.geojson` | 走行可能領域 |
| `sensors/calibration.yaml` | センサキャリブレーション |
| `sensors/tf_static.json` | 静的TF（base_link → センサ） |
| `sim/timebase.yaml` | シミュレーション設定 |

### 任意ファイル

| ファイル | 説明 |
|---------|------|
| `geometry/static_mesh.glb` | 精密3Dメッシュ（高精度LiDAR用） |

---

## World Bundle の生成

World bundle は `gs_world_builder` で MCAP ファイルから生成します：

```bash
# 基本的な使用方法
gs-world-builder build \
    --input input.mcap \
    --output worlds/my_scene \
    --config config.yaml \
    --gs-config gs_training_config.yaml \
    --geometry-config geometry_config.yaml
```

詳細は **[../../docs/interfaces/offline_builder.md](../../docs/interfaces/offline_builder.md)** を参照してください。

---

## World Bundle の使用

World bundle は `gs_ros2_simulator` で読み込んでシミュレーションを実行します：

```bash
# シミュレータ起動
ros2 launch gs_ros2_simulator bringup.launch.xml \
    world:=worlds/my_scene
```

詳細は **[../../docs/interfaces/runtime_simulator.md](../../docs/interfaces/runtime_simulator.md)** を参照してください。

---

## World Bundle の検証

World bundle が正しく生成されたか検証：

```bash
# gs_world_builder の検証コマンド
gs-world-builder validate worlds/my_scene
```

検証内容：
- 必須ファイルの存在確認
- world.yaml のスキーマ準拠
- calibration.yaml と tf_static.json の一貫性
- heightmap サイズ整合性
- drivable.geojson の妥当性
- Gaussian データの読み込み可能性

---

## 例：最小構成の world_bundle

開発・テスト用の最小構成例：

```
worlds/minimal_test/
├── world.yaml              # 必要最小限の参照
├── metadata.json           # 基本情報のみ
├── gaussians/
│   ├── background.splat.ply    # 小規模（100x100m程度）
│   └── render_config.json      # デフォルト設定
├── geometry/
│   ├── heightmap.bin           # 平坦な地面
│   ├── heightmap.yaml
│   └── drivable.geojson        # 単純な矩形領域
├── sensors/
│   ├── calibration.yaml        # 1カメラ + 1 LiDAR
│   └── tf_static.json
└── sim/
    └── timebase.yaml           # dt=0.01, 100Hz
```

---

## データサイズの目安

| シーンサイズ | Gaussian数 | Heightmap | 合計サイズ |
|------------|-----------|-----------|-----------|
| 小（100m x 100m） | ~100K | 1000x1000 | ~100 MB |
| 中（500m x 500m） | ~500K | 5000x5000 | ~500 MB |
| 大（1km x 1km） | ~2M | 10000x10000 | ~2 GB |

---

## 変更履歴

| バージョン | 日付 | 変更内容 |
|-----------|------|---------|
| 1.0.0 | 2026-02-15 | 初版作成 |
