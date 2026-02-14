# インターフェース契約

**バージョン**: 1.0.0
**最終更新**: 2026-02-14

このディレクトリには、GS-ROS2 Simulator の**コンポーネント間インターフェース契約**が含まれています。

---

## 📋 契約一覧

| ファイル | 契約内容 | 関連コンポーネント |
|---------|---------|------------------|
| **[ros2_messages.md](ros2_messages.md)** | ROS2メッセージ・サービス定義 | `gs_sim_msgs` (定義) <br> `gs_ros2_simulator` (使用) |
| **[world_bundle_schema.md](world_bundle_schema.md)** | world_bundle データフォーマット | `gs_world_builder` (生成) <br> `gs_ros2_simulator` (読込) |
| **[offline_builder.md](offline_builder.md)** | MCAP → world_bundle 変換契約 | `gs_world_builder` (実装) |
| **[runtime_simulator.md](runtime_simulator.md)** | world_bundle → ROS2 シミュレーション契約 | `gs_ros2_simulator` (実装) |

---

## 🔗 コンポーネント間の契約関係

```
                    ┌─────────────────┐
                    │  ros2_messages  │
                    │  (メッセージ定義)│
                    └────────┬────────┘
                             │
                             │ 使用
                             ↓
┌─────┐  offline_builder  ┌──────────────────┐  runtime_simulator  ┌────────┐
│MCAP │ ───────────────→ │ world_bundle     │ ─────────────────→ │ ROS2   │
└─────┘                   │ (データ)         │                     │トピック│
                          └──────────────────┘                     └────────┘
                          world_bundle_schema
                          (データフォーマット)
```

**データフロー**:
1. `MCAP` → **offline_builder 契約** → `world_bundle`
2. `world_bundle` → **runtime_simulator 契約** → `ROS2 トピック`
3. ROS2 トピックは **ros2_messages 契約** で定義されたメッセージを使用

---

## 📐 契約の種類

### 1. データフォーマット契約

**[world_bundle_schema.md](world_bundle_schema.md)**
- **内容**: world_bundle のファイル構造とデータ形式
- **生成者**: `gs_world_builder`
- **消費者**: `gs_ros2_simulator`
- **中立性**: どちらのコンポーネントにも属さない、共有データ形式

---

### 2. 処理契約（入出力）

#### [offline_builder.md](offline_builder.md)
- **内容**: MCAP から world_bundle への変換処理の契約
- **入力**: MCAP ファイル、設定ファイル
- **出力**: world_bundle
- **実装**: `gs_world_builder`

#### [runtime_simulator.md](runtime_simulator.md)
- **内容**: world_bundle を使った ROS2 シミュレーションの契約
- **入力**: world_bundle、ROS2 制御トピック
- **出力**: ROS2 センサトピック（Camera, LiDAR, TF, Odom）
- **実装**: `gs_ros2_simulator`

---

### 3. メッセージ定義契約

**[ros2_messages.md](ros2_messages.md)**
- **内容**: シミュレータ固有の ROS2 メッセージ・サービス定義
- **定義**: `gs_sim_msgs` パッケージ
- **使用**: `gs_ros2_simulator`

---

## 🎯 契約変更時のルール

### 契約を変更する場合

1. **影響範囲の確認**
   - どのコンポーネントが影響を受けるか確認
   - 上記の「関連コンポーネント」列を参照

2. **バージョニング**
   - 各契約ファイルの `version` フィールドを更新
   - Semantic Versioning に従う（MAJOR.MINOR.PATCH）

3. **レビュー必須**
   - 契約変更は複数コンポーネントに影響するため、レビュー必須
   - 両側のコンポーネント実装者の承認が必要

4. **移行期間**
   - 非互換な変更の場合、旧バージョンのサポート期間を設ける
   - 両方のバージョンを一時的にサポート

---

## 📖 契約の読み方

### コンポーネント開発者向け

**gs_world_builder を開発する場合**:
1. ✅ **[offline_builder.md](offline_builder.md)** - 実装すべき入出力契約
2. ✅ **[world_bundle_schema.md](world_bundle_schema.md)** - 生成する world_bundle の形式
3. 📄 内部設計: `gs_world_builder/docs/architecture.md`

**gs_ros2_simulator を開発する場合**:
1. ✅ **[runtime_simulator.md](runtime_simulator.md)** - 実装すべき ROS2 契約
2. ✅ **[world_bundle_schema.md](world_bundle_schema.md)** - 読み込む world_bundle の形式
3. ✅ **[ros2_messages.md](ros2_messages.md)** - 使用する ROS2 メッセージ定義
4. 📄 内部設計: `gs_ros2_simulator/docs/architecture.md`

**gs_sim_msgs を開発する場合**:
1. ✅ **[ros2_messages.md](ros2_messages.md)** - 定義すべきメッセージ仕様

---

## 🔍 契約の検証

各契約には検証方法が記載されています：

- **world_bundle_schema**: `gs-world-builder validate` コマンド
- **offline_builder**: 入出力テスト
- **runtime_simulator**: ROS2 統合テスト
- **ros2_messages**: メッセージ定義のコンパイルチェック

---

## 📚 関連ドキュメント

- **[../conventions.md](../conventions.md)** - 全契約に共通する規約（座標系、単位系等）
- **[../architecture.md](../architecture.md)** - 全体アーキテクチャと設計議論

---

## 変更履歴

| バージョン | 日付 | 変更内容 |
|-----------|------|---------|
| 1.0.0 | 2026-02-14 | 初版作成、ファイル名を契約内容ベースに変更 |
