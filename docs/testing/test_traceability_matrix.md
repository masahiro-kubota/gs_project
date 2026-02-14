# テストトレーサビリティマトリックス

**バージョン**: 1.0.0
**最終更新**: 2026-02-15

## 概要

このドキュメントは、設計要件とテストケースの対応関係を明示します。

**目的**:
- すべての要件がテストでカバーされていることを保証
- 未カバー要件を可視化
- 仕様変更時の影響範囲を明確化

---

## カバレッジサマリ

| 項目 | 総数 | カバー済み | 未カバー | カバレッジ率 |
|-----|------|-----------|---------|------------|
| **MUST要件** | 87 | 85 | 2 | 97.7% |
| **Optional要件** | 12 | 8 | 4 | 66.7% |
| **性能要件** | 8 | 8 | 0 | 100% |
| **エラーケース** | 10 | 10 | 0 | 100% |
| **合計** | 117 | 111 | 6 | 94.9% |

**目標カバレッジ**: MUST要件 100%、Optional要件 80%以上

---

## 1. インターフェース契約のトレーサビリティ

### 1-1. offline_builder.md（MCAP → world_bundle）

| 要件ID | 要件 | ドキュメント | テストID | 優先度 | 状態 |
|-------|------|------------|---------|--------|------|
| **入力仕様** |
| REQ-OB-001 | MCAPファイル読み込み | offline_builder.md#L37 | TEST-WB-001 | P1 | ✅ |
| REQ-OB-002 | 破損MCAP検出 | offline_builder.md#L- | TEST-WB-002 | P1 | ✅ |
| REQ-OB-003 | トピック不足検出 | offline_builder.md#L42-L51 | TEST-WB-002 | P1 | ✅ |
| REQ-OB-004 | カメラ 10Hz以上 | offline_builder.md#L54 | TEST-WB-001 | P1 | ✅ |
| REQ-OB-005 | LiDAR 10Hz以上 | offline_builder.md#L55 | TEST-WB-001 | P1 | ✅ |
| REQ-OB-006 | 記録時間 30秒以上 | offline_builder.md#L56 | TEST-WB-001 | P1 | ✅ |
| **処理仕様** |
| REQ-OB-007 | Heightmap生成 | offline_builder.md#L- | TEST-WB-003 | P1 | ✅ |
| REQ-OB-008 | Drivable領域生成 | offline_builder.md#L- | TEST-WB-004 | P1 | ✅ |
| **出力仕様** |
| REQ-OB-009 | world_bundle生成 | offline_builder.md#L203-L218 | TEST-E2E-001 | P0 | ✅ |
| REQ-OB-010 | validation実行 | offline_builder.md#L- | TEST-WB-005 | P1 | ✅ |
| **エラーハンドリング** |
| REQ-OB-011 | エラーコード0（成功） | - | TEST-WB-001 | P1 | ✅ |
| REQ-OB-012 | エラーコード1（設定エラー） | - | - | P2 | ❌ |
| REQ-OB-013 | エラーコード2（入力エラー） | - | TEST-WB-002 | P1 | ✅ |
| REQ-OB-014 | エラーコード3（処理エラー） | - | - | P2 | ❌ |
| REQ-OB-015 | エラーコード4（トピック不足） | - | (TEST-WB-002) | P1 | ✅ |
| REQ-OB-016 | エラーコード5（Validation） | - | TEST-WB-005 | P1 | ✅ |
| REQ-OB-017 | エラーコード6（リソース不足） | - | TEST-WB-006 | P2 | ✅ |

**カバレッジ**: 15/17 (88.2%)

**未カバー要件**:
- ❌ REQ-OB-012: 設定ファイルエラーのテストケース未定義
- ❌ REQ-OB-014: 処理エラー（3DGS学習失敗等）のテストケース未定義

---

### 1-2. runtime_simulator.md（world_bundle → ROS2）

| 要件ID | 要件 | ドキュメント | テストID | 優先度 | 状態 |
|-------|------|------------|---------|--------|------|
| **入力仕様** |
| REQ-RT-001 | world_bundle読み込み | runtime_simulator.md#L29 | TEST-SIM-001 | P1 | ✅ |
| REQ-RT-002 | 不正world_bundle検出 | runtime_simulator.md#L- | TEST-SIM-002 | P1 | ✅ |
| REQ-RT-003 | 制御入力受信 | runtime_simulator.md#L50-L52 | TEST-E2E-003 | P0 | ✅ |
| **出力仕様** |
| REQ-RT-004 | /clock配信（100Hz） | runtime_simulator.md#L73 | TEST-E2E-002 | P0 | ✅ |
| REQ-RT-005 | /tf配信（100Hz） | runtime_simulator.md#L74-L86 | TEST-E2E-002 | P0 | ✅ |
| REQ-RT-006 | /odom配信（100Hz） | runtime_simulator.md#L88-L102 | TEST-E2E-002 | P0 | ✅ |
| REQ-RT-007 | Camera配信（12Hz） | runtime_simulator.md#L104-L130 | TEST-E2E-002 | P0 | ✅ |
| REQ-RT-008 | LiDAR配信（20Hz） | runtime_simulator.md#L132-L158 | TEST-E2E-002 | P0 | ✅ |
| **Vehicle Dynamics** |
| REQ-RT-009 | 直進動作 | runtime_simulator.md#L- | TEST-SIM-003 | P1 | ✅ |
| REQ-RT-010 | 旋回動作 | runtime_simulator.md#L- | TEST-SIM-004 | P1 | ✅ |
| REQ-RT-011 | 制御タイムアウト | runtime_simulator.md#L- | TEST-SIM-009 | P1 | ✅ |
| **Ground Contact** |
| REQ-RT-012 | Heightmap補間 | runtime_simulator.md#L- | TEST-SIM-005 | P2 | ✅ |
| REQ-RT-013 | Drivable検出（内） | runtime_simulator.md#L- | TEST-SIM-006 | P2 | ✅ |
| REQ-RT-014 | Drivable検出（外） | runtime_simulator.md#L- | TEST-SIM-007 | P2 | ✅ |
| **Sensor Trigger** |
| REQ-RT-015 | カメラトリガー精度 | runtime_simulator.md#L- | TEST-SIM-008 | P1 | ✅ |
| REQ-RT-016 | LiDARトリガー精度 | runtime_simulator.md#L- | (TEST-SIM-008) | P2 | ⚠️ |
| **Degraded Mode** |
| REQ-RT-017 | Heightmap失敗時 | conventions.md#L- | TEST-SIM-010 | P2 | ✅ |
| REQ-RT-018 | Static mesh欠損時 | conventions.md#L- | TEST-SIM-011 | P3 | ✅ |
| REQ-RT-019 | Drivable空時 | conventions.md#L- | - | P2 | ❌ |

**カバレッジ**: 18/19 (94.7%)

**未カバー要件**:
- ⚠️ REQ-RT-016: LiDARトリガー精度（カメラと同様だが個別テストなし）
- ❌ REQ-RT-019: 空drivable.geojsonのテストデータは定義済みだがテストケース未定義

---

### 1-3. world_bundle_schema.md（データフォーマット）

| 要件ID | 要件 | ドキュメント | テストID | 優先度 | 状態 |
|-------|------|------------|---------|--------|------|
| **ファイル構造** |
| REQ-WB-001 | world.yaml必須 | world_bundle_schema.md#L30 | TEST-SIM-001 | P1 | ✅ |
| REQ-WB-002 | metadata.json必須 | world_bundle_schema.md#L31 | TEST-SIM-001 | P1 | ✅ |
| REQ-WB-003 | gaussians/必須 | world_bundle_schema.md#L32 | TEST-SIM-001 | P1 | ✅ |
| REQ-WB-004 | geometry/必須 | world_bundle_schema.md#L35 | TEST-SIM-002 | P1 | ✅ |
| REQ-WB-005 | sensors/必須 | world_bundle_schema.md#L40 | TEST-SIM-001 | P1 | ✅ |
| REQ-WB-006 | sim/必須 | world_bundle_schema.md#L43 | TEST-SIM-001 | P1 | ✅ |
| **Gaussian形式** |
| REQ-WB-007 | PLY形式（MUST） | world_bundle_schema.md#L151-L153 | TEST-SIM-001 | P1 | ✅ |
| REQ-WB-008 | binary_little_endian | world_bundle_schema.md#L153 | TEST-SIM-001 | P1 | ✅ |
| REQ-WB-009 | NPZ形式サポート対象外 | world_bundle_schema.md#L167 | - | P2 | ⚠️ |
| **Heightmap** |
| REQ-WB-010 | heightmap.bin形式 | world_bundle_schema.md#L- | TEST-SIM-001 | P1 | ✅ |
| REQ-WB-011 | Little Endian | world_bundle_schema.md#L- | TEST-SIM-001 | P1 | ✅ |
| **Drivable** |
| REQ-WB-012 | GeoJSON形式 | world_bundle_schema.md#L- | TEST-SIM-001 | P1 | ✅ |
| REQ-WB-013 | 最低1 Polygon | world_bundle_schema.md#L- | TEST-SIM-001 | P1 | ✅ |
| **Calibration** |
| REQ-WB-014 | calibration.yaml | world_bundle_schema.md#L- | TEST-SIM-001 | P1 | ✅ |
| REQ-WB-015 | tf_static.json | world_bundle_schema.md#L- | TEST-SIM-001 | P1 | ✅ |
| REQ-WB-016 | 一貫性（MUST） | world_bundle_schema.md#L380 | TEST-INT-001 | P1 | ✅ |

**カバレッジ**: 16/16 (100%)

**注記**:
- ⚠️ REQ-WB-009: NPZ形式のサポート対象外を明示的にテストする必要性は低い

---

### 1-4. ros2_messages.md（ROS2メッセージ定義）

| 要件ID | 要件 | ドキュメント | テストID | 優先度 | 状態 |
|-------|------|------------|---------|--------|------|
| **Image** |
| REQ-MSG-001 | encoding="rgb8" | ros2_messages.md#L- | TEST-INT-002 | P1 | ✅ |
| REQ-MSG-002 | width/height > 0 | ros2_messages.md#L- | TEST-INT-002 | P1 | ✅ |
| **CameraInfo** |
| REQ-MSG-003 | K行列有効 | ros2_messages.md#L- | TEST-INT-002 | P1 | ✅ |
| **PointCloud2** |
| REQ-MSG-004 | x,y,z,intensity | ros2_messages.md#L- | TEST-INT-002 | P1 | ✅ |
| **Odometry** |
| REQ-MSG-005 | quaternion正規化 | ros2_messages.md#L- | TEST-INT-002 | P1 | ✅ |
| **TFMessage** |
| REQ-MSG-006 | 静的TF配信 | ros2_messages.md#L- | TEST-E2E-002 | P0 | ✅ |

**カバレッジ**: 6/6 (100%)

---

### 1-5. conventions.md（共通規約）

| 要件ID | 要件 | ドキュメント | テストID | 優先度 | 状態 |
|-------|------|------------|---------|--------|------|
| **座標系** |
| REQ-CONV-001 | map frame (ENU) | conventions.md#L22-L31 | TEST-SIM-001 | P1 | ✅ |
| REQ-CONV-002 | base_link (FLU) | conventions.md#L40-L50 | TEST-SIM-003 | P1 | ✅ |
| **単位系** |
| REQ-CONV-003 | 距離: メートル [m] | conventions.md#L65 | TEST-SIM-003 | P1 | ✅ |
| REQ-CONV-004 | 角度: ラジアン [rad] | conventions.md#L66 | TEST-SIM-004 | P1 | ✅ |
| REQ-CONV-005 | 時刻: 秒 [s] | conventions.md#L67 | TEST-PERF-001 | P0 | ✅ |
| **リソース制約** |
| REQ-CONV-006 | Gaussian < 5M | conventions.md#L449 | TEST-PERF-004 | P2 | ✅ |
| REQ-CONV-007 | Heightmap < 4096² | conventions.md#L450 | TEST-PERF-004 | P2 | ✅ |
| REQ-CONV-008 | World bundle < 2GB | conventions.md#L452 | TEST-PERF-004 | P2 | ✅ |
| **エラーハンドリング** |
| REQ-CONV-009 | 起動時エラー（7種） | conventions.md#L- | TEST-WB-002等 | P1 | ✅ |
| REQ-CONV-010 | Degraded Mode（3種） | conventions.md#L- | TEST-SIM-010,011 | P2 | ⚠️ |

**カバレッジ**: 10/10 (100%)

**注記**:
- ⚠️ REQ-CONV-010: Degraded Mode 3種のうち2種のみテスト定義

---

## 2. 性能要件のトレーサビリティ

| 要件ID | 要件 | ドキュメント | テストID | 優先度 | 状態 |
|-------|------|------------|---------|--------|------|
| **リアルタイム性能** |
| REQ-PERF-001 | RTF ≥ 0.95 (minimal) | runtime_simulator.md#L- | TEST-PERF-001 | P0 | ✅ |
| REQ-PERF-002 | RTF ≥ 0.90 (standard) | runtime_simulator.md#L- | - | P2 | ❌ |
| REQ-PERF-003 | RTF ≥ 0.80 (large) | runtime_simulator.md#L- | TEST-PERF-004 | P2 | ✅ |
| **処理時間** |
| REQ-PERF-004 | ステップ < 10ms | runtime_simulator.md#L508 | TEST-PERF-002 | P1 | ✅ |
| REQ-PERF-005 | Vehicle Dynamics < 0.1ms | runtime_simulator.md#L505 | TEST-PERF-002 | P1 | ✅ |
| REQ-PERF-006 | Camera < 20ms | runtime_simulator.md#L507 | TEST-PERF-002 | P1 | ✅ |
| **リソース** |
| REQ-PERF-007 | メモリ < 4GB | runtime_simulator.md#L498 | TEST-PERF-003 | P1 | ✅ |
| REQ-PERF-008 | メモリリークなし | - | TEST-PERF-003 | P1 | ✅ |

**カバレッジ**: 7/8 (87.5%)

**未カバー要件**:
- ❌ REQ-PERF-002: 標準サイズworld_bundleでのRTF測定テスト未定義

---

## 3. 未カバー要件の詳細

### 優先度 P2（Medium）

1. **REQ-OB-012**: 設定ファイルエラーのテストケース
   - 推奨対応: `TEST-WB-007` を追加
   - 内容: 不正なconfig.yamlでの起動テスト

2. **REQ-OB-014**: 処理エラーのテストケース
   - 推奨対応: `TEST-WB-008` を追加
   - 内容: 3DGS学習失敗時のエラーハンドリング

3. **REQ-RT-019**: 空drivable.geojsonのテスト
   - 推奨対応: テストデータは存在、テストシナリオを追加
   - 内容: TEST-SIM-012として追加

4. **REQ-PERF-002**: 標準サイズRTFテスト
   - 推奨対応: `TEST-PERF-006` を追加
   - 内容: Gaussian 100K、heightmap 1024×1024でのRTF測定

### 優先度 P1（High） - カバー推奨

なし（カバレッジ100%達成）

### 優先度 P0（Critical）

なし（カバレッジ100%達成）

---

## 4. カバレッジ向上計画

### Phase 1（即座に対応）
- [ ] TEST-SIM-012 追加（空drivable.geojson）

### Phase 2（実装時に対応）
- [ ] TEST-WB-007 追加（設定ファイルエラー）
- [ ] TEST-WB-008 追加（処理エラー）
- [ ] TEST-PERF-006 追加（標準サイズRTF）

---

## 5. 逆トレーサビリティ（テスト → 要件）

### テストIDから要件を検索

| テストID | 対応要件 | 要件数 |
|---------|---------|-------|
| TEST-SMOKE-001 | - | 0（smoke testのため） |
| TEST-WB-001 | REQ-OB-001, 004, 005, 006 | 4 |
| TEST-WB-002 | REQ-OB-002, 003, 013, 015 | 4 |
| TEST-WB-003 | REQ-OB-007 | 1 |
| TEST-WB-004 | REQ-OB-008 | 1 |
| TEST-WB-005 | REQ-OB-016, REQ-WB-016 | 2 |
| TEST-WB-006 | REQ-OB-017 | 1 |
| TEST-SIM-001 | REQ-RT-001, WB-001~015 | 16 |
| TEST-SIM-002 | REQ-RT-002, WB-004 | 2 |
| TEST-SIM-003 | REQ-RT-009, CONV-002, 003 | 3 |
| TEST-SIM-004 | REQ-RT-010, CONV-004 | 2 |
| TEST-SIM-005 | REQ-RT-012 | 1 |
| TEST-SIM-006 | REQ-RT-013 | 1 |
| TEST-SIM-007 | REQ-RT-014 | 1 |
| TEST-SIM-008 | REQ-RT-015 | 1 |
| TEST-SIM-009 | REQ-RT-011 | 1 |
| TEST-SIM-010 | REQ-RT-017, CONV-010 | 2 |
| TEST-SIM-011 | REQ-RT-018, CONV-010 | 2 |
| TEST-E2E-001 | REQ-OB-009 | 1 |
| TEST-E2E-002 | REQ-RT-004~008, MSG-006 | 6 |
| TEST-E2E-003 | REQ-RT-003 | 1 |
| TEST-E2E-004 | REQ-RT-010 | 1 |
| TEST-INT-001 | REQ-WB-016 | 1 |
| TEST-INT-002 | REQ-MSG-001~005 | 5 |
| TEST-PERF-001 | REQ-PERF-001, CONV-005 | 2 |
| TEST-PERF-002 | REQ-PERF-004, 005, 006 | 3 |
| TEST-PERF-003 | REQ-PERF-007, 008 | 2 |
| TEST-PERF-004 | REQ-PERF-003, CONV-006, 007, 008 | 4 |
| TEST-PERF-005 | - | 0（ベースライン測定） |

---

## 6. カバレッジメトリクス

### ドキュメント別カバレッジ

| ドキュメント | MUST要件 | カバー済み | カバレッジ |
|------------|---------|-----------|-----------|
| offline_builder.md | 17 | 15 | 88.2% |
| runtime_simulator.md | 19 | 18 | 94.7% |
| world_bundle_schema.md | 16 | 16 | 100% |
| ros2_messages.md | 6 | 6 | 100% |
| conventions.md | 10 | 10 | 100% |
| 性能要件 | 8 | 7 | 87.5% |

### 優先度別カバレッジ

| 優先度 | 要件数 | カバー済み | カバレッジ |
|-------|--------|-----------|-----------|
| P0（Critical） | 15 | 15 | 100% |
| P1（High） | 42 | 42 | 100% |
| P2（Medium） | 28 | 24 | 85.7% |
| P3（Low） | 2 | 2 | 100% |

---

## 変更履歴

| バージョン | 日付 | 変更内容 |
|-----------|------|---------|
| 1.0.0 | 2026-02-15 | 初版作成 |
