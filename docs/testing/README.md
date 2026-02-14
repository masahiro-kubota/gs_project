# テスティングドキュメント

**バージョン**: 1.0.1
**最終更新**: 2026-02-15

## 概要

このディレクトリには、GS-ROS2 Simulator のテスト実装に必要なすべての仕様が含まれています。

**更新内容（v1.0.1）**:
- ✅ Smoke Test追加（30秒で基本動作確認）
- ✅ エラーコード5,6のテスト追加
- ✅ Degraded Modeテスト追加
- ✅ 境界値テスト追加（最大・最小構成）
- ✅ すべてのテストに優先度を追加（P0-P3）
- ✅ 並列実行ガイドライン追加
- ✅ トレーサビリティマトリックス追加（要件⇄テスト対応表）
- ✅ test_config.py追加（共通設定の集約）

---

## ドキュメント一覧

### 1. [test_config.py](../../tests/test_config.py)
**テスト共通設定**（NEW）

- カメラ・LiDAR・Vehicle Dynamicsの設定値
- 許容誤差の定義
- テストデータパス
- ヘルパー関数（位置・角度・時刻の比較）

**用途**: テスト実装時にimportして使用

```python
from test_config import *

# 設定値を使用
assert msg.width == CAMERA_MINIMAL_WIDTH
assert_position_close(actual, expected)
```

---

### 2. [test_data_spec.md](test_data_spec.md)
**テストデータ仕様**

- minimal_test world_bundle の完全な仕様
- テスト用MCAP生成方法
- エラーケース用テストデータ（7種類）
- データ生成スクリプト（Python/Bash）

**用途**: テストデータの準備・生成

---

### 3. [test_scenarios.md](test_scenarios.md)
**テストシナリオ**（v1.0.1に更新）

- **Smoke Test**: 30秒で基本動作確認（CI用）
- **単体テスト**: 13シナリオ（エラーコード5,6、Degraded Mode含む）
- **統合テスト**: 6シナリオ（E2E、インターフェース統合）
- **性能テスト**: 5シナリオ（RTF、メモリ、境界値）
- **並列実行ガイドライン**: UUID使用、pytest -n auto

**用途**: テストケースの設計・実装

**すべてのテストに優先度を付与**:
- P0（Critical）: 必須機能
- P1（High）: 重要機能
- P2（Medium）: 通常機能
- P3（Low）: エッジケース

---

### 4. [validation_criteria.md](validation_criteria.md)
**検証基準**

- ROS2メッセージの検証基準（Image, PointCloud2, Odometry等）
- 許容誤差の定義（位置: ±0.01m、ヨー角: ±0.01rad等）
- パフォーマンス測定基準（RTF ≥ 0.95、メモリ < 4GB等）
- 成功/失敗の判定基準

**用途**: テストの合格判定・検証

---

### 5. [test_traceability_matrix.md](test_traceability_matrix.md)
**トレーサビリティマトリックス**（NEW）

- 要件⇄テストの対応表
- カバレッジサマリ（MUST要件: 97.7%）
- 未カバー要件の明示（6件）
- ドキュメント別カバレッジ

**用途**: 要件網羅性の確認、仕様変更時の影響範囲特定

**カバレッジ現状**:
- MUST要件: 85/87 (97.7%)
- Optional要件: 8/12 (66.7%)
- 性能要件: 8/8 (100%)

---

## テスト実装の流れ

### ステップ1: テストデータ準備

```bash
# すべてのテストデータを生成
bash scripts/generate_test_data.sh
```

**生成されるデータ**:
- `worlds/minimal_test/` - 最小限のworld_bundle
- `test_data/minimal_test.mcap` - テスト用MCAP
- `test_data/*` - エラーケース用データ

詳細: [test_data_spec.md](test_data_spec.md)

---

### ステップ2: テスト実装

#### 単体テスト

```python
# gs_world_builder/tests/test_mcap_ingest.py

def test_mcap_read_valid():
    """TEST-WB-001: MCAP読み込み（正常系）"""
    # test_scenarios.md の TEST-WB-001 を参照
    ...

def test_mcap_read_corrupted():
    """TEST-WB-002: MCAP読み込み（破損ファイル）"""
    # test_scenarios.md の TEST-WB-002 を参照
    ...
```

詳細: [test_scenarios.md#1-単体テスト](test_scenarios.md#1-単体テストunit-test)

---

#### 統合テスト

```python
# tests/integration/test_e2e.py

def test_mcap_to_world_bundle():
    """TEST-E2E-001: MCAP → world_bundle 生成"""
    # test_scenarios.md の TEST-E2E-001 を参照
    ...
```

詳細: [test_scenarios.md#2-統合テスト](test_scenarios.md#2-統合テストintegration-test)

---

#### 性能テスト

```python
# tests/performance/test_rtf.py

def test_rtf_minimal():
    """TEST-PERF-001: Real-time Factor測定"""
    # test_scenarios.md の TEST-PERF-001 を参照
    rtf = measure_rtf(60)
    assert rtf >= 0.95  # validation_criteria.md 参照
```

詳細: [test_scenarios.md#3-性能テスト](test_scenarios.md#3-性能テストperformance-test)

---

### ステップ3: 検証基準の適用

```python
from validation import validate_image, validate_odometry

def test_camera_output():
    """カメラ出力の検証"""
    msg = get_camera_image()

    # validation_criteria.md の検証基準を適用
    validate_image(msg)
```

詳細: [validation_criteria.md](validation_criteria.md)

---

## テスト実行

### ローカル実行

```bash
# すべての単体テスト
colcon test --packages-select gs_world_builder gs_ros2_simulator

# 統合テスト
pytest tests/integration/

# 性能テスト
pytest tests/performance/ --benchmark-only

# すべてのテスト
bash scripts/run_all_tests.sh
```

---

### CI/CD実行

GitHub Actionsで自動実行：

```yaml
# .github/workflows/test.yml
- name: Run tests
  run: |
    bash scripts/generate_test_data.sh
    colcon test
    pytest tests/
```

---

## テストカバレッジの確認

### 優先度別チェックリスト

#### **P0（Critical）** - 必須（リリースブロッカー）

**Smoke Test**:
- [ ] TEST-SMOKE-001: world_bundle最小限起動テスト（< 30秒）

**統合テスト**:
- [ ] TEST-E2E-001: MCAP → world_bundle
- [ ] TEST-E2E-002: world_bundle → ROS2
- [ ] TEST-E2E-003: ダミー制御での走行

**性能テスト**:
- [ ] TEST-PERF-001: Real-time Factor ≥ 0.95

---

#### **P1（High）** - 重要機能

**gs_world_builder**:
- [ ] TEST-WB-001: MCAP読み込み（正常系）
- [ ] TEST-WB-002: MCAP読み込み（破損ファイル）
- [ ] TEST-WB-003: Heightmap生成
- [ ] TEST-WB-004: Drivable領域生成
- [ ] TEST-WB-005: Validationエラー（エラーコード5）

**gs_ros2_simulator**:
- [ ] TEST-SIM-001: world_bundle読み込み（正常系）
- [ ] TEST-SIM-002: world_bundle読み込み（不正ファイル）
- [ ] TEST-SIM-003: Vehicle Dynamics（直進）
- [ ] TEST-SIM-004: Vehicle Dynamics（左旋回）
- [ ] TEST-SIM-008: Sensor Trigger（12Hz）
- [ ] TEST-SIM-009: 制御入力タイムアウト

**統合テスト**:
- [ ] TEST-E2E-004: 旋回動作
- [ ] TEST-INT-001: calibration ⇄ tf_static整合性
- [ ] TEST-INT-002: ROS2メッセージ型検証

**性能テスト**:
- [ ] TEST-PERF-002: ステップ処理時間
- [ ] TEST-PERF-003: メモリ使用量
- [ ] TEST-PERF-005: 最小world_bundle性能

---

#### **P2（Medium）** - 通常機能

**gs_world_builder**:
- [ ] TEST-WB-006: リソース不足エラー（エラーコード6）

**gs_ros2_simulator**:
- [ ] TEST-SIM-005: Ground Contact（補間）
- [ ] TEST-SIM-006: Drivable検出（正常）
- [ ] TEST-SIM-007: Drivable検出（offroad）
- [ ] TEST-SIM-010: Degraded Mode（heightmap失敗）

**性能テスト**:
- [ ] TEST-PERF-004: 大規模world_bundle性能

---

#### **P3（Low）** - エッジケース

**gs_ros2_simulator**:
- [ ] TEST-SIM-011: Degraded Mode（static_mesh欠損）

---

### カバレッジ目標

| 優先度 | 目標 | 現状 |
|-------|------|------|
| P0（Critical） | 100% | テスト定義済み |
| P1（High） | 100% | テスト定義済み |
| P2（Medium） | 80%以上 | テスト定義済み |
| P3（Low） | 50%以上 | テスト定義済み |

**要件カバレッジ**: 詳細は [test_traceability_matrix.md](test_traceability_matrix.md) 参照

---

## トラブルシューティング

### テストデータが生成されない

```bash
# 依存関係を確認
pip install -r requirements.txt
sudo apt-get install ros-humble-desktop

# 手動生成
python3 scripts/generate_minimal_world.py
python3 scripts/generate_minimal_mcap.py
```

---

### テストが失敗する

1. **検証基準を確認**: [validation_criteria.md](validation_criteria.md)
2. **許容誤差を確認**: 期待値との差が許容範囲内か
3. **環境を確認**: GPU、ROS2バージョン等

---

### 性能テストが合格しない

1. **システム要件を確認**: RTX 3060以上、4GB RAM以上
2. **他のプロセスを停止**: GPU/CPU使用率を下げる
3. **world_bundleサイズを確認**: minimal_testから開始

---

## 関連ドキュメント

- **[../conventions.md](../conventions.md)** - 共通規約
- **[../interfaces/](../interfaces/)** - インターフェース契約
- **[../architecture.md](../architecture.md)** - アーキテクチャ設計

---

## 変更履歴

| バージョン | 日付 | 変更内容 |
|-----------|------|---------|
| 1.0.1 | 2026-02-15 | test_config.py追加、test_traceability_matrix.md追加、test_scenarios.md v1.0.1（Smoke Test、エラーコード5,6、Degraded Mode、境界値テスト、優先度、並列実行ガイドライン追加）、カバレッジチェックリスト優先度別に再編 |
| 1.0.0 | 2026-02-15 | 初版作成 |
