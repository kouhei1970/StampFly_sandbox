# オリジナル シングルタスク版 バックアップ

このディレクトリには、マルチタスク化前のオリジナルコードのバックアップが保存されています。

## バックアップファイル

- `flight_control.cpp` - メイン制御ループ（400Hz割り込み版）
- `sensor.cpp` - センサー読み取り処理
- `main.cpp` - Arduino setup/loop関数

## 復元方法

マルチタスク版から元のシングルタスク版に戻したい場合：

```bash
# バックアップから復元
cp src_backup/original_single_task/flight_control.cpp src/
cp src_backup/original_single_task/sensor.cpp src/
cp src_backup/original_single_task/main.cpp src/

# マルチタスク関連ファイルを削除
rm src/multitask_*.cpp
rm src/multitask_*.hpp
```

## 主な違い

### オリジナル版（シングルタスク）
- 400Hz割り込みタイマーで`loop_400Hz()`を実行
- 全処理が1つのループ内で順次実行
- グローバル変数でデータ共有

### マルチタスク版
- FreeRTOSタスクで処理分離
- 制御・センサー・通信を独立タスクで実行
- ミューテックス保護による安全なデータ共有

## 削除されたコード

マルチタスク化により以下のコードが削除されました：

### flight_control.cpp
- `hw_timer_t *timer` - 割り込みタイマー
- `void IRAM_ATTR onTimer()` - 割り込み関数
- `loop_400Hz()` - メインループ関数
- 割り込み設定コード

### 理由
- FreeRTOSタスクで制御周期を管理するため
- 割り込みベースからタスクベースへの移行

## 作成日時
2025年6月16日 - マルチタスク化実装時

## 注意事項
- バックアップファイルは`src_backup`ディレクトリに移動されています
- これはコンパイル時の重複定義エラーを防ぐためです
- 復元時は上記の手順に従ってください
