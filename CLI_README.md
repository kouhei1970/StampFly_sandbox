# StampFly CLI Terminal 機能説明書

## 概要
StampFly CLI Terminalは、M5StampS3搭載ドローンであるStampFlyをUSBケーブル経由でリアルタイム制御・監視するためのコマンドラインインターフェースです。センサーデータの取得、PIDゲイン調整、キャリブレーション、設定保存など、包括的な機能を提供します。

## 基本操作

### CLI起動
USBケーブルでM5StampS3を接続し、シリアルモニタ（115200 baud）を開くとCLIが起動します。
```
=== StampFly CLI Terminal ===
コマンド一覧は 'help' を入力してください
矢印キー↑↓でコマンド履歴を使用できます
StampFly> 
```

### ヘルプ表示
```bash
StampFly> help
=== 利用可能なコマンド ===
help       : ヘルプを表示
imu        : IMUデータを表示 (加速度・ジャイロ)
tof        : ToFセンサーデータを表示 (距離)
voltage    : バッテリー電圧を表示
mag        : 磁気センサーデータを表示
attitude   : 姿勢角を表示 (Roll/Pitch/Yaw)
all        : 全センサーデータを表示
status     : システム状態を表示
stream     : データストリーミング制御 [start/stop] [interval_ms]
offset     : センサーオフセット制御 [get/reset/start]
mag_cal    : 磁気センサー校正 [start/stop/reset/save]
reset      : システムリセット [ahrs/sensors/all]
save       : 設定保存 [offsets/mag_cal/pid/all]
load       : 設定読み込み [offsets/mag_cal/pid/all]
pid        : PIDゲイン制御 [get/set/save/load] [controller] [param] [value]
history    : コマンド履歴を表示
```

## センサーデータ表示コマンド

### `imu` - IMUデータ表示
加速度センサーとジャイロスコープのデータを表示
```bash
StampFly> imu
=== IMU データ ===
加速度 [G]: X=0.123, Y=-0.045, Z=0.987
角速度 [rad/s]: X=0.012, Y=-0.003, Z=0.001
生データ加速度: X=0.125, Y=-0.043, Z=0.985
生データ角速度: X=0.014, Y=-0.001, Z=0.003
```

### `tof` - ToFセンサーデータ表示
Time-of-Flight距離センサーのデータを表示
```bash
StampFly> tof
=== ToF センサー ===
下向き距離: 1234 mm (生データ: 1236 mm)
前向き距離: 567 mm (生データ: 569 mm)
フィルタ済み高度: 1.234 m
推定高度: 1.235 m
高度速度: 0.012 m/s
```

### `voltage` - バッテリー電圧表示
バッテリー電圧と電源状態を表示
```bash
StampFly> voltage
=== 電圧情報 ===
バッテリー電圧: 3.85 V
```

### `mag` - 磁気センサーデータ表示
3軸磁気センサーのデータを表示
```bash
StampFly> mag
=== 磁気センサー ===
磁場 [μT]: X=12.34, Y=-5.67, Z=89.01
オフセット: X=1.23, Y=-0.45, Z=2.34
平均値: X=12.30, Y=-5.70, Z=89.00
```

### `attitude` - 姿勢角表示
ドローンの姿勢角（Roll/Pitch/Yaw）を表示
```bash
StampFly> attitude
=== 姿勢角 ===
Roll:  1.23° (0.0215 rad)
Pitch: -2.34° (-0.0408 rad)
Yaw:   45.67° (0.7969 rad)
```

### `all` - 全センサーデータ表示
全てのセンサーデータを一括表示
```bash
StampFly> all
=== 全センサーデータ ===
IMU - 加速度[G]: X=0.123, Y=-0.045, Z=0.987
IMU - 角速度[rad/s]: X=0.012, Y=-0.003, Z=0.001
姿勢角[°]: Roll=1.23, Pitch=-2.34, Yaw=45.67
ToF - 距離: 下=1234mm, 前=567mm, 高度=1.234m
電圧: 3.85V
磁場[μT]: X=12.34, Y=-5.67, Z=89.01
```

## システム状態・制御コマンド

### `status` - システム状態表示
システムの動作状態と警告を表示
```bash
StampFly> status
=== システム状態 ===
動作モード: 2
稼働時間: 123.45秒
制御周期: 0.002500秒
オフセットカウンタ: 4000
ストリーミング: 無効
```

### `stream` - データストリーミング制御
リアルタイムデータストリーミングの制御（複数の出力フォーマット対応）
```bash
# ヘルプ表示
StampFly> stream
使用法: stream [start/stop/format] [interval_ms] [format]
  start [interval_ms] [format] - ストリーミング開始
  stop                         - ストリーミング停止
  format [format_type]         - 出力フォーマット設定

フォーマット:
  default  - デフォルト形式 (STREAM: IMU[...] ATT[...] ...)
  csv      - CSV形式 (カンマ区切り)
  tsv      - TSV形式 (タブ区切り)
  teleplot - Teleplot形式 (>name:timestamp:value)

現在の状態: 無効
出力フォーマット: default

データ項目順序 (CSV/TSV):
ax,ay,az,gx,gy,gz,roll,pitch,yaw,mx,my,mz,range,voltage,altitude

# デフォルト形式でストリーミング開始（100ms間隔）
StampFly> stream start 100
ストリーミング開始 (間隔: 100ms, フォーマット: default)
停止するにはエンターキーを押してください
STREAM: IMU[ax:0.123,ay:-0.045,az:0.987,gx:0.012,gy:-0.003,gz:0.001] ATT[r:1.23,p:-2.34,y:45.67] MAG[x:12.34,y:-5.67,z:89.01] TOF[1234mm] VOLT[3.85V] ALT[1.234m]

# CSV形式でストリーミング開始
StampFly> stream start 100 csv
# CSV Header:
# ax,ay,az,gx,gy,gz,roll,pitch,yaw,mx,my,mz,range,voltage,altitude
ストリーミング開始 (間隔: 100ms, フォーマット: csv)
停止するにはエンターキーを押してください
0.123,-0.045,0.987,0.012,-0.003,0.001,1.23,-2.34,45.67,12.34,-5.67,89.01,1234,3.85,1.234

# Teleplot形式でストリーミング開始
StampFly> stream start 50 teleplot
ストリーミング開始 (間隔: 50ms, フォーマット: teleplot)
停止するにはエンターキーを押してください
>accel_x:12345:0.123
>accel_y:12345:-0.045
>accel_z:12345:0.987
>gyro_x:12345:0.012
>gyro_y:12345:-0.003
>gyro_z:12345:0.001
>roll:12345:1.23
>pitch:12345:-2.34
>yaw:12345:45.67
>mag_x:12345:12.34
>mag_y:12345:-5.67
>mag_z:12345:89.01
>range:12345:1234
>voltage:12345:3.85
>altitude:12345:1.234

# 出力フォーマットのみ変更
StampFly> stream format csv
出力フォーマットをCSVに設定しました

# エンターキーでストリーミング停止
[エンターキー押下]
ストリーミング停止
StampFly> 
```

#### 出力フォーマット詳細

**1. Default形式**
- 人間が読みやすい形式
- 各センサーデータがラベル付きで表示
- デバッグや手動確認に最適

**2. CSV形式（カンマ区切り）**
- データ解析ソフト（Excel、Python pandas等）で読み込み可能
- ヘッダー行でデータ項目を明示
- 数値データの統計解析に最適

**3. TSV形式（タブ区切り）**
- タブ区切りのテキストファイル
- 一部のソフトウェアでCSVより扱いやすい
- データベースへのインポートに適している

**4. Teleplot形式**
- リアルタイムプロット用の標準フォーマット
- タイムスタンプ付きでグラフ化に最適
- 各データ項目が個別の時系列として出力
- Teleplotツールで直接可視化可能

### `reset` - システムリセット
各種システムコンポーネントのリセット
```bash
# AHRS（姿勢推定）をリセット
StampFly> reset ahrs
AHRSをリセットしました

# センサーをリセット
StampFly> reset sensors
センサーをリセットしました

# 全システムをリセット
StampFly> reset all
全システムをリセットしました
```

## センサーキャリブレーション

### `offset` - センサーオフセット制御
IMUセンサーのオフセット値の管理

**注意**: オフセット計算には2つの方式があります：
- **起動時自動計算**: 800サンプル（高速起動用）
- **CLIコマンド**: 4000サンプル（高精度キャリブレーション用）

```bash
# 現在のオフセット値を表示
StampFly> offset get
=== センサーオフセット値 ===
ジャイロオフセット [rad/s]:
  Roll:  0.001234
  Pitch: -0.000567
  Yaw:   0.000890
加速度Zオフセット [G]: 0.987654
オフセット計算回数: 800  # 起動時自動計算の場合

# オフセット値をリセット
StampFly> offset reset
センサーオフセットをリセットしました

# 高精度オフセット計算を開始（4000サンプル）
StampFly> offset start
オフセット計算を開始します
機体を水平に静置してください...
400Hzループと同期してオフセット計算を実行します
進行状況: 0/4000
進行状況: 400/4000
進行状況: 800/4000
...
オフセット計算完了
=== センサーオフセット値 ===
ジャイロオフセット [rad/s]:
  Roll:  0.001234
  Pitch: -0.000567
  Yaw:   0.000890
加速度Zオフセット [G]: 0.987654
オフセット計算回数: 4000  # CLIコマンドの場合
```

### `mag_cal` - 磁気センサーキャリブレーション
3軸磁気センサーの高精度キャリブレーション
```bash
# キャリブレーション開始
StampFly> mag_cal start
磁気センサーキャリブレーション開始
機体をゆっくりと全方向に回転させてください
300サンプル収集後、自動的に完了します

# キャリブレーション状態確認
StampFly> mag_cal status
キャリブレーションモード: 有効
現在の磁気データ:
  生データ: X=12.34, Y=-5.67, Z=89.01 μT
  平均値:   X=12.30, Y=-5.70, Z=89.00 μT
  オフセット: X=1.23, Y=-0.45, Z=2.34 μT

# キャリブレーション停止
StampFly> mag_cal stop
磁気センサーキャリブレーション停止

# キャリブレーション保存
StampFly> mag_cal save
磁気センサーキャリブレーションを保存しました

# キャリブレーションリセット
StampFly> mag_cal reset
磁気センサーキャリブレーションをリセットしました
```

## PIDゲイン制御

### `pid get [controller]` - PIDゲイン表示
```bash
# 全PIDゲインを表示
StampFly> pid get
=== 全PIDゲイン ===
Roll Rate:   kp=0.650, ti=0.700, td=0.010, eta=0.125
Pitch Rate:  kp=0.950, ti=0.700, td=0.025, eta=0.125
Yaw Rate:    kp=3.000, ti=0.800, td=0.010, eta=0.125
Roll Angle:  kp=5.000, ti=4.000, td=0.040, eta=0.125
Pitch Angle: kp=5.000, ti=4.000, td=0.040, eta=0.125
Altitude:    kp=0.380, ti=10.000, td=0.500, eta=0.125

# 特定のコントローラーのゲインを表示
StampFly> pid get roll_rate
=== Roll Rate PID ===
kp=0.650, ti=0.700, td=0.010, eta=0.125
```

### `pid set [controller] [param] [value]` - PIDゲイン設定
```bash
# Roll Rate PIDのKpゲインを0.8に設定
StampFly> pid set roll_rate kp 0.8
roll_rate kp を 0.800 に設定しました

# Pitch Angle PIDのTiゲインを3.5に設定
StampFly> pid set pitch_angle ti 3.5
pitch_angle ti を 3.500 に設定しました

# Altitude PIDのTdゲインを0.6に設定
StampFly> pid set altitude td 0.6
altitude td を 0.600 に設定しました
```

### `pid save/load/reset` - PIDゲイン管理
```bash
# 現在のPIDゲインを保存
StampFly> pid save
PIDゲインを保存しました

# 保存されたPIDゲインを読み込み
StampFly> pid load
PIDゲインを読み込みました

# PIDゲインをデフォルト値にリセット
StampFly> pid reset
PIDゲインをデフォルト値にリセットしました
```

#### 利用可能なコントローラー
- **roll_rate**: ロール角速度制御PID
- **pitch_rate**: ピッチ角速度制御PID  
- **yaw_rate**: ヨー角速度制御PID
- **roll_angle**: ロール角度制御PID
- **pitch_angle**: ピッチ角度制御PID
- **altitude**: 高度制御PID

#### 利用可能なパラメータ
- **kp**: 比例ゲイン（Proportional Gain）
- **ti**: 積分時間（Integral Time）
- **td**: 微分時間（Derivative Time）
- **eta**: 微分フィルタ時定数（Derivative Filter Time Constant）

## 設定保存・読み込み

### `save` - 設定保存
各種設定をフラッシュメモリに永続保存（統一されたNVS API使用）
```bash
# センサーオフセット値を保存
StampFly> save offsets
[SENSOR] Saving sensor offsets to NVS...
[SENSOR] Sensor offsets saved successfully
センサーオフセット値を保存しました

# 磁気センサーキャリブレーションを保存
StampFly> save mag_cal
[MAG] Saving magnetometer calibration to NVS...
[MAG] Magnetometer calibration saved successfully
磁気センサーキャリブレーションを保存しました

# PIDゲインを保存
StampFly> save pid
[PID] Saving PID gains to NVS...
[PID] PID gains saved successfully
PIDゲインを保存しました

# 全設定を一括保存
StampFly> save all
[SENSOR] Saving sensor offsets to NVS...
[SENSOR] Sensor offsets saved successfully
[MAG] Saving magnetometer calibration to NVS...
[MAG] Magnetometer calibration saved successfully
[PID] Saving PID gains to NVS...
[PID] PID gains saved successfully
全設定を保存しました
  - センサーオフセット値
  - 磁気センサーキャリブレーション
  - PIDゲイン
```

### `load` - 設定読み込み
保存された設定をフラッシュメモリから読み込み（統一されたNVS API使用）
```bash
# センサーオフセット値を読み込み
StampFly> load offsets
[SENSOR] Loading sensor offsets from NVS...
[SENSOR] Loaded sensor offsets:
[SENSOR] Roll rate offset: 0.001234
[SENSOR] Pitch rate offset: -0.000567
[SENSOR] Yaw rate offset: 0.000890
[SENSOR] Accel Z offset: 0.987654
[SENSOR] Offset counter: 800
センサーオフセット値を読み込みました

# 磁気センサーキャリブレーションを読み込み
StampFly> load mag_cal
[MAG] Loading magnetometer calibration from NVS...
[MAG] Loaded magnetometer calibration:
[MAG] Offset: X=1.23, Y=-0.45, Z=2.34
[MAG] Scale: X=1.05, Y=0.98, Z=1.02
磁気センサーキャリブレーションを読み込みました

# PIDゲインを読み込み
StampFly> load pid
[PID] Loading PID gains from NVS...
[PID] Loaded PID gains:
[PID] Roll Rate: kp=0.650, ti=0.700, td=0.010, eta=0.125
[PID] Pitch Rate: kp=0.950, ti=0.700, td=0.025, eta=0.125
[PID] Yaw Rate: kp=3.000, ti=0.800, td=0.010, eta=0.125
[PID] Roll Angle: kp=5.000, ti=4.000, td=0.040, eta=0.125
[PID] Pitch Angle: kp=5.000, ti=4.000, td=0.040, eta=0.125
[PID] Altitude: kp=0.380, ti=10.000, td=0.500, eta=0.125
PIDゲインを読み込みました

# 全設定を一括読み込み
StampFly> load all
[SENSOR] Loading sensor offsets from NVS...
[SENSOR] Sensor offsets loaded successfully
[MAG] Loading magnetometer calibration from NVS...
[MAG] Magnetometer calibration loaded successfully
[PID] Loading PID gains from NVS...
[PID] PID gains loaded successfully
全設定を読み込みました
  - センサーオフセット値
  - 磁気センサーキャリブレーション
  - PIDゲイン
```

## コマンド履歴機能

### `history` - コマンド履歴表示
過去に実行したコマンドの履歴を表示
```bash
StampFly> history
=== コマンド履歴 ===
 1: imu
 2: tof
 3: voltage
 4: stream start 100
 5: stream stop
 6: pid get all
 7: pid set roll_rate kp 0.6
 8: offset start
 9: mag_cal start
10: status
矢印キー↑↓で履歴を呼び出せます
```

### 矢印キーによる履歴操作
- **↑キー**: 過去のコマンドを遡る（最大10個）
- **↓キー**: 新しいコマンドに進む
- **文字入力**: 履歴モードを終了し、新しいコマンドを入力
- **Backspace**: 履歴モードを終了し、編集中のコマンドに戻る

#### 履歴機能の特徴
- **履歴サイズ**: 最大10個のコマンドを保存
- **重複排除**: 同じコマンドが連続する場合は1つのみ保存
- **クロスプラットフォーム対応**: Mac、Windows、Linux等の多様なキーボード入力に対応
- **ANSI エスケープシーケンス**: 標準的なターミナルエスケープシーケンスを使用

## 実用的な使用例

### 1. 初期セットアップワークフロー
```bash
# 1. システム状態確認
StampFly> status

# 2. 全センサーデータ確認
StampFly> all

# 3. センサーオフセット計算
StampFly> offset start

# 4. 磁気センサーキャリブレーション
StampFly> mag_cal start
# （機体を全方向に回転させる）

# 5. PIDゲイン確認・調整
StampFly> pid get
StampFly> pid set roll_rate kp 0.8

# 6. 全設定を保存
StampFly> save all
```

### 2. 飛行前チェック
```bash
# バッテリー電圧確認
StampFly> voltage

# センサー動作確認
StampFly> imu
StampFly> tof
StampFly> mag

# 姿勢角確認
StampFly> attitude

# システム状態確認
StampFly> status
```

### 3. PIDチューニング
```bash
# 現在のゲイン確認
StampFly> pid get roll_rate

# ゲイン調整
StampFly> pid set roll_rate kp 0.8
StampFly> pid set roll_rate ti 0.6

# 飛行テスト実施

# 良好な結果が得られたら保存
StampFly> pid save

# 必要に応じてデフォルト値に戻す
StampFly> pid reset
```

### 4. リアルタイム監視
```bash
# 100ms間隔でデータストリーミング開始
StampFly> stream start 100

# データを監視しながら飛行

# ストリーミング停止
StampFly> stream stop
```

### 5. トラブルシューティング
```bash
# システム状態確認
StampFly> status

# センサーリセット
StampFly> reset sensors

# AHRS（姿勢推定）リセット
StampFly> reset ahrs

# 保存された設定を読み込み
StampFly> load all

# オフセット再計算
StampFly> offset start
```

## 技術仕様

### 通信仕様
- **接続方式**: USB Serial（CDC）
- **ボーレート**: 115200 bps
- **データビット**: 8
- **パリティ**: なし
- **ストップビット**: 1
- **フロー制御**: なし

### データ更新頻度
- **IMU**: 400Hz
- **ToF**: 30Hz
- **磁気センサー**: 400Hz
- **電圧**: 400Hz
- **ストリーミング**: 10ms〜10000ms（可変）

### 永続化機能（統一されたNVS API）
- **保存先**: ESP32内蔵フラッシュメモリ（NVS - Non-Volatile Storage）
- **API**: 統一された直接NVS API（`nvs_open`, `nvs_set_blob`, `nvs_get_blob`）
- **名前空間分離**: 
  - センサーオフセット: `"sensor_offset"`
  - 磁気キャリブレーション: `"mag_calib"`
  - PIDゲイン: `"pid_gains"`
- **キー名規則**: 
  - 15文字以内（ESP32制限対応）
  - 短縮形統一（例: `"roll_r_kp"`, `"pitch_a_ti"`, `"yaw_rate_off"`）
- **保存データ**: 
  - センサーオフセット（5項目）
  - 磁気キャリブレーション（6項目）
  - PIDゲイン（24項目）
- **エラーハンドリング**: 個別パラメータの保存・読み込み成功/失敗を詳細ログ出力
- **フォールバック**: 読み込み失敗時は自動的にデフォルト値を使用
- **自動読み込み**: 起動時に保存された設定を自動適用
- **データ保護**: 電源断時もデータを保持
- **競合回避**: 完全に独立した名前空間で機能間の競合を排除

### 対応プラットフォーム
- **Arduino IDE**: シリアルモニタ
- **PlatformIO**: シリアルモニタ
- **ターミナルソフト**: PuTTY、TeraTerm、screen、minicom等
- **OS**: Windows、macOS、Linux

この包括的なCLI機能により、StampFlyの開発、調整、運用が大幅に効率化されます。
