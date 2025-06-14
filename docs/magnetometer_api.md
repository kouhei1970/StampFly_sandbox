# 地磁気センサーAPI使用方法

## 概要

このドキュメントでは、ESP32-S3用に実装された地磁気センサー（BMM150）のAPIの使用方法について説明します。このAPIは楕円体フィッティングによる高精度なキャリブレーション機能を提供し、姿勢推定システムで使用できる補正済みの地磁気データを提供します。

## NVS（Non-Volatile Storage）とは

**NVS（Non-Volatile Storage）**は、ESP32/ESP32-S3で提供される不揮発性ストレージシステムです。

### NVSの特徴
- **不揮発性**: 電源が切れてもデータが保持される
- **キー・バリュー形式**: 文字列キーと値のペアでデータを保存
- **型安全**: int、float、string、blobなど様々なデータ型をサポート
- **名前空間**: 異なるアプリケーション間でのデータ分離
- **耐久性**: フラッシュメモリの書き込み回数制限を考慮した設計

### 本APIでのNVS使用目的
地磁気センサーのキャリブレーションパラメータ（オフセット値とスケールファクター）を永続保存するために使用しています。これにより：

1. **電源再投入後も設定保持**: デバイスを再起動してもキャリブレーション結果が保持される
2. **再キャリブレーション不要**: 一度キャリブレーションを行えば、環境が変わらない限り再実行不要
3. **工場出荷時設定**: 製品出荷前にキャリブレーションを行い、エンドユーザーは即座に使用可能

### NVSに保存されるデータ
```cpp
// 保存されるキャリブレーションパラメータ
struct MagCalibParams {
    float offset_x, offset_y, offset_z;  // ハードアイアン補正値
    float scale_x, scale_y, scale_z;     // ソフトアイアン補正値
};
```

### NVSの代替手段
- **EEPROM**: より単純だが容量が小さい
- **SPIFFS/LittleFS**: ファイルシステムだが、小さなデータには過剰
- **外部EEPROM**: 追加のハードウェアが必要

## 主要機能

- **楕円体フィッティングキャリブレーション**: ハードアイアン・ソフトアイアン補正
- **自動データ収集**: 最大300サンプルの地磁気データを自動収集
- **リアルタイム補正**: キャリブレーションパラメータを使用したリアルタイム補正
- **永続保存**: NVSを使用したキャリブレーションパラメータの保存
- **傾き補正**: ロール・ピッチ角を考慮した方位角計算

## ファイル構成

```
src/
├── mag.hpp          # 地磁気センサークラスの定義
├── mag.cpp          # 地磁気センサークラスの実装
├── matrix.hpp       # 行列計算ライブラリの定義
└── matrix.cpp       # 行列計算ライブラリの実装

lib/bmm150/
├── bmm150.h         # BMM150センサードライバー（ESP-IDF対応版）
├── bmm150.cpp       # BMM150センサードライバーの実装
└── bmm150_defs.h    # BMM150センサーの定義
```

## API リファレンス

### MagSensorクラス

#### 初期化

```cpp
#include "mag.hpp"

// グローバルインスタンスを使用
extern MagSensor mag_sensor;

// 初期化
bool success = mag_sensor.init();
if (!success) {
    ESP_LOGE("MAIN", "Failed to initialize magnetometer");
}
```

#### 基本的な使用方法

```cpp
void loop() {
    // 地磁気データを更新
    mag_sensor.update();
    
    // 補正済みデータを取得
    float mag_x, mag_y, mag_z;
    mag_sensor.getCalibratedData(mag_x, mag_y, mag_z);
    
    // 生データを取得（デバッグ用）
    float raw_x, raw_y, raw_z;
    mag_sensor.getRawData(raw_x, raw_y, raw_z);
    
    // 方位角を計算（ロール・ピッチ角が必要）
    float roll = 0.0f;   // ラジアン
    float pitch = 0.0f;  // ラジアン
    float heading = mag_sensor.getHeading(roll, pitch);
    
    ESP_LOGI("MAG", "Calibrated: [%.2f, %.2f, %.2f], Heading: %.2f°", 
             mag_x, mag_y, mag_z, heading * 180.0f / M_PI);
}
```

### キャリブレーション

#### 自動キャリブレーション

```cpp
void startCalibration() {
    // キャリブレーションモードを開始
    mag_sensor.setCalibrationMode(true);
    
    ESP_LOGI("CALIB", "Calibration started. Move the device in a figure-8 pattern.");
    
    // デバイスを8の字に動かしながらデータを収集
    while (mag_sensor.isCalibrationMode()) {
        mag_sensor.update();
        vTaskDelay(pdMS_TO_TICKS(10));
        
        // 進捗を表示（オプション）
        static int last_count = 0;
        if (mag_sensor.calib_sample_count != last_count) {
            ESP_LOGI("CALIB", "Samples collected: %d/300", mag_sensor.calib_sample_count);
            last_count = mag_sensor.calib_sample_count;
        }
    }
    
    ESP_LOGI("CALIB", "Calibration completed automatically");
}
```

#### 手動キャリブレーション

```cpp
void manualCalibration() {
    // キャリブレーションモードを開始
    mag_sensor.setCalibrationMode(true);
    
    // 十分なサンプルを収集
    for (int i = 0; i < 200; i++) {
        mag_sensor.update();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // 手動でキャリブレーションを実行
    mag_sensor.calibrateEllipsoid();
    
    // キャリブレーションモードを終了
    mag_sensor.setCalibrationMode(false);
}
```

### キャリブレーションパラメータの管理

```cpp
// キャリブレーションパラメータを保存
mag_sensor.saveCalibration();

// キャリブレーションパラメータを読み込み
mag_sensor.loadCalibration();

// キャリブレーションパラメータをリセット
mag_sensor.resetCalibration();
```

## 使用例

### 基本的な地磁気センサーの使用

```cpp
#include "mag.hpp"
#include <esp_log.h>

void app_main() {
    // 地磁気センサーを初期化
    if (!mag_sensor.init()) {
        ESP_LOGE("MAIN", "Failed to initialize magnetometer");
        return;
    }
    
    while (1) {
        // データを更新
        mag_sensor.update();
        
        // 補正済みデータを取得
        float x, y, z;
        mag_sensor.getCalibratedData(x, y, z);
        
        // 方位角を計算（傾きなしの場合）
        float heading = mag_sensor.getHeading(0.0f, 0.0f);
        
        ESP_LOGI("MAG", "Mag: [%.2f, %.2f, %.2f] µT, Heading: %.1f°", 
                 x, y, z, heading * 180.0f / M_PI);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

### IMUと組み合わせた使用

```cpp
#include "mag.hpp"
#include "imu.hpp"  // 仮想的なIMUライブラリ

void app_main() {
    // センサーを初期化
    mag_sensor.init();
    imu_sensor.init();
    
    while (1) {
        // センサーデータを更新
        mag_sensor.update();
        imu_sensor.update();
        
        // IMUからロール・ピッチ角を取得
        float roll, pitch, yaw;
        imu_sensor.getEulerAngles(roll, pitch, yaw);
        
        // 地磁気データを取得
        float mag_x, mag_y, mag_z;
        mag_sensor.getCalibratedData(mag_x, mag_y, mag_z);
        
        // 傾き補正付き方位角を計算
        float magnetic_heading = mag_sensor.getHeading(roll, pitch);
        
        ESP_LOGI("AHRS", "Roll: %.1f°, Pitch: %.1f°, Mag Heading: %.1f°", 
                 roll * 180.0f / M_PI, 
                 pitch * 180.0f / M_PI, 
                 magnetic_heading * 180.0f / M_PI);
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
```

### キャリブレーション付きの完全な例

```cpp
#include "mag.hpp"
#include "button.hpp"  // 仮想的なボタンライブラリ

void app_main() {
    // 地磁気センサーを初期化
    if (!mag_sensor.init()) {
        ESP_LOGE("MAIN", "Failed to initialize magnetometer");
        return;
    }
    
    ESP_LOGI("MAIN", "Press button to start calibration");
    
    while (1) {
        // ボタンが押されたらキャリブレーション開始
        if (button_pressed()) {
            ESP_LOGI("CALIB", "Starting calibration...");
            ESP_LOGI("CALIB", "Move the device in a figure-8 pattern for 30 seconds");
            
            mag_sensor.setCalibrationMode(true);
            
            // キャリブレーション中
            while (mag_sensor.isCalibrationMode()) {
                mag_sensor.update();
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            
            ESP_LOGI("CALIB", "Calibration completed!");
        }
        
        // 通常の動作
        mag_sensor.update();
        
        float x, y, z;
        mag_sensor.getCalibratedData(x, y, z);
        float heading = mag_sensor.getHeading(0.0f, 0.0f);
        
        ESP_LOGI("MAG", "Heading: %.1f°", heading * 180.0f / M_PI);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

## キャリブレーション手順

### 1. 準備
- デバイスを金属物から離れた場所に置く
- 周囲に強い磁場源がないことを確認

### 2. キャリブレーション実行
1. `mag_sensor.setCalibrationMode(true)` を呼び出す
2. デバイスを3次元空間で8の字を描くように動かす
3. 全ての軸で最大・最小値を記録するため、様々な向きに回転させる
4. 300サンプル収集されると自動的にキャリブレーションが実行される

### 3. 確認
- キャリブレーション後、デバイスを水平に保ち、ゆっくり回転させる
- 方位角が滑らかに変化することを確認

## トラブルシューティング

### よくある問題

#### 1. センサーの初期化に失敗する
```
E (1234) MAG_SENSOR: Failed to initialize BMM150 sensor
```
**解決方法:**
- I2C接続を確認
- 電源電圧を確認（3.3V）
- プルアップ抵抗を確認

#### 2. キャリブレーションが不正確
```
E (5678) ELLIPSOID_FIT: Ellipsoid fitting failed
```
**解決方法:**
- より多くのサンプルを収集
- デバイスをより大きく動かす
- 金属物から離れた場所でキャリブレーション

#### 3. 方位角が不安定
**解決方法:**
- キャリブレーションを再実行
- 周囲の磁場環境を確認
- IMUのロール・ピッチ角の精度を確認

## パフォーマンス

- **更新レート**: 最大100Hz（BMM150の制限による）
- **精度**: キャリブレーション後±2°
- **メモリ使用量**: 約4KB（キャリブレーションサンプル含む）
- **CPU使用率**: 1%未満（100Hz更新時）

## 注意事項

1. **磁場環境**: 強い磁場源（モーター、スピーカー、磁石など）の近くでは使用しない
2. **キャリブレーション**: 使用環境が変わった場合は再キャリブレーションを推奨
3. **温度**: 極端な温度変化がある場合は再キャリブレーションが必要
4. **取り付け**: センサーの取り付け向きを変更した場合は再キャリブレーション必須

## 関連ファイル

- `src/mag.hpp` - MagSensorクラスの定義
- `src/mag.cpp` - MagSensorクラスの実装
- `src/matrix.hpp` - 行列計算ライブラリ
- `src/matrix.cpp` - 行列計算ライブラリの実装
- `lib/bmm150/` - BMM150ドライバー（ESP-IDF対応版）

## 参考資料

- [BMM150 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/magnetometers/bmm150/)
- [楕円体フィッティングアルゴリズム](https://en.wikipedia.org/wiki/Ellipsoid)
- [地磁気キャリブレーション理論](https://www.vectornav.com/resources/inertial-navigation-primer/specifications--and--error-budgets/specs-imucal)
