#include "mag.hpp"
#include <Arduino.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <cmath>

const char* MagSensor::TAG = "MAG_SENSOR";

MagSensor mag_sensor; // グローバルインスタンス

MagSensor::MagSensor() 
    : mag_x(0.0f), mag_y(0.0f), mag_z(0.0f),
      cal_x(0.0f), cal_y(0.0f), cal_z(0.0f),
      calibration_mode(false), calib_sample_count(0) {
    
    // キャリブレーションパラメータの初期化
    calib_params.offset_x = 0.0f;
    calib_params.offset_y = 0.0f;
    calib_params.offset_z = 0.0f;
    calib_params.scale_x = 1.0f;
    calib_params.scale_y = 1.0f;
    calib_params.scale_z = 1.0f;
}

bool MagSensor::init() {
    ESP_LOGI(TAG, "Initializing magnetometer...");
    
    // BMM150センサーの初期化
    if (bmm150.initialize() != BMM150_OK) {
        ESP_LOGE(TAG, "Failed to initialize BMM150 sensor");
        return false;
    }
    
    // 保存されたキャリブレーションパラメータの読み込み
    loadCalibration();
    
    ESP_LOGI(TAG, "Magnetometer initialized successfully");
    return true;
}

bool MagSensor::update() {
    // BMM150センサーからデータを読み取る
    bmm150.read_mag_data();
    
    // 生の地磁気データを取得
    mag_x = static_cast<float>(bmm150.mag_data.x);
    mag_y = static_cast<float>(bmm150.mag_data.y);
    mag_z = static_cast<float>(bmm150.mag_data.z);
    
    // キャリブレーションモードの場合、データを収集
    if (calibration_mode) {
        collectCalibrationData();
    }
    
    // キャリブレーションを適用
    applyCalibration();
    
    return true;
}

void MagSensor::collectCalibrationData() {
    // サンプル数が最大値に達していない場合のみデータを追加
    if (calib_sample_count < MAX_CALIB_SAMPLES) {
        calib_samples[calib_sample_count] = Vector3(mag_x, mag_y, mag_z);
        calib_sample_count++;
        
        // 10サンプルごとにログを出力
        if (calib_sample_count % 10 == 0) {
            ESP_LOGI(TAG, "Collected %d calibration samples", calib_sample_count);
        }
        
        // サンプル数が最大値に達したら自動的にキャリブレーションを実行
        if (calib_sample_count == MAX_CALIB_SAMPLES) {
            ESP_LOGI(TAG, "Maximum number of calibration samples reached");
            calibrateEllipsoid();
            setCalibrationMode(false);
        }
    }
}

void MagSensor::calibrateEllipsoid() {
    ESP_LOGI(TAG, "Starting ellipsoid fitting calibration...");
    
    if (calib_sample_count < 10) {
        ESP_LOGE(TAG, "Not enough calibration samples");
        return;
    }
    
    // 楕円体フィッティングを実行
    ellipsoidFitting();
    
    // キャリブレーションパラメータを保存
    saveCalibration();
    
    ESP_LOGI(TAG, "Calibration completed");
    ESP_LOGI(TAG, "Offset: [%.2f, %.2f, %.2f]", 
             calib_params.offset_x, calib_params.offset_y, calib_params.offset_z);
    ESP_LOGI(TAG, "Scale: [%.2f, %.2f, %.2f]", 
             calib_params.scale_x, calib_params.scale_y, calib_params.scale_z);
}

void MagSensor::ellipsoidFitting() {
    Vector3 center;
    Vector3 radii;
    Matrix3x3 evecs;
    
    // 楕円体フィッティングを実行
    bool success = EllipsoidFit::fit(calib_samples, calib_sample_count, center, radii, evecs);
    
    if (!success) {
        ESP_LOGE(TAG, "Ellipsoid fitting failed");
        return;
    }
    
    // キャリブレーションパラメータを設定
    calib_params.offset_x = center.x();
    calib_params.offset_y = center.y();
    calib_params.offset_z = center.z();
    
    // スケールファクターを計算（最大半径で正規化）
    float max_radius = fmax(fmax(radii.x(), radii.y()), radii.z());
    calib_params.scale_x = max_radius / radii.x();
    calib_params.scale_y = max_radius / radii.y();
    calib_params.scale_z = max_radius / radii.z();
}

void MagSensor::applyCalibration() {
    // ハードアイアン補正（オフセット）
    float x_centered = mag_x - calib_params.offset_x;
    float y_centered = mag_y - calib_params.offset_y;
    float z_centered = mag_z - calib_params.offset_z;
    
    // ソフトアイアン補正（スケールファクター）
    cal_x = x_centered * calib_params.scale_x;
    cal_y = y_centered * calib_params.scale_y;
    cal_z = z_centered * calib_params.scale_z;
}

void MagSensor::getCalibratedData(float &x, float &y, float &z) {
    x = cal_x;
    y = cal_y;
    z = cal_z;
}

void MagSensor::getRawData(float &x, float &y, float &z) {
    x = mag_x;
    y = mag_y;
    z = mag_z;
}

float MagSensor::getHeading(float roll, float pitch) {
    // 地磁気データを水平面に投影
    float cos_roll = cos(roll);
    float sin_roll = sin(roll);
    float cos_pitch = cos(pitch);
    float sin_pitch = sin(pitch);
    
    // 傾き補正（Tilt Compensation）
    float x_h = cal_x * cos_pitch + cal_y * sin_roll * sin_pitch + cal_z * cos_roll * sin_pitch;
    float y_h = cal_y * cos_roll - cal_z * sin_roll;
    
    // 方位角を計算（ラジアン）
    float heading = atan2(-y_h, x_h);
    
    // 正の値に変換（0〜2π）
    if (heading < 0) {
        heading += 2.0f * M_PI;
    }
    
    return heading;
}

void MagSensor::saveCalibration() {
    ESP_LOGI(TAG, "Saving calibration parameters to NVS...");
    
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    // NVSを初期化
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVSパーティションを消去して再初期化
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    
    // NVSを開く
    err = nvs_open("mag_calib", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return;
    }
    
    // キャリブレーションパラメータを保存
    err = nvs_set_blob(nvs_handle, "offset_x", &calib_params.offset_x, sizeof(float));
    if (err != ESP_OK) ESP_LOGE(TAG, "Error saving offset_x: %s", esp_err_to_name(err));
    
    err = nvs_set_blob(nvs_handle, "offset_y", &calib_params.offset_y, sizeof(float));
    if (err != ESP_OK) ESP_LOGE(TAG, "Error saving offset_y: %s", esp_err_to_name(err));
    
    err = nvs_set_blob(nvs_handle, "offset_z", &calib_params.offset_z, sizeof(float));
    if (err != ESP_OK) ESP_LOGE(TAG, "Error saving offset_z: %s", esp_err_to_name(err));
    
    err = nvs_set_blob(nvs_handle, "scale_x", &calib_params.scale_x, sizeof(float));
    if (err != ESP_OK) ESP_LOGE(TAG, "Error saving scale_x: %s", esp_err_to_name(err));
    
    err = nvs_set_blob(nvs_handle, "scale_y", &calib_params.scale_y, sizeof(float));
    if (err != ESP_OK) ESP_LOGE(TAG, "Error saving scale_y: %s", esp_err_to_name(err));
    
    err = nvs_set_blob(nvs_handle, "scale_z", &calib_params.scale_z, sizeof(float));
    if (err != ESP_OK) ESP_LOGE(TAG, "Error saving scale_z: %s", esp_err_to_name(err));
    
    // 変更をコミット
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) ESP_LOGE(TAG, "Error committing NVS: %s", esp_err_to_name(err));
    
    // NVSを閉じる
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "Calibration parameters saved successfully");
}

void MagSensor::loadCalibration() {
    ESP_LOGI(TAG, "Loading calibration parameters from NVS...");
    
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    // NVSを初期化
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVSパーティションを消去して再初期化
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    
    // NVSを開く
    err = nvs_open("mag_calib", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        ESP_LOGW(TAG, "Using default calibration parameters");
        return;
    }
    
    // キャリブレーションパラメータを読み込み
    size_t size = sizeof(float);
    
    err = nvs_get_blob(nvs_handle, "offset_x", &calib_params.offset_x, &size);
    if (err != ESP_OK) ESP_LOGW(TAG, "Error loading offset_x: %s", esp_err_to_name(err));
    
    err = nvs_get_blob(nvs_handle, "offset_y", &calib_params.offset_y, &size);
    if (err != ESP_OK) ESP_LOGW(TAG, "Error loading offset_y: %s", esp_err_to_name(err));
    
    err = nvs_get_blob(nvs_handle, "offset_z", &calib_params.offset_z, &size);
    if (err != ESP_OK) ESP_LOGW(TAG, "Error loading offset_z: %s", esp_err_to_name(err));
    
    err = nvs_get_blob(nvs_handle, "scale_x", &calib_params.scale_x, &size);
    if (err != ESP_OK) ESP_LOGW(TAG, "Error loading scale_x: %s", esp_err_to_name(err));
    
    err = nvs_get_blob(nvs_handle, "scale_y", &calib_params.scale_y, &size);
    if (err != ESP_OK) ESP_LOGW(TAG, "Error loading scale_y: %s", esp_err_to_name(err));
    
    err = nvs_get_blob(nvs_handle, "scale_z", &calib_params.scale_z, &size);
    if (err != ESP_OK) ESP_LOGW(TAG, "Error loading scale_z: %s", esp_err_to_name(err));
    
    // NVSを閉じる
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "Loaded calibration parameters:");
    ESP_LOGI(TAG, "Offset: [%.2f, %.2f, %.2f]", 
             calib_params.offset_x, calib_params.offset_y, calib_params.offset_z);
    ESP_LOGI(TAG, "Scale: [%.2f, %.2f, %.2f]", 
             calib_params.scale_x, calib_params.scale_y, calib_params.scale_z);
}

void MagSensor::resetCalibration() {
    ESP_LOGI(TAG, "Resetting calibration parameters...");
    
    // キャリブレーションパラメータをリセット
    calib_params.offset_x = 0.0f;
    calib_params.offset_y = 0.0f;
    calib_params.offset_z = 0.0f;
    calib_params.scale_x = 1.0f;
    calib_params.scale_y = 1.0f;
    calib_params.scale_z = 1.0f;
    
    // サンプルカウントをリセット
    calib_sample_count = 0;
    
    // NVSからキャリブレーションパラメータを削除
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    err = nvs_open("mag_calib", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        nvs_erase_all(nvs_handle);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }
    
    ESP_LOGI(TAG, "Calibration parameters reset successfully");
}

void MagSensor::setCalibrationMode(bool enable) {
    if (enable && !calibration_mode) {
        // キャリブレーションモードを有効化
        calibration_mode = true;
        calib_sample_count = 0;
        ESP_LOGI(TAG, "Calibration mode enabled");
    } else if (!enable && calibration_mode) {
        // キャリブレーションモードを無効化
        calibration_mode = false;
        ESP_LOGI(TAG, "Calibration mode disabled");
    }
}

bool MagSensor::isCalibrationMode() {
    return calibration_mode;
}
