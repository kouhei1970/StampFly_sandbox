/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "sensor.hpp"
#include "imu.hpp"
#include "tof.hpp"
#include "flight_control.hpp"
#include "spi_s3.hpp"
#include "i2c.hpp"
#include "wrapper.hpp"
#include "mag.hpp"
#include "opt.hpp"
#include <nvs_flash.h>
#include <nvs.h>

Madgwick Drone_ahrs;
Alt_kalman EstimatedAltitude;

INA3221 ina3221(INA3221_ADDR40_GND); // Set I2C address to 0x40 (A0 pin -> GND)
Filter acc_filter;
Filter az_filter;
Filter voltage_filter;
Filter raw_ax_filter;
Filter raw_ay_filter;
Filter raw_az_filter;
Filter raw_az_d_filter;
Filter raw_gx_filter;
Filter raw_gy_filter;
Filter raw_gz_filter;
Filter alt_filter;

// Sensor data
volatile float Roll_angle = 0.0f, Pitch_angle = 0.0f, Yaw_angle = 0.0f;
volatile float Roll_rate, Pitch_rate, Yaw_rate;
volatile float Roll_rate_offset = 0.0f, Pitch_rate_offset = 0.0f, Yaw_rate_offset = 0.0f;
volatile float Accel_z_d;
volatile float Accel_z_offset = 0.0f;
volatile float Accel_x_raw, Accel_y_raw, Accel_z_raw;
volatile float Accel_x, Accel_y, Accel_z;
volatile float Roll_rate_raw, Pitch_rate_raw, Yaw_rate_raw;
volatile float Mx, My, Mz, Mx0, My0, Mz0, Mx_ave, My_ave, Mz_ave;
volatile int16_t RawRange = 0;
volatile int16_t Range = 0;
volatile int16_t RawRangeFront = 0;
volatile int16_t RangeFront = 0;
volatile float Altitude = 0.0f;
volatile float Altitude2 = 0.0f;
volatile float Alt_velocity = 0.0f;
volatile float Az = 0.0;
volatile float Az_bias = 0.0;
int16_t deltaX, deltaY;

// オプティカルフロー関連変数
volatile float Optical_flow_x = 0.0f;
volatile float Optical_flow_y = 0.0f;
volatile float Velocity_x = 0.0f;
volatile float Velocity_y = 0.0f;

volatile uint16_t Offset_counter = 0;

// オフセット計算制御用フラグ
volatile uint8_t Offset_calc_flag = 0;
volatile uint16_t Offset_calc_counter = 0;
volatile uint16_t Offset_calc_target = 0;

volatile float Voltage;
float Acc_norm = 0.0f;
// quat_t Quat;
float Over_g = 0.0f, Over_rate = 0.0f;
uint8_t OverG_flag = 0;
uint8_t Range0flag = 0;
volatile uint8_t Under_voltage_flag = 0;
// volatile uint8_t ToF_bottom_data_ready_flag;
// volatile uint16_t Range=1000;

void sensor_reset_offset(void)
{
    Roll_rate_offset = 0.0f;
    Pitch_rate_offset = 0.0f;
    Yaw_rate_offset = 0.0f;
    Accel_z_offset = 0.0f;
    Offset_counter = 0;
}

void sensor_calc_offset_avarage(void)
{
    Roll_rate_offset = (Offset_counter * Roll_rate_offset + Roll_rate_raw) / (Offset_counter + 1);
    Pitch_rate_offset = (Offset_counter * Pitch_rate_offset + Pitch_rate_raw) / (Offset_counter + 1);
    Yaw_rate_offset = (Offset_counter * Yaw_rate_offset + Yaw_rate_raw) / (Offset_counter + 1);
    Accel_z_offset = (Offset_counter * Accel_z_offset + Accel_z_raw) / (Offset_counter + 1);
    Offset_counter++;
}

void sensor_start_offset_calc(uint16_t target_count)
{
    sensor_reset_offset();
    Offset_calc_flag = 1;
    Offset_calc_counter = 0;
    Offset_calc_target = target_count;
}

uint8_t sensor_is_offset_calc_running(void)
{
    return Offset_calc_flag;
}

void test_voltage(void)
{
    for (uint16_t i = 0; i < 1000; i++)
    {
        ESPSerial.printf("Voltage[%03d]:%f\n\r", i, ina3221.getVoltage(INA3221_CH2));
    }
}

void ahrs_reset(void)
{
    Drone_ahrs.reset();
}

void sensor_init()
{
    // beep_init();

    ESPSerial.printf("Start Sensor Initilize!\r\n");
    if (i2c_master_init() == ESP_OK)
    {
        ESPSerial.printf("I2C INIT Success!\n\r");
    }
    else
    {
        ESPSerial.printf("I2C INIT failure!\n\r");
        while (1)
            ;
    }
    if (i2c_scan() == 0)
    {
        ESPSerial.printf("No I2C device!\r\n");
        ESPSerial.printf("Can not boot StampFly.\r\n");
        while (1)
            ;
    }
    // SPI init
    if (spi_init() == ESP_OK)
    {
        ESPSerial.printf("SPI INIT Success!\n\r");
    }
    else
    {
        ESPSerial.printf("SPI INIT failure!\n\r");
        while (1)
            ;
    }

    tof_init();

    if (i2c_scan() == 0)
    {
        ESPSerial.printf("No I2C device!\r\n");
        ESPSerial.printf("Can not boot AtomFly2.\r\n");
        while (1)
            ;
    }

    imu_init();
    Drone_ahrs.begin(400.0);
    ina3221.begin(I2C_MASTER_NUM);
    ina3221.reset();
    voltage_filter.set_parameter(0.005, 0.0025);

    uint16_t cnt = 0;
    while (cnt < 10)
    {
        if (ToF_bottom_data_ready_flag)
        {
            ToF_bottom_data_ready_flag = 0;
            cnt++;
            ESPSerial.printf("%d %d\n\r", cnt, tof_bottom_get_range());
        }
    }
    delay(10);

    // Magnetometer initialization
    if (mag_sensor.init()) {
        ESPSerial.printf("Magnetometer INIT Success!\n\r");
    } else {
        ESPSerial.printf("Magnetometer INIT failure!\n\r");
        // 磁気センサーが初期化できなくても続行
    }

    // Acceleration filter
    acc_filter.set_parameter(0.005, 0.0025);

    raw_ax_filter.set_parameter(0.003, 0.0025);
    raw_ay_filter.set_parameter(0.003, 0.0025);
    raw_az_filter.set_parameter(0.003, 0.0025);

    raw_gx_filter.set_parameter(0.003, 0.0025);
    raw_gy_filter.set_parameter(0.003, 0.0025);
    raw_gz_filter.set_parameter(0.003, 0.0025);

    raw_az_d_filter.set_parameter(0.1, 0.0025); // alt158
    az_filter.set_parameter(0.1, 0.0025);       // alt158
    alt_filter.set_parameter(0.005, 0.0025);

    // PMW3901 Optical Flow Sensor initialization
    if (powerUp(&optconfig)) {
        ESPSerial.printf("PMW3901 INIT Success!\n\r");
        initRegisters();
    } else {
        ESPSerial.printf("PMW3901 INIT failure!\n\r");
        // オプティカルフローセンサーが初期化できなくても続行
    }
}

float sensor_read(void)
{
    float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
    float ax, ay, az, gx, gy, gz, acc_norm, rate_norm;
    float filterd_v;
    static float dp, dq, dr;
    static uint16_t dcnt = 0u;
    int16_t deff;
    static int16_t old_range[4] = {0};
    static float alt_time = 0.0f;
    static float sensor_time = 0.0f;
    static float old_alt_time = 0.0f;
    static uint8_t first_flag = 0;
    static uint8_t preMode = 0;
    static uint8_t outlier_counter = 0;
    const uint8_t interval = 400 / 30 + 1;
    float old_sensor_time = 0.0;
    uint32_t st;
    float sens_interval;
    float h;
    static float opt_interval = 0.0;

    st = micros();
    old_sensor_time = sensor_time;
    sensor_time = (float)st * 1.0e-6;
    sens_interval = sensor_time - old_sensor_time;
    opt_interval = opt_interval + sens_interval;

    // 以下では航空工学の座標軸の取り方に従って
    // X軸：前後（前が正）左肩上がりが回転の正
    // Y軸：右左（右が正）頭上げが回転の正
    // Z軸：下上（下が正）右回りが回転の正
    // となる様に軸の変換を施しています
    // BMI270の座標軸の撮り方は
    // X軸：右左（右が正）頭上げが回転の正
    // Y軸：前後（前が正）左肩上がりが回転の正
    // Z軸：上下（上が正）左回りが回転の正

    // Get IMU raw data
    imu_update(); // IMUの値を読む前に必ず実行
    acc_x = imu_get_acc_x();
    acc_y = imu_get_acc_y();
    acc_z = imu_get_acc_z();
    gyro_x = imu_get_gyro_x();
    gyro_y = imu_get_gyro_y();
    gyro_z = imu_get_gyro_z();

    // USBSerial.printf("%9.6f %9.6f %9.6f\n\r", Elapsed_time, sens_interval, acc_z);

    // Axis Transform
    Accel_x_raw = acc_y;
    Accel_y_raw = acc_x;
    Accel_z_raw = -acc_z;
    Roll_rate_raw = gyro_y;
    Pitch_rate_raw = gyro_x;
    Yaw_rate_raw = -gyro_z;

    if ((Mode == PARKING_MODE) && (Mode != preMode)) // モードが遷移した時Static変数を初期化する。外れ値除去のバグ対策
    {
        first_flag = 0;
        old_range[0] = 0;
        old_range[1] = 0;
        old_range[2] = 0;
        old_range[3] = 0;

        raw_ax_filter.reset();
        raw_ay_filter.reset();
        raw_az_filter.reset();
        raw_az_d_filter.reset();

        raw_gx_filter.reset();
        raw_gy_filter.reset();
        raw_gz_filter.reset();

        az_filter.reset();
        alt_filter.reset();

        acc_filter.reset();
    }

    if (Mode > AVERAGE_MODE)
    {
        Accel_x = raw_ax_filter.update(Accel_x_raw, Interval_time);
        Accel_y = raw_ay_filter.update(Accel_y_raw, Interval_time);
        Accel_z = raw_az_filter.update(Accel_z_raw, Interval_time);
        Accel_z_d = raw_az_d_filter.update(Accel_z_raw - Accel_z_offset, Interval_time);

        Roll_rate = raw_gx_filter.update(Roll_rate_raw - Roll_rate_offset, Interval_time);
        Pitch_rate = raw_gy_filter.update(Pitch_rate_raw - Pitch_rate_offset, Interval_time);
        Yaw_rate = raw_gz_filter.update(Yaw_rate_raw - Yaw_rate_offset, Interval_time);

        Drone_ahrs.updateIMU((Pitch_rate) * (float)RAD_TO_DEG, (Roll_rate) * (float)RAD_TO_DEG,
                             -(Yaw_rate) * (float)RAD_TO_DEG, Accel_y, Accel_x, -Accel_z);
        Roll_angle = Drone_ahrs.getPitch() * (float)DEG_TO_RAD;
        Pitch_angle = Drone_ahrs.getRoll() * (float)DEG_TO_RAD;
        Yaw_angle = -Drone_ahrs.getYaw() * (float)DEG_TO_RAD;

        // for debug
        // USBSerial.printf("%6.3f %7.4f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n\r",
        //   Elapsed_time, Interval_time, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);

        // Get Magnetometer data
        if (mag_sensor.update()) {
            // 一時変数を使用してvolatile変数への参照問題を回避
            float temp_mx, temp_my, temp_mz;
            float cal_x, cal_y, cal_z;
            
            // 生の磁気データを取得
            mag_sensor.getRawData(temp_mx, temp_my, temp_mz);
            Mx = temp_mx;
            My = temp_my;
            Mz = temp_mz;
            
            // キャリブレーション済みデータを取得
            mag_sensor.getCalibratedData(cal_x, cal_y, cal_z);
            
            // オフセット値を設定（キャリブレーション用）
            Mx0 = cal_x - Mx;  // オフセット = キャリブレーション済み - 生データ
            My0 = cal_y - My;
            Mz0 = cal_z - Mz;
            
            // 平均値（移動平均フィルタ）
            static float mx_sum = 0.0f, my_sum = 0.0f, mz_sum = 0.0f;
            static int mag_count = 0;
            const int MAG_AVE_SIZE = 10;
            
            mx_sum += cal_x;
            my_sum += cal_y;
            mz_sum += cal_z;
            mag_count++;
            
            if (mag_count >= MAG_AVE_SIZE) {
                Mx_ave = mx_sum / MAG_AVE_SIZE;
                My_ave = my_sum / MAG_AVE_SIZE;
                Mz_ave = mz_sum / MAG_AVE_SIZE;
                mx_sum = my_sum = mz_sum = 0.0f;
                mag_count = 0;
            }
        }

        // Get Optical Flow (PMW3901データシート準拠 + 品質チェック)
        int16_t raw_deltaX, raw_deltaY;
        uint8_t motion_status = readMotionCount(&raw_deltaX, &raw_deltaY);
        
        // motion_status: 0=新データなし, 1=有効データ, 2=品質不良で破棄
        if (motion_status == 1 && sens_interval > 0.0f) {
            // 有効なデータが利用可能な場合のみ更新
            // PMW3901データシート準拠の移動量計算（高さベースCPI使用）
            float movement_x, movement_y;
            calculateMovementFromDelta(-raw_deltaY, raw_deltaX, &movement_x, &movement_y, Altitude2); // 軸変換、高度使用
            
            // 移動量を更新
            Optical_flow_x = movement_x;
            Optical_flow_y = movement_y;
            
            // 速度計算
            Velocity_x = movement_x / sens_interval;
            Velocity_y = movement_y / sens_interval;
            
            // 旧形式の変数も更新（互換性のため）
            deltaX = -raw_deltaY; // X軸は前後方向
            deltaY = raw_deltaX;  // Y軸は左右方向
        }
        // motion_status == 0 (新データなし) または motion_status == 2 (品質不良) の場合は前回値を保持

        // Get Altitude (30Hz)
        Az = az_filter.update(-Accel_z_d, sens_interval);

        if (dcnt > interval)
        {
            if (ToF_bottom_data_ready_flag)
            {
                dcnt = 0u;
                old_alt_time = alt_time;
                alt_time = micros() * 1.0e-6;
                h = alt_time - old_alt_time;
                ToF_bottom_data_ready_flag = 0;

                // 距離の値の更新
                // old_range[0] = dist;
                RawRange = tof_bottom_get_range();
                if (Mode == PARKING_MODE)
                    RawRangeFront = tof_front_get_range();
                // USBSerial.printf("%9.6f %d\n\r", Elapsed_time, RawRange);
                if (RawRange > 20)
                {
                    Range = RawRange;
                }
                if (RawRangeFront > 0.01)
                {
                    RangeFront = RawRangeFront;
                }

                // 外れ値処理
                deff = Range - old_range[1];
                if (deff > 500 && outlier_counter < 2)
                {
                    Range = old_range[1] + (old_range[1] - old_range[3]) / 2;
                    outlier_counter++;
                }
                else if (deff < -500 && outlier_counter < 2)
                {
                    Range = old_range[1] + (old_range[1] - old_range[3]) / 2;
                    outlier_counter++;
                }
                // old_range[3] = old_range[2];
                // old_range[2] = old_range[1];
                // old_range[1] = Range;
                else
                {
                    outlier_counter = 0;
                    old_range[3] = old_range[2];
                    old_range[2] = old_range[1];
                    old_range[1] = Range;
                }

                // USBSerial.printf("%9.6f, %9.6f, %9.6f, %9.6f, %9.6f\r\n",Elapsed_time,Altitude/1000.0,  Altitude2,
                // Alt_velocity,-(Accel_z_raw - Accel_z_offset)*9.81/(-Accel_z_offset));
            }
        }
        else
            dcnt++;

        Altitude = alt_filter.update((float)Range / 1000.0, Interval_time);
        if (first_flag == 1)
            EstimatedAltitude.update(Altitude, Az, Interval_time);
        else
            first_flag = 1;
        Altitude2 = EstimatedAltitude.Altitude;
        // MAX_ALTを超えたら高度下げる（自動着陸）
        if ((Altitude2 > ALT_LIMIT && Alt_flag >= 1 && Flip_flag == 0) || RawRange == 0)
            Range0flag++;
        else
            Range0flag = 0;
        if (Range0flag > RNAGE0FLAG_MAX)
            Range0flag = RNAGE0FLAG_MAX;
        Alt_velocity = EstimatedAltitude.Velocity;
        Az_bias = EstimatedAltitude.Bias;
        // USBSerial.printf("Sens=%f Az=%f Altitude=%f Velocity=%f Bias=%f\n\r",Altitude, Az, Altitude2, Alt_velocity,
        // Az_bias);
    } // End of if Mode > Average mode

    // Accel fail safe
    acc_norm = sqrt(Accel_x * Accel_x + Accel_y * Accel_y + Accel_z_d * Accel_z_d);
    Acc_norm = acc_filter.update(acc_norm, Control_period);
    if (Acc_norm > 2.0)
    {
        OverG_flag = 1;
        if (Over_g == 0.0)
            Over_g = acc_norm;
    }

    // Battery voltage check
    Voltage = ina3221.getVoltage(INA3221_CH2);
    filterd_v = voltage_filter.update(Voltage, Control_period);

    if (Under_voltage_flag != UNDER_VOLTAGE_COUNT)
    {
        if (filterd_v < POWER_LIMIT)
            Under_voltage_flag++;
        else
            Under_voltage_flag = 0;
        if (Under_voltage_flag > UNDER_VOLTAGE_COUNT)
            Under_voltage_flag = UNDER_VOLTAGE_COUNT;
    }

    preMode = Mode; // 今のモードを記憶

    uint32_t et = micros();
    // USBSerial.printf("Sensor read %f %f %f\n\r", (mt-st)*1.0e-6, (et-mt)*1e-6, (et-st)*1.0e-6);
    return (et - st) * 1.0e-6;
}

void save_sensor_offsets(void)
{
    ESP_LOGI("SENSOR", "Saving sensor offsets to NVS...");
    
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    // NVSを初期化
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    
    // NVSを開く
    err = nvs_open("sensor_offset", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE("SENSOR", "Error opening NVS handle: %s", esp_err_to_name(err));
        return;
    }
    
    // オフセット値を保存（volatile変数のコピーを作成）
    float temp_roll_rate_offset = Roll_rate_offset;
    float temp_pitch_rate_offset = Pitch_rate_offset;
    float temp_yaw_rate_offset = Yaw_rate_offset;
    float temp_accel_z_offset = Accel_z_offset;
    uint16_t temp_offset_counter = Offset_counter;
    
    err = nvs_set_blob(nvs_handle, "roll_rate_off", &temp_roll_rate_offset, sizeof(float));
    if (err != ESP_OK) ESP_LOGE("SENSOR", "Error saving roll_rate_off: %s", esp_err_to_name(err));
    
    err = nvs_set_blob(nvs_handle, "pitch_rate_off", &temp_pitch_rate_offset, sizeof(float));
    if (err != ESP_OK) ESP_LOGE("SENSOR", "Error saving pitch_rate_off: %s", esp_err_to_name(err));
    
    err = nvs_set_blob(nvs_handle, "yaw_rate_off", &temp_yaw_rate_offset, sizeof(float));
    if (err != ESP_OK) ESP_LOGE("SENSOR", "Error saving yaw_rate_off: %s", esp_err_to_name(err));
    
    err = nvs_set_blob(nvs_handle, "accel_z_off", &temp_accel_z_offset, sizeof(float));
    if (err != ESP_OK) ESP_LOGE("SENSOR", "Error saving accel_z_off: %s", esp_err_to_name(err));
    
    err = nvs_set_blob(nvs_handle, "offset_counter", &temp_offset_counter, sizeof(uint16_t));
    if (err != ESP_OK) ESP_LOGE("SENSOR", "Error saving offset_counter: %s", esp_err_to_name(err));
    
    // 変更をコミット
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) ESP_LOGE("SENSOR", "Error committing NVS: %s", esp_err_to_name(err));
    
    // NVSを閉じる
    nvs_close(nvs_handle);
    
    ESP_LOGI("SENSOR", "Sensor offsets saved successfully");
}

void load_sensor_offsets(void)
{
    ESP_LOGI("SENSOR", "Loading sensor offsets from NVS...");
    
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    // NVSを初期化
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    
    // NVSを開く
    err = nvs_open("sensor_offset", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW("SENSOR", "Error opening NVS handle: %s", esp_err_to_name(err));
        ESP_LOGW("SENSOR", "Using default offset values");
        return;
    }
    
    // オフセット値を読み込み
    size_t size = sizeof(float);
    
    err = nvs_get_blob(nvs_handle, "roll_rate_off", (void*)&Roll_rate_offset, &size);
    if (err != ESP_OK) ESP_LOGW("SENSOR", "Error loading roll_rate_off: %s", esp_err_to_name(err));
    
    err = nvs_get_blob(nvs_handle, "pitch_rate_off", (void*)&Pitch_rate_offset, &size);
    if (err != ESP_OK) ESP_LOGW("SENSOR", "Error loading pitch_rate_off: %s", esp_err_to_name(err));
    
    err = nvs_get_blob(nvs_handle, "yaw_rate_off", (void*)&Yaw_rate_offset, &size);
    if (err != ESP_OK) ESP_LOGW("SENSOR", "Error loading yaw_rate_off: %s", esp_err_to_name(err));
    
    err = nvs_get_blob(nvs_handle, "accel_z_off", (void*)&Accel_z_offset, &size);
    if (err != ESP_OK) ESP_LOGW("SENSOR", "Error loading accel_z_off: %s", esp_err_to_name(err));
    
    size = sizeof(uint16_t);
    err = nvs_get_blob(nvs_handle, "offset_counter", (void*)&Offset_counter, &size);
    if (err != ESP_OK) ESP_LOGW("SENSOR", "Error loading offset_counter: %s", esp_err_to_name(err));
    
    // NVSを閉じる
    nvs_close(nvs_handle);
    
    ESP_LOGI("SENSOR", "Loaded sensor offsets:");
    ESP_LOGI("SENSOR", "Roll rate offset: %.6f", Roll_rate_offset);
    ESP_LOGI("SENSOR", "Pitch rate offset: %.6f", Pitch_rate_offset);
    ESP_LOGI("SENSOR", "Yaw rate offset: %.6f", Yaw_rate_offset);
    ESP_LOGI("SENSOR", "Accel Z offset: %.6f", Accel_z_offset);
    ESP_LOGI("SENSOR", "Offset counter: %d", Offset_counter);
}
