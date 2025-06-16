/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * マルチタスク対応センサー処理
 * 高速センサー（IMU）と低速センサーを分離
 */

#include "sensor.hpp"
#include "imu.hpp"
#include "tof.hpp"
#include "flight_control.hpp"
#include "wrapper.hpp"
#include "mag.hpp"
#include "opt.hpp"
#include <INA3221.h>

// 外部変数の参照
extern Madgwick Drone_ahrs;
extern Alt_kalman EstimatedAltitude;
extern INA3221 ina3221;
extern Filter raw_ax_filter, raw_ay_filter, raw_az_filter, raw_az_d_filter;
extern Filter raw_gx_filter, raw_gy_filter, raw_gz_filter;
extern Filter az_filter, alt_filter, acc_filter, voltage_filter;

// センサーデータ変数
extern volatile float Roll_angle, Pitch_angle, Yaw_angle;
extern volatile float Roll_rate, Pitch_rate, Yaw_rate;
extern volatile float Roll_rate_offset, Pitch_rate_offset, Yaw_rate_offset;
extern volatile float Accel_z_d, Accel_z_offset;
extern volatile float Accel_x_raw, Accel_y_raw, Accel_z_raw;
extern volatile float Accel_x, Accel_y, Accel_z;
extern volatile float Roll_rate_raw, Pitch_rate_raw, Yaw_rate_raw;
extern volatile float Mx, My, Mz, Mx0, My0, Mz0, Mx_ave, My_ave, Mz_ave;
extern volatile int16_t RawRange, Range, RawRangeFront, RangeFront;
extern volatile float Altitude, Altitude2, Alt_velocity, Az, Az_bias;
extern volatile float Optical_flow_x, Optical_flow_y, Velocity_x, Velocity_y;
extern volatile uint16_t Offset_counter;
extern volatile uint8_t Offset_calc_flag;
extern volatile uint16_t Offset_calc_counter, Offset_calc_target;
extern volatile float Voltage;
extern float Acc_norm, Over_g;
extern uint8_t OverG_flag, Range0flag;
extern volatile uint8_t Under_voltage_flag;
extern volatile uint8_t ToF_bottom_data_ready_flag;
extern volatile optical_test_t Optical_test;

// システム変数
extern volatile uint8_t Mode;
extern volatile float Interval_time;
extern uint8_t Alt_flag, Flip_flag;

// 関数
extern void sensor_calc_offset_avarage(void);
extern uint8_t readMotionCount(int16_t* deltaX, int16_t* deltaY);
extern void calculateMovementFromDelta(int16_t deltaX, int16_t deltaY, float* movement_x, float* movement_y, float altitude);

// 磁気センサー（mag.hppで定義されている）
#include "mag.hpp"

// 高速センサー読み取り（400Hz）- IMUのみ
float sensor_read_high_speed(void)
{
    uint32_t start_time = micros();
    
    // IMU生データ取得
    imu_update();
    float acc_x = imu_get_acc_x();
    float acc_y = imu_get_acc_y();
    float acc_z = imu_get_acc_z();
    float gyro_x = imu_get_gyro_x();
    float gyro_y = imu_get_gyro_y();
    float gyro_z = imu_get_gyro_z();

    // 軸変換（航空工学座標系へ）
    Accel_x_raw = acc_y;
    Accel_y_raw = acc_x;
    Accel_z_raw = -acc_z;
    Roll_rate_raw = gyro_y;
    Pitch_rate_raw = gyro_x;
    Yaw_rate_raw = -gyro_z;

    // フィルタ処理とオフセット補正（制御に必要な場合のみ）
    if (Mode > AVERAGE_MODE) {
        // 加速度フィルタリング
        Accel_x = raw_ax_filter.update(Accel_x_raw, Interval_time);
        Accel_y = raw_ay_filter.update(Accel_y_raw, Interval_time);
        Accel_z = raw_az_filter.update(Accel_z_raw, Interval_time);
        Accel_z_d = raw_az_d_filter.update(Accel_z_raw - Accel_z_offset, Interval_time);

        // 角速度フィルタリング
        Roll_rate = raw_gx_filter.update(Roll_rate_raw - Roll_rate_offset, Interval_time);
        Pitch_rate = raw_gy_filter.update(Pitch_rate_raw - Pitch_rate_offset, Interval_time);
        Yaw_rate = raw_gz_filter.update(Yaw_rate_raw - Yaw_rate_offset, Interval_time);

        // 姿勢推定（Madgwick AHRS）- 最重要処理
        Drone_ahrs.updateIMU((Pitch_rate) * (float)RAD_TO_DEG, (Roll_rate) * (float)RAD_TO_DEG,
                             -(Yaw_rate) * (float)RAD_TO_DEG, Accel_y, Accel_x, -Accel_z);
        
        // 姿勢角取得
        Roll_angle = Drone_ahrs.getPitch() * (float)DEG_TO_RAD;
        Pitch_angle = Drone_ahrs.getRoll() * (float)DEG_TO_RAD;
        Yaw_angle = -Drone_ahrs.getYaw() * (float)DEG_TO_RAD;

        // 高度制御用の垂直加速度フィルタリング
        Az = az_filter.update(-Accel_z_d, Interval_time);
    }

    // オフセット計算処理（キャリブレーション中）
    if (Offset_calc_flag && Offset_calc_counter < Offset_calc_target) {
        sensor_calc_offset_avarage();
        Offset_calc_counter++;
        if (Offset_calc_counter >= Offset_calc_target) {
            Offset_calc_flag = 0;
        }
    }

    uint32_t end_time = micros();
    return (end_time - start_time) * 1.0e-6f;
}

// 低速センサー読み取り（50Hz以下）- ToF、磁気センサー、オプティカルフロー等
float sensor_read_low_speed(void)
{
    uint32_t start_time = micros();
    static uint16_t dcnt = 0u;
    static int16_t old_range[4] = {0};
    static uint8_t outlier_counter = 0;
    static float alt_time = 0.0f;
    static float old_alt_time = 0.0f;
    static uint8_t first_flag = 0;
    const uint8_t interval = 400 / 30 + 1; // 30Hz相当
    
    // ToFセンサー処理（30Hz）
    if (dcnt > interval) {
        if (ToF_bottom_data_ready_flag) {
            dcnt = 0u;
            old_alt_time = alt_time;
            alt_time = micros() * 1.0e-6;
            ToF_bottom_data_ready_flag = 0;

            // 距離データ取得
            RawRange = tof_bottom_get_range();
            if (Mode == PARKING_MODE) {
                RawRangeFront = tof_front_get_range();
            }
            
            // 有効範囲チェック
            if (RawRange > 20) {
                Range = RawRange;
            }
            if (RawRangeFront > 0.01) {
                RangeFront = RawRangeFront;
            }

            // 外れ値除去処理
            int16_t deff = Range - old_range[1];
            if (deff > 500 && outlier_counter < 2) {
                Range = old_range[1] + (old_range[1] - old_range[3]) / 2;
                outlier_counter++;
            } else if (deff < -500 && outlier_counter < 2) {
                Range = old_range[1] + (old_range[1] - old_range[3]) / 2;
                outlier_counter++;
            } else {
                outlier_counter = 0;
                old_range[3] = old_range[2];
                old_range[2] = old_range[1];
                old_range[1] = Range;
            }

            // 高度推定（カルマンフィルタ）
            #if 1
            Altitude = alt_filter.update((float)Range / 1000.0, Interval_time);
            if (first_flag == 1) {
                EstimatedAltitude.update(Altitude, Az, Interval_time);
            } else {
                first_flag = 1;
            }
            Altitude2 = EstimatedAltitude.Altitude;
            Alt_velocity = EstimatedAltitude.Velocity;
            Az_bias = EstimatedAltitude.Bias;
            
            // 高度制限チェック
            if ((Altitude2 > ALT_LIMIT && Alt_flag >= 1 && Flip_flag == 0) || RawRange == 0) {
                Range0flag++;
            } else {
                Range0flag = 0;
            }
            if (Range0flag > RNAGE0FLAG_MAX) {
                Range0flag = RNAGE0FLAG_MAX;
            }
            #endif
        }
    } else {
        dcnt++;
    }

    // 磁気センサー処理
    if (mag_sensor.update()) {
        float temp_mx, temp_my, temp_mz;
        float cal_x, cal_y, cal_z;
        
        // 生の磁気データを取得
        mag_sensor.getRawData(temp_mx, temp_my, temp_mz);
        Mx = temp_mx;
        My = temp_my;
        Mz = temp_mz;
        
        // キャリブレーション済みデータを取得
        mag_sensor.getCalibratedData(cal_x, cal_y, cal_z);
        
        // オフセット値を設定
        Mx0 = cal_x - Mx;
        My0 = cal_y - My;
        Mz0 = cal_z - Mz;
        
        // 移動平均フィルタ
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

    // オプティカルフローセンサー処理
    int16_t raw_deltaX, raw_deltaY;
    uint8_t motion_status = readMotionCount(&raw_deltaX, &raw_deltaY);
    Optical_test.total_reads++;
    
    if (motion_status == 1 && Interval_time > 0.0f) {
        // 有効なデータが利用可能
        Optical_test.valid_reads++;

        float movement_x, movement_y;
        calculateMovementFromDelta(-raw_deltaY, raw_deltaX, &movement_x, &movement_y, Altitude2);
        
        // 移動量と速度を更新
        Optical_flow_x = movement_x;
        Optical_flow_y = movement_y;
        Velocity_x = movement_x / Interval_time;
        Velocity_y = movement_y / Interval_time;
        
        // 統計更新
        Optical_test.total_movement_x += movement_x;
        Optical_test.total_movement_y += movement_y;
        if (abs(Velocity_x) > abs(Optical_test.max_velocity_x)) {
            Optical_test.max_velocity_x = Velocity_x;
        }
        if (abs(Velocity_y) > abs(Optical_test.max_velocity_y)) {
            Optical_test.max_velocity_y = Velocity_y;
        }
    } else if (motion_status == 2) {
        Optical_test.quality_failed++;
    } else {
        Optical_test.failed_reads++;
    }

    // 電圧監視
    Voltage = ina3221.getVoltage(INA3221_CH2);
    float filtered_v = voltage_filter.update(Voltage, Interval_time);

    if (Under_voltage_flag != UNDER_VOLTAGE_COUNT) {
        if (filtered_v < POWER_LIMIT) {
            Under_voltage_flag++;
        } else {
            Under_voltage_flag = 0;
        }
        if (Under_voltage_flag > UNDER_VOLTAGE_COUNT) {
            Under_voltage_flag = UNDER_VOLTAGE_COUNT;
        }
    }

    // 加速度ノルム監視（安全機能）
    float acc_norm = sqrt(Accel_x * Accel_x + Accel_y * Accel_y + Accel_z_d * Accel_z_d);
    Acc_norm = acc_filter.update(acc_norm, Interval_time);
    if (Acc_norm > 2.0) {
        OverG_flag = 1;
        if (Over_g == 0.0) {
            Over_g = acc_norm;
        }
    }

    uint32_t end_time = micros();
    return (end_time - start_time) * 1.0e-6f;
}

// フィルタリセット関数（モード変更時）
void sensor_reset_filters(void)
{
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
