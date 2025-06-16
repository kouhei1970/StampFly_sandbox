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

#include "multitask_functions.hpp"
#include "multitask_debug.hpp"
#include "wrapper.hpp"
#include "rc.hpp"
#include "led.hpp"
#include "telemetry.hpp"
#include "cli.hpp"
#include "imu.hpp"
#include "tof.hpp"
#include "mag.hpp"
#include "opt.hpp"
#include "sensor.hpp"
#include "sensor_multitask.hpp"
#include <INA3221.h>

// 既存の制御関数の宣言
extern void control_init(void);
extern void get_command(void);
extern void angle_control(void);
extern void rate_control(void);
extern uint8_t judge_mode_change(void);
extern uint8_t auto_landing(void);
extern void flip(void);
extern void motor_stop(void);

// 外部変数（既存のコードとの互換性のため）
extern volatile uint8_t Mode;
extern volatile float Elapsed_time;
extern volatile float Interval_time;
extern volatile uint8_t Under_voltage_flag;
extern volatile uint8_t ToF_bottom_data_ready_flag;
extern INA3221 ina3221;


// 制御タイマータスク（400Hz周期生成）
void control_timer_task(void* parameter)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS);
    
    ESPSerial.printf("Control timer task started\r\n");
    
    while (true) {
        // 制御タスクに通知を送信
        if (control_task_handle != NULL) {
            xTaskNotifyGive(control_task_handle);
        }
        
        // 次の周期まで待機
        vTaskDelayUntil(&last_wake_time, period_ticks);
    }
}

// 制御タスク（400Hz）
void control_task(void* parameter)
{
    ESPSerial.printf("Control task started\r\n");
    
    control_task_init();
    
    while (true) {
        // タイマータスクからの通知を待機
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // 緊急停止チェック
        if (get_emergency_stop()) {
            motor_duty_t duty = {0};
            set_motor_duty(&duty);
            continue;
        }
        
        // 制御処理実行
        control_task_process();
    }
}

// 高速センサータスク（400Hz）
void high_speed_sensor_task(void* parameter)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(HIGH_SPEED_SENSOR_PERIOD_MS);
    
    ESPSerial.printf("High speed sensor task started\r\n");
    
    high_speed_sensor_init();
    
    while (true) {
        // IMUデータ読み取りと処理
        high_speed_sensor_process();
        
        // 次の周期まで待機
        vTaskDelayUntil(&last_wake_time, period_ticks);
    }
}

// 低速センサータスク（50Hz）
void low_speed_sensor_task(void* parameter)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(LOW_SPEED_SENSOR_PERIOD_MS);
    
    ESPSerial.printf("Low speed sensor task started\r\n");
    
    low_speed_sensor_init();
    
    while (true) {
        // 低速センサーデータ読み取りと処理
        low_speed_sensor_process();
        
        // 次の周期まで待機
        vTaskDelayUntil(&last_wake_time, period_ticks);
    }
}

// 通信・CLIタスク（10Hz）
void communication_task(void* parameter)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(COMMUNICATION_TASK_PERIOD_MS);
    
    ESPSerial.printf("Communication task started\r\n");
    
    communication_task_init();
    
    while (true) {
        // 通信処理
        communication_task_process();
        
        // 次の周期まで待機
        vTaskDelayUntil(&last_wake_time, period_ticks);
    }
}

// タスク作成関数
esp_err_t create_all_tasks(void)
{
    BaseType_t result;
    
    // 制御タイマータスク作成
    result = xTaskCreate(
        control_timer_task,
        "ControlTimer",
        2048,
        NULL,
        CONTROL_TASK_PRIORITY + 1, // 制御タスクより高い優先度
        &control_timer_task_handle
    );
    if (result != pdPASS) {
        ESPSerial.printf("Failed to create control timer task\r\n");
        return ESP_FAIL;
    }
    
    // 制御タスク作成
    result = xTaskCreate(
        control_task,
        "Control",
        CONTROL_TASK_STACK_SIZE,
        NULL,
        CONTROL_TASK_PRIORITY,
        &control_task_handle
    );
    if (result != pdPASS) {
        ESPSerial.printf("Failed to create control task\r\n");
        return ESP_FAIL;
    }
    
    // 高速センサータスク作成
    result = xTaskCreate(
        high_speed_sensor_task,
        "HighSpeedSensor",
        HIGH_SPEED_SENSOR_STACK_SIZE,
        NULL,
        HIGH_SPEED_SENSOR_PRIORITY,
        &high_speed_sensor_task_handle
    );
    if (result != pdPASS) {
        ESPSerial.printf("Failed to create high speed sensor task\r\n");
        return ESP_FAIL;
    }
    
    // 低速センサータスク作成
    result = xTaskCreate(
        low_speed_sensor_task,
        "LowSpeedSensor",
        LOW_SPEED_SENSOR_STACK_SIZE,
        NULL,
        LOW_SPEED_SENSOR_PRIORITY,
        &low_speed_sensor_task_handle
    );
    if (result != pdPASS) {
        ESPSerial.printf("Failed to create low speed sensor task\r\n");
        return ESP_FAIL;
    }
    
    // 通信タスク作成
    result = xTaskCreate(
        communication_task,
        "Communication",
        COMMUNICATION_TASK_STACK_SIZE,
        NULL,
        COMMUNICATION_TASK_PRIORITY,
        &communication_task_handle
    );
    if (result != pdPASS) {
        ESPSerial.printf("Failed to create communication task\r\n");
        return ESP_FAIL;
    }
    
    ESPSerial.printf("All tasks created successfully\r\n");
    return ESP_OK;
}

// タスク削除関数
void delete_all_tasks(void)
{
    if (control_timer_task_handle != NULL) {
        vTaskDelete(control_timer_task_handle);
        control_timer_task_handle = NULL;
    }
    
    if (control_task_handle != NULL) {
        vTaskDelete(control_task_handle);
        control_task_handle = NULL;
    }
    
    if (high_speed_sensor_task_handle != NULL) {
        vTaskDelete(high_speed_sensor_task_handle);
        high_speed_sensor_task_handle = NULL;
    }
    
    if (low_speed_sensor_task_handle != NULL) {
        vTaskDelete(low_speed_sensor_task_handle);
        low_speed_sensor_task_handle = NULL;
    }
    
    if (communication_task_handle != NULL) {
        vTaskDelete(communication_task_handle);
        communication_task_handle = NULL;
    }
}

// 制御タスク初期化
void control_task_init(void)
{
    // 既存の制御初期化関数を呼び出し
    control_init();
}

// 制御タスクメイン処理
void control_task_process(void)
{
    MULTITASK_DEBUG_TASK_START("Control");
    
    static uint32_t last_time_us = 0;
    uint32_t current_time_us = get_timestamp_us();
    
    // 制御周期計算
    if (last_time_us != 0) {
        Interval_time = (current_time_us - last_time_us) * 1e-6f;
    }
    last_time_us = current_time_us;
    
    // 経過時間更新（既存コードとの互換性のため）
    Elapsed_time = current_time_us * 1e-6f;
    
    // フライトモード判定・更新（重要：既存のjudge_mode_change()を使用）
    judge_mode_change();
    
    // 自動着陸判定
    if (Mode == AUTO_LANDING_MODE) {
        auto_landing();
    }
    
    // モード別処理
    switch (Mode) {
        case FLIGHT_MODE:
        case AUTO_LANDING_MODE:
            control_get_command();
            control_angle_control();
            control_rate_control();
            control_motor_output();
            break;
            
        case FLIP_MODE:
            flip(); // フリップ処理
            control_motor_output();
            break;
            
        case PARKING_MODE:
        case AVERAGE_MODE:
        default:
            // モーター停止
            motor_duty_t duty = {0};
            set_motor_duty(&duty);
            break;
    }
    
    MULTITASK_DEBUG_TASK_END("Control");
}

// 制御コマンド取得（既存のget_command()を参考）
void control_get_command(void)
{
    // 既存のget_command()関数を呼び出し
    get_command();
    
    // 制御コマンドを共有データに設定
    control_command_t cmd;
    cmd.thrust_command = Thrust_command;
    cmd.roll_rate_command = Roll_rate_command;
    cmd.pitch_rate_command = Pitch_rate_command;
    cmd.yaw_rate_command = Yaw_rate_command;
    cmd.roll_angle_command = Roll_angle_command;
    cmd.pitch_angle_command = Pitch_angle_command;
    cmd.yaw_angle_command = Yaw_angle_command;
    cmd.timestamp_us = get_timestamp_us();
    
    set_control_command(&cmd);
}

// 角度制御（既存のangle_control()を参考）
void control_angle_control(void)
{
    // 既存のangle_control()関数を呼び出し
    angle_control();
}

// レート制御（既存のrate_control()を参考）
void control_rate_control(void)
{
    // 既存のrate_control()関数を呼び出し
    rate_control();
    
    // モーターデューティを共有データに設定
    motor_duty_t duty;
    duty.front_right_duty = FrontRight_motor_duty;
    duty.front_left_duty = FrontLeft_motor_duty;
    duty.rear_right_duty = RearRight_motor_duty;
    duty.rear_left_duty = RearLeft_motor_duty;
    duty.timestamp_us = get_timestamp_us();
    
    set_motor_duty(&duty);
}

// モーター出力
void control_motor_output(void)
{
    motor_duty_t duty;
    if (get_motor_duty(&duty)) {
        set_duty_fr(duty.front_right_duty);
        set_duty_fl(duty.front_left_duty);
        set_duty_rr(duty.rear_right_duty);
        set_duty_rl(duty.rear_left_duty);
    }
}

// 高速センサー初期化
void high_speed_sensor_init(void)
{
    // IMU初期化は既にsensor_init()で完了
}

// 高速センサー処理
void high_speed_sensor_process(void)
{
    MULTITASK_DEBUG_TASK_START("HighSpeedSensor");
    
    imu_data_t imu_data;
    static uint8_t preMode = 0;
    
    // モード変更時のフィルタリセット
    if ((Mode == PARKING_MODE) && (Mode != preMode)) {
        sensor_reset_filters();
    }
    preMode = Mode;
    
    // 高速センサー読み取り（IMUのみ、姿勢推定含む）
    float process_time = sensor_read_high_speed();
    
    // データ構造体に格納
    imu_data.roll_angle = Roll_angle;
    imu_data.pitch_angle = Pitch_angle;
    imu_data.yaw_angle = Yaw_angle;
    imu_data.roll_rate = Roll_rate;
    imu_data.pitch_rate = Pitch_rate;
    imu_data.yaw_rate = Yaw_rate;
    imu_data.accel_x = Accel_x;
    imu_data.accel_y = Accel_y;
    imu_data.accel_z = Accel_z;
    imu_data.accel_z_d = Accel_z_d;
    imu_data.timestamp_us = get_timestamp_us();
    imu_data.data_valid = true;
    
    // 共有データに設定
    set_imu_data(&imu_data);
    
    // 処理時間監視（デバッグ用）
    if (process_time > 0.002f) { // 2ms以上の場合警告
        ESPSerial.printf("WARNING: High-speed sensor process time: %.3f ms\r\n", process_time * 1000.0f);
    }
    
    MULTITASK_DEBUG_TASK_END("HighSpeedSensor");
}

// 低速センサー初期化
void low_speed_sensor_init(void)
{
    // 低速センサー初期化は既にsensor_init()で完了
}

// 低速センサー処理
void low_speed_sensor_process(void)
{
    MULTITASK_DEBUG_TASK_START("LowSpeedSensor");
    
    low_speed_sensor_data_t sensor_data = {0};
    
    // 低速センサー読み取り（ToF、磁気センサー、オプティカルフロー等）
    float process_time = sensor_read_low_speed();
    
    // データ構造体に格納
    sensor_data.raw_range = RawRange;
    sensor_data.range = Range;
    sensor_data.raw_range_front = RawRangeFront;
    sensor_data.range_front = RangeFront;
    
    // 高度データ
    sensor_data.altitude = Altitude;
    sensor_data.altitude2 = Altitude2;
    sensor_data.alt_velocity = Alt_velocity;
    
    // 磁気センサー読み取り
    sensor_data.mx = Mx;
    sensor_data.my = My;
    sensor_data.mz = Mz;
    sensor_data.mx_ave = Mx_ave;
    sensor_data.my_ave = My_ave;
    sensor_data.mz_ave = Mz_ave;
    
    // オプティカルフロー
    sensor_data.optical_flow_x = Optical_flow_x;
    sensor_data.optical_flow_y = Optical_flow_y;
    sensor_data.velocity_x = Velocity_x;
    sensor_data.velocity_y = Velocity_y;
    
    // 電圧とフラグ
    sensor_data.voltage = Voltage;
    sensor_data.over_g_flag = OverG_flag;
    sensor_data.range0_flag = Range0flag;
    sensor_data.under_voltage_flag = Under_voltage_flag;
    
    sensor_data.timestamp_us = get_timestamp_us();
    sensor_data.data_valid = true;
    
    // 共有データに設定
    set_low_speed_sensor_data(&sensor_data);
    
    // 処理時間監視（デバッグ用）
    if (process_time > 0.015f) { // 15ms以上の場合警告
        ESPSerial.printf("WARNING: Low-speed sensor process time: %.3f ms\r\n", process_time * 1000.0f);
    }
    
    MULTITASK_DEBUG_TASK_END("LowSpeedSensor");
}

// 通信タスク初期化
void communication_task_init(void)
{
    // 通信初期化
}

// 通信タスク処理
void communication_task_process(void)
{
    // LED制御
    led_drive();
    
    // テレメトリ送信
    telemetry();
    
    // CLI処理（PARKING_MODEでのみ）
    if (get_system_mode() == PARKING_MODE) {
        cli_process();
    }
}

// ユーティリティ関数
uint32_t get_timestamp_us(void)
{
    return micros();
}

void task_delay_until_next_period(TickType_t* last_wake_time, TickType_t period_ticks)
{
    vTaskDelayUntil(last_wake_time, period_ticks);
}
