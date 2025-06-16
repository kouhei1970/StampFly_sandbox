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

#ifndef MULTITASK_DATA_HPP
#define MULTITASK_DATA_HPP

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <stdint.h>

// タスク優先度定義
#define CONTROL_TASK_PRIORITY       (configMAX_PRIORITIES - 1)  // 最高優先度
#define HIGH_SPEED_SENSOR_PRIORITY  (configMAX_PRIORITIES - 2)  // 高速センサー
#define LOW_SPEED_SENSOR_PRIORITY   (configMAX_PRIORITIES - 4)  // 低速センサー
#define COMMUNICATION_TASK_PRIORITY (configMAX_PRIORITIES - 6)  // 通信・CLI

// タスクスタックサイズ
#define CONTROL_TASK_STACK_SIZE       4096
#define HIGH_SPEED_SENSOR_STACK_SIZE  4096
#define LOW_SPEED_SENSOR_STACK_SIZE   4096
#define COMMUNICATION_TASK_STACK_SIZE 8192

// タスク周期（ms）
#define CONTROL_TASK_PERIOD_MS       2.5f   // 400Hz
#define HIGH_SPEED_SENSOR_PERIOD_MS  2.5f   // 400Hz
#define LOW_SPEED_SENSOR_PERIOD_MS   20.0f  // 50Hz
#define COMMUNICATION_TASK_PERIOD_MS 100.0f // 10Hz

// IMUデータ構造体
typedef struct {
    float roll_angle;
    float pitch_angle;
    float yaw_angle;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
    float accel_x;
    float accel_y;
    float accel_z;
    float accel_z_d;
    uint32_t timestamp_us;
    bool data_valid;
} imu_data_t;

// 低速センサーデータ構造体
typedef struct {
    // 高度関連
    float altitude;
    float altitude2;
    float alt_velocity;
    int16_t raw_range;
    int16_t range;
    int16_t raw_range_front;
    int16_t range_front;
    
    // 磁気センサー
    float mx, my, mz;
    float mx_ave, my_ave, mz_ave;
    
    // オプティカルフロー
    float optical_flow_x;
    float optical_flow_y;
    float velocity_x;
    float velocity_y;
    
    // 電圧
    float voltage;
    
    // フラグ
    uint8_t over_g_flag;
    uint8_t range0_flag;
    uint8_t under_voltage_flag;
    
    uint32_t timestamp_us;
    bool data_valid;
} low_speed_sensor_data_t;

// 制御コマンド構造体
typedef struct {
    float thrust_command;
    float roll_rate_command;
    float pitch_rate_command;
    float yaw_rate_command;
    float roll_angle_command;
    float pitch_angle_command;
    float yaw_angle_command;
    uint32_t timestamp_us;
} control_command_t;

// モーターデューティ構造体
typedef struct {
    float front_right_duty;
    float front_left_duty;
    float rear_right_duty;
    float rear_left_duty;
    uint32_t timestamp_us;
} motor_duty_t;

// 共有データ構造体
typedef struct {
    // IMUデータ（高速更新）
    imu_data_t imu_data;
    SemaphoreHandle_t imu_mutex;
    
    // 低速センサーデータ
    low_speed_sensor_data_t low_speed_data;
    SemaphoreHandle_t low_speed_mutex;
    
    // 制御コマンド
    control_command_t control_cmd;
    SemaphoreHandle_t control_mutex;
    
    // モーターデューティ
    motor_duty_t motor_duty;
    SemaphoreHandle_t motor_mutex;
    
    // システム状態
    volatile uint8_t system_mode;
    volatile bool emergency_stop;
    SemaphoreHandle_t system_mutex;
    
} shared_data_t;

// タスクハンドル
extern TaskHandle_t control_task_handle;
extern TaskHandle_t high_speed_sensor_task_handle;
extern TaskHandle_t low_speed_sensor_task_handle;
extern TaskHandle_t communication_task_handle;

// 共有データ
extern shared_data_t g_shared_data;

// タスク通知用
extern TaskHandle_t control_timer_task_handle;

// 関数プロトタイプ
esp_err_t multitask_init(void);
void multitask_cleanup(void);

// データアクセス関数
bool get_imu_data(imu_data_t* data);
bool set_imu_data(const imu_data_t* data);
bool get_low_speed_sensor_data(low_speed_sensor_data_t* data);
bool set_low_speed_sensor_data(const low_speed_sensor_data_t* data);
bool get_control_command(control_command_t* cmd);
bool set_control_command(const control_command_t* cmd);
bool get_motor_duty(motor_duty_t* duty);
bool set_motor_duty(const motor_duty_t* duty);

// システム状態アクセス関数
uint8_t get_system_mode(void);
void set_system_mode(uint8_t mode);
bool get_emergency_stop(void);
void set_emergency_stop(bool stop);

#endif // MULTITASK_DATA_HPP
