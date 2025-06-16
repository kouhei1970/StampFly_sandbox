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

#include "multitask_data.hpp"
#include "wrapper.hpp"
#include <string.h>

// タスクハンドル
TaskHandle_t control_task_handle = NULL;
TaskHandle_t high_speed_sensor_task_handle = NULL;
TaskHandle_t low_speed_sensor_task_handle = NULL;
TaskHandle_t communication_task_handle = NULL;
TaskHandle_t control_timer_task_handle = NULL;

// 共有データ
shared_data_t g_shared_data;

// ミューテックス待機時間（ms）
#define MUTEX_WAIT_TIME_MS 10

esp_err_t multitask_init(void)
{
    // 共有データ初期化
    memset(&g_shared_data, 0, sizeof(shared_data_t));
    
    // ミューテックス作成
    g_shared_data.imu_mutex = xSemaphoreCreateMutex();
    if (g_shared_data.imu_mutex == NULL) {
        ESPSerial.printf("Failed to create IMU mutex\r\n");
        return ESP_FAIL;
    }
    
    g_shared_data.low_speed_mutex = xSemaphoreCreateMutex();
    if (g_shared_data.low_speed_mutex == NULL) {
        ESPSerial.printf("Failed to create low speed sensor mutex\r\n");
        return ESP_FAIL;
    }
    
    g_shared_data.control_mutex = xSemaphoreCreateMutex();
    if (g_shared_data.control_mutex == NULL) {
        ESPSerial.printf("Failed to create control mutex\r\n");
        return ESP_FAIL;
    }
    
    g_shared_data.motor_mutex = xSemaphoreCreateMutex();
    if (g_shared_data.motor_mutex == NULL) {
        ESPSerial.printf("Failed to create motor mutex\r\n");
        return ESP_FAIL;
    }
    
    g_shared_data.system_mutex = xSemaphoreCreateMutex();
    if (g_shared_data.system_mutex == NULL) {
        ESPSerial.printf("Failed to create system mutex\r\n");
        return ESP_FAIL;
    }
    
    // 初期値設定
    g_shared_data.system_mode = 0; // INIT_MODE
    g_shared_data.emergency_stop = false;
    
    ESPSerial.printf("Multitask data structures initialized\r\n");
    return ESP_OK;
}

void multitask_cleanup(void)
{
    // ミューテックス削除
    if (g_shared_data.imu_mutex != NULL) {
        vSemaphoreDelete(g_shared_data.imu_mutex);
        g_shared_data.imu_mutex = NULL;
    }
    
    if (g_shared_data.low_speed_mutex != NULL) {
        vSemaphoreDelete(g_shared_data.low_speed_mutex);
        g_shared_data.low_speed_mutex = NULL;
    }
    
    if (g_shared_data.control_mutex != NULL) {
        vSemaphoreDelete(g_shared_data.control_mutex);
        g_shared_data.control_mutex = NULL;
    }
    
    if (g_shared_data.motor_mutex != NULL) {
        vSemaphoreDelete(g_shared_data.motor_mutex);
        g_shared_data.motor_mutex = NULL;
    }
    
    if (g_shared_data.system_mutex != NULL) {
        vSemaphoreDelete(g_shared_data.system_mutex);
        g_shared_data.system_mutex = NULL;
    }
}

// IMUデータアクセス関数
bool get_imu_data(imu_data_t* data)
{
    if (data == NULL || g_shared_data.imu_mutex == NULL) {
        return false;
    }
    
    if (xSemaphoreTake(g_shared_data.imu_mutex, pdMS_TO_TICKS(MUTEX_WAIT_TIME_MS)) == pdTRUE) {
        memcpy(data, &g_shared_data.imu_data, sizeof(imu_data_t));
        xSemaphoreGive(g_shared_data.imu_mutex);
        return true;
    }
    
    return false;
}

bool set_imu_data(const imu_data_t* data)
{
    if (data == NULL || g_shared_data.imu_mutex == NULL) {
        return false;
    }
    
    if (xSemaphoreTake(g_shared_data.imu_mutex, pdMS_TO_TICKS(MUTEX_WAIT_TIME_MS)) == pdTRUE) {
        memcpy(&g_shared_data.imu_data, data, sizeof(imu_data_t));
        xSemaphoreGive(g_shared_data.imu_mutex);
        return true;
    }
    
    return false;
}

// 低速センサーデータアクセス関数
bool get_low_speed_sensor_data(low_speed_sensor_data_t* data)
{
    if (data == NULL || g_shared_data.low_speed_mutex == NULL) {
        return false;
    }
    
    if (xSemaphoreTake(g_shared_data.low_speed_mutex, pdMS_TO_TICKS(MUTEX_WAIT_TIME_MS)) == pdTRUE) {
        memcpy(data, &g_shared_data.low_speed_data, sizeof(low_speed_sensor_data_t));
        xSemaphoreGive(g_shared_data.low_speed_mutex);
        return true;
    }
    
    return false;
}

bool set_low_speed_sensor_data(const low_speed_sensor_data_t* data)
{
    if (data == NULL || g_shared_data.low_speed_mutex == NULL) {
        return false;
    }
    
    if (xSemaphoreTake(g_shared_data.low_speed_mutex, pdMS_TO_TICKS(MUTEX_WAIT_TIME_MS)) == pdTRUE) {
        memcpy(&g_shared_data.low_speed_data, data, sizeof(low_speed_sensor_data_t));
        xSemaphoreGive(g_shared_data.low_speed_mutex);
        return true;
    }
    
    return false;
}

// 制御コマンドアクセス関数
bool get_control_command(control_command_t* cmd)
{
    if (cmd == NULL || g_shared_data.control_mutex == NULL) {
        return false;
    }
    
    if (xSemaphoreTake(g_shared_data.control_mutex, pdMS_TO_TICKS(MUTEX_WAIT_TIME_MS)) == pdTRUE) {
        memcpy(cmd, &g_shared_data.control_cmd, sizeof(control_command_t));
        xSemaphoreGive(g_shared_data.control_mutex);
        return true;
    }
    
    return false;
}

bool set_control_command(const control_command_t* cmd)
{
    if (cmd == NULL || g_shared_data.control_mutex == NULL) {
        return false;
    }
    
    if (xSemaphoreTake(g_shared_data.control_mutex, pdMS_TO_TICKS(MUTEX_WAIT_TIME_MS)) == pdTRUE) {
        memcpy(&g_shared_data.control_cmd, cmd, sizeof(control_command_t));
        xSemaphoreGive(g_shared_data.control_mutex);
        return true;
    }
    
    return false;
}

// モーターデューティアクセス関数
bool get_motor_duty(motor_duty_t* duty)
{
    if (duty == NULL || g_shared_data.motor_mutex == NULL) {
        return false;
    }
    
    if (xSemaphoreTake(g_shared_data.motor_mutex, pdMS_TO_TICKS(MUTEX_WAIT_TIME_MS)) == pdTRUE) {
        memcpy(duty, &g_shared_data.motor_duty, sizeof(motor_duty_t));
        xSemaphoreGive(g_shared_data.motor_mutex);
        return true;
    }
    
    return false;
}

bool set_motor_duty(const motor_duty_t* duty)
{
    if (duty == NULL || g_shared_data.motor_mutex == NULL) {
        return false;
    }
    
    if (xSemaphoreTake(g_shared_data.motor_mutex, pdMS_TO_TICKS(MUTEX_WAIT_TIME_MS)) == pdTRUE) {
        memcpy(&g_shared_data.motor_duty, duty, sizeof(motor_duty_t));
        xSemaphoreGive(g_shared_data.motor_mutex);
        return true;
    }
    
    return false;
}

// システム状態アクセス関数
uint8_t get_system_mode(void)
{
    if (g_shared_data.system_mutex == NULL) {
        return 0;
    }
    
    uint8_t mode = 0;
    if (xSemaphoreTake(g_shared_data.system_mutex, pdMS_TO_TICKS(MUTEX_WAIT_TIME_MS)) == pdTRUE) {
        mode = g_shared_data.system_mode;
        xSemaphoreGive(g_shared_data.system_mutex);
    }
    
    return mode;
}

void set_system_mode(uint8_t mode)
{
    if (g_shared_data.system_mutex == NULL) {
        return;
    }
    
    if (xSemaphoreTake(g_shared_data.system_mutex, pdMS_TO_TICKS(MUTEX_WAIT_TIME_MS)) == pdTRUE) {
        g_shared_data.system_mode = mode;
        xSemaphoreGive(g_shared_data.system_mutex);
    }
}

bool get_emergency_stop(void)
{
    if (g_shared_data.system_mutex == NULL) {
        return true; // セーフティファースト
    }
    
    bool stop = true;
    if (xSemaphoreTake(g_shared_data.system_mutex, pdMS_TO_TICKS(MUTEX_WAIT_TIME_MS)) == pdTRUE) {
        stop = g_shared_data.emergency_stop;
        xSemaphoreGive(g_shared_data.system_mutex);
    }
    
    return stop;
}

void set_emergency_stop(bool stop)
{
    if (g_shared_data.system_mutex == NULL) {
        return;
    }
    
    if (xSemaphoreTake(g_shared_data.system_mutex, pdMS_TO_TICKS(MUTEX_WAIT_TIME_MS)) == pdTRUE) {
        g_shared_data.emergency_stop = stop;
        xSemaphoreGive(g_shared_data.system_mutex);
    }
}
