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

#ifndef FREERTOS_TASKS_HPP
#define FREERTOS_TASKS_HPP

#include <stdint.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/timers.h>
#include <esp_log.h>
#include <esp_heap_caps.h>

// タスク優先度定義
#define CONTROL_TASK_PRIORITY       (configMAX_PRIORITIES - 1)  // 最高優先度
#define IMU_TASK_PRIORITY          (configMAX_PRIORITIES - 2)  // 高優先度
#define MEDIUM_SENSOR_TASK_PRIORITY (configMAX_PRIORITIES - 3)  // 中優先度
#define SLOW_SENSOR_TASK_PRIORITY   (configMAX_PRIORITIES - 4)  // 低優先度
#define VERY_SLOW_SENSOR_TASK_PRIORITY (configMAX_PRIORITIES - 5)  // 最低優先度

// タスクスタックサイズ定義
#define CONTROL_TASK_STACK_SIZE       (4096)
#define IMU_TASK_STACK_SIZE          (4096)
#define MEDIUM_SENSOR_TASK_STACK_SIZE (3072)
#define SLOW_SENSOR_TASK_STACK_SIZE   (3072)
#define VERY_SLOW_SENSOR_TASK_STACK_SIZE (2048)

// タスク実行周期定義（ティック数）
// FreeRTOSのデフォルトtick周期は1ms
#define CONTROL_TASK_PERIOD_TICKS    (3)     // 約333Hz = 3ms (400Hzに最も近い実現可能な値)
#define IMU_TASK_PERIOD_TICKS        (3)     // 約333Hz = 3ms (400Hzに最も近い実現可能な値)
#define MEDIUM_SENSOR_TASK_PERIOD_TICKS (10) // 100Hz = 10ms
#define SLOW_SENSOR_TASK_PERIOD_TICKS   (20) // 50Hz = 20ms
#define VERY_SLOW_SENSOR_TASK_PERIOD_TICKS  (33) // 約30Hz = 33ms

// タスクハンドル
extern TaskHandle_t control_task_handle;
extern TaskHandle_t imu_task_handle;
extern TaskHandle_t medium_sensor_task_handle;
extern TaskHandle_t slow_sensor_task_handle;
extern TaskHandle_t very_slow_sensor_task_handle;

// セマフォとキュー
extern SemaphoreHandle_t sensor_data_mutex;
extern QueueHandle_t imu_data_queue;
extern QueueHandle_t sensor_data_queue;

// データ構造体定義
typedef struct {
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
    float accel_x;
    float accel_y;
    float accel_z;
    uint32_t timestamp;
} imu_data_t;

typedef struct {
    float magnetometer_x;
    float magnetometer_y;
    float magnetometer_z;
    float optical_flow_x;
    float optical_flow_y;
    uint32_t timestamp;
} medium_sensor_data_t;

typedef struct {
    float pressure;
    float altitude;
    float temperature;
    uint16_t tof_distance;
    uint32_t timestamp;
} slow_sensor_data_t;

typedef struct {
    float voltage;
    float current;
    float power;
    uint8_t battery_level;
    uint32_t timestamp;
} very_slow_sensor_data_t;

// タスク関数プロトタイプ
void control_task(void *pvParameters);
void imu_task(void *pvParameters);
void medium_sensor_task(void *pvParameters);
void slow_sensor_task(void *pvParameters);
void very_slow_sensor_task(void *pvParameters);

// FreeRTOSシステム初期化・制御関数
void freertos_tasks_init(void);
void freertos_tasks_start(void);
void freertos_tasks_stop(void);
bool freertos_tasks_is_running(void);

// タスク統計・デバッグ関数
void print_task_stats(void);
void print_heap_info(void);

#endif // FREERTOS_TASKS_HPP
