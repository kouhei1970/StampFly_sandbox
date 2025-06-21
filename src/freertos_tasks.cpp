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

//
// StampFly FreeRTOS Multi-Task System
//
// Designed by Kouhei Ito 2024
//

#include "freertos_tasks.hpp"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_system.h>

static const char* TAG = "FREERTOS_TASKS";

// タスクハンドル
TaskHandle_t control_task_handle = NULL;
TaskHandle_t imu_task_handle = NULL;
TaskHandle_t medium_sensor_task_handle = NULL;
TaskHandle_t slow_sensor_task_handle = NULL;
TaskHandle_t very_slow_sensor_task_handle = NULL;

// セマフォとキュー
SemaphoreHandle_t sensor_data_mutex = NULL;
QueueHandle_t imu_data_queue = NULL;
QueueHandle_t sensor_data_queue = NULL;

// システム状態フラグ
static bool tasks_running = false;
static bool tasks_initialized = false;

// タスク統計用変数
static uint32_t control_task_counter = 0;
static uint32_t imu_task_counter = 0;
static uint32_t medium_sensor_task_counter = 0;
static uint32_t slow_sensor_task_counter = 0;
static uint32_t very_slow_sensor_task_counter = 0;

//=============================================================================
// 制御タスク (333Hz)
// 姿勢制御、レート制御、モーター出力を担当
//=============================================================================
void control_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = CONTROL_TASK_PERIOD_TICKS;
    
    // 初期化
    xLastWakeTime = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "Control task started (333Hz)");
    
    while (tasks_running) {
        // 制御処理（現在は空の実装）
        // TODO: 既存の制御ロジックをここに移植
        
        // タスクカウンタ更新
        control_task_counter++;
        
        // デバッグ用（1秒に1回ログ出力）
        if (control_task_counter % 333 == 0) {
            ESP_LOGD(TAG, "Control task running: %lu", control_task_counter);
        }
        
        // 次の実行まで待機
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    
    ESP_LOGI(TAG, "Control task stopped");
    vTaskDelete(NULL);
}

//=============================================================================
// IMUタスク (333Hz)
// BMI270からの加速度・ジャイロデータ読み取りを担当
//=============================================================================
void imu_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = IMU_TASK_PERIOD_TICKS;
    imu_data_t imu_data;
    
    // 初期化
    xLastWakeTime = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "IMU task started (333Hz)");
    
    while (tasks_running) {
        // IMUデータ読み取り処理（現在は空の実装）
        // TODO: BMI270からのデータ読み取りロジックをここに移植
        
        // ダミーデータ設定（実装時に削除）
        imu_data.roll_rate = 0.0f;
        imu_data.pitch_rate = 0.0f;
        imu_data.yaw_rate = 0.0f;
        imu_data.accel_x = 0.0f;
        imu_data.accel_y = 0.0f;
        imu_data.accel_z = 0.0f;
        imu_data.timestamp = xTaskGetTickCount();
        
        // データをキューに送信（ノンブロッキング）
        if (imu_data_queue != NULL) {
            xQueueSend(imu_data_queue, &imu_data, 0);
        }
        
        // タスクカウンタ更新
        imu_task_counter++;
        
        // デバッグ用（1秒に1回ログ出力）
        if (imu_task_counter % 333 == 0) {
            ESP_LOGD(TAG, "IMU task running: %lu", imu_task_counter);
        }
        
        // 次の実行まで待機
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    
    ESP_LOGI(TAG, "IMU task stopped");
    vTaskDelete(NULL);
}

//=============================================================================
// 中速センサータスク (100Hz)
// 磁力計、オプティカルフローセンサーを担当
//=============================================================================
void medium_sensor_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = MEDIUM_SENSOR_TASK_PERIOD_TICKS;
    medium_sensor_data_t sensor_data;
    
    // 初期化
    xLastWakeTime = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "Medium sensor task started (100Hz)");
    
    while (tasks_running) {
        // 中速センサーデータ読み取り処理（現在は空の実装）
        // TODO: 磁力計、オプティカルフローセンサーの読み取りロジックをここに移植
        
        // ダミーデータ設定（実装時に削除）
        sensor_data.magnetometer_x = 0.0f;
        sensor_data.magnetometer_y = 0.0f;
        sensor_data.magnetometer_z = 0.0f;
        sensor_data.optical_flow_x = 0.0f;
        sensor_data.optical_flow_y = 0.0f;
        sensor_data.timestamp = xTaskGetTickCount();
        
        // タスクカウンタ更新
        medium_sensor_task_counter++;
        
        // デバッグ用（1秒に1回ログ出力）
        if (medium_sensor_task_counter % 100 == 0) {
            ESP_LOGD(TAG, "Medium sensor task running: %lu", medium_sensor_task_counter);
        }
        
        // 次の実行まで待機
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    
    ESP_LOGI(TAG, "Medium sensor task stopped");
    vTaskDelete(NULL);
}

//=============================================================================
// 低速センサータスク (50Hz)
// 気圧センサー、ToFセンサーを担当
//=============================================================================
void slow_sensor_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = SLOW_SENSOR_TASK_PERIOD_TICKS;
    slow_sensor_data_t sensor_data;
    
    // 初期化
    xLastWakeTime = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "Slow sensor task started (50Hz)");
    
    while (tasks_running) {
        // 低速センサーデータ読み取り処理（現在は空の実装）
        // TODO: 気圧センサー、ToFセンサーの読み取りロジックをここに移植
        
        // ダミーデータ設定（実装時に削除）
        sensor_data.pressure = 0.0f;
        sensor_data.altitude = 0.0f;
        sensor_data.temperature = 0.0f;
        sensor_data.tof_distance = 0;
        sensor_data.timestamp = xTaskGetTickCount();
        
        // タスクカウンタ更新
        slow_sensor_task_counter++;
        
        // デバッグ用（1秒に1回ログ出力）
        if (slow_sensor_task_counter % 50 == 0) {
            ESP_LOGD(TAG, "Slow sensor task running: %lu", slow_sensor_task_counter);
        }
        
        // 次の実行まで待機
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    
    ESP_LOGI(TAG, "Slow sensor task stopped");
    vTaskDelete(NULL);
}

//=============================================================================
// 超低速センサータスク (約30Hz)
// 電圧監視、テレメトリを担当
//=============================================================================
void very_slow_sensor_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = VERY_SLOW_SENSOR_TASK_PERIOD_TICKS;
    very_slow_sensor_data_t sensor_data;
    
    // 初期化
    xLastWakeTime = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "Very slow sensor task started (約30Hz)");
    
    while (tasks_running) {
        // 超低速センサーデータ読み取り処理（現在は空の実装）
        // TODO: 電圧監視、テレメトリの処理ロジックをここに移植
        
        // ダミーデータ設定（実装時に削除）
        sensor_data.voltage = 0.0f;
        sensor_data.current = 0.0f;
        sensor_data.power = 0.0f;
        sensor_data.battery_level = 0;
        sensor_data.timestamp = xTaskGetTickCount();
        
        // タスクカウンタ更新
        very_slow_sensor_task_counter++;
        
        // デバッグ用（1秒に1回ログ出力）
        if (very_slow_sensor_task_counter % 30 == 0) {
            ESP_LOGD(TAG, "Very slow sensor task running: %lu", very_slow_sensor_task_counter);
        }
        
        // 次の実行まで待機
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    
    ESP_LOGI(TAG, "Very slow sensor task stopped");
    vTaskDelete(NULL);
}

//=============================================================================
// FreeRTOSシステム初期化
//=============================================================================
void freertos_tasks_init(void)
{
    if (tasks_initialized) {
        ESP_LOGW(TAG, "Tasks already initialized");
        return;
    }
    
    ESP_LOGI(TAG, "Initializing FreeRTOS tasks...");
    
    // セマフォ作成
    sensor_data_mutex = xSemaphoreCreateMutex();
    if (sensor_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor data mutex");
        return;
    }
    
    // キュー作成
    imu_data_queue = xQueueCreate(10, sizeof(imu_data_t));
    if (imu_data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create IMU data queue");
        return;
    }
    
    sensor_data_queue = xQueueCreate(5, sizeof(medium_sensor_data_t));
    if (sensor_data_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor data queue");
        return;
    }
    
    tasks_initialized = true;
    ESP_LOGI(TAG, "FreeRTOS tasks initialized successfully");
}

//=============================================================================
// FreeRTOSタスク開始
//=============================================================================
void freertos_tasks_start(void)
{
    if (!tasks_initialized) {
        ESP_LOGE(TAG, "Tasks not initialized. Call freertos_tasks_init() first");
        return;
    }
    
    if (tasks_running) {
        ESP_LOGW(TAG, "Tasks already running");
        return;
    }
    
    ESP_LOGI(TAG, "Starting FreeRTOS tasks...");
    
    tasks_running = true;
    
    // 制御タスク作成
    BaseType_t result = xTaskCreatePinnedToCore(
        control_task,
        "ControlTask",
        CONTROL_TASK_STACK_SIZE,
        NULL,
        CONTROL_TASK_PRIORITY,
        &control_task_handle,
        1  // Core 1で実行
    );
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control task");
        tasks_running = false;
        return;
    }
    
    // IMUタスク作成
    result = xTaskCreatePinnedToCore(
        imu_task,
        "IMUTask",
        IMU_TASK_STACK_SIZE,
        NULL,
        IMU_TASK_PRIORITY,
        &imu_task_handle,
        1  // Core 1で実行
    );
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU task");
        tasks_running = false;
        return;
    }
    
    // 中速センサータスク作成
    result = xTaskCreatePinnedToCore(
        medium_sensor_task,
        "MediumSensorTask",
        MEDIUM_SENSOR_TASK_STACK_SIZE,
        NULL,
        MEDIUM_SENSOR_TASK_PRIORITY,
        &medium_sensor_task_handle,
        0  // Core 0で実行
    );
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create medium sensor task");
        tasks_running = false;
        return;
    }
    
    // 低速センサータスク作成
    result = xTaskCreatePinnedToCore(
        slow_sensor_task,
        "SlowSensorTask",
        SLOW_SENSOR_TASK_STACK_SIZE,
        NULL,
        SLOW_SENSOR_TASK_PRIORITY,
        &slow_sensor_task_handle,
        0  // Core 0で実行
    );
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create slow sensor task");
        tasks_running = false;
        return;
    }
    
    // 超低速センサータスク作成
    result = xTaskCreatePinnedToCore(
        very_slow_sensor_task,
        "VerySlowSensorTask",
        VERY_SLOW_SENSOR_TASK_STACK_SIZE,
        NULL,
        VERY_SLOW_SENSOR_TASK_PRIORITY,
        &very_slow_sensor_task_handle,
        0  // Core 0で実行
    );
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create very slow sensor task");
        tasks_running = false;
        return;
    }
    
    ESP_LOGI(TAG, "All FreeRTOS tasks started successfully");
}

//=============================================================================
// FreeRTOSタスク停止
//=============================================================================
void freertos_tasks_stop(void)
{
    if (!tasks_running) {
        ESP_LOGW(TAG, "Tasks not running");
        return;
    }
    
    ESP_LOGI(TAG, "Stopping FreeRTOS tasks...");
    
    tasks_running = false;
    
    // タスクが自然に終了するまで少し待機
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "FreeRTOS tasks stopped");
}

//=============================================================================
// FreeRTOSタスク実行状態確認
//=============================================================================
bool freertos_tasks_is_running(void)
{
    return tasks_running;
}

//=============================================================================
// タスク統計情報出力
//=============================================================================
void print_task_stats(void)
{
    ESP_LOGI(TAG, "=== Task Statistics ===");
    ESP_LOGI(TAG, "Control Task: %lu executions", control_task_counter);
    ESP_LOGI(TAG, "IMU Task: %lu executions", imu_task_counter);
    ESP_LOGI(TAG, "Medium Sensor Task: %lu executions", medium_sensor_task_counter);
    ESP_LOGI(TAG, "Slow Sensor Task: %lu executions", slow_sensor_task_counter);
    ESP_LOGI(TAG, "Very Slow Sensor Task: %lu executions", very_slow_sensor_task_counter);
    ESP_LOGI(TAG, "Tasks Running: %s", tasks_running ? "Yes" : "No");
    ESP_LOGI(TAG, "Tasks Initialized: %s", tasks_initialized ? "Yes" : "No");
}

//=============================================================================
// ヒープ情報出力
//=============================================================================
void print_heap_info(void)
{
    ESP_LOGI(TAG, "=== Heap Information ===");
    ESP_LOGI(TAG, "Free heap size: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Minimum free heap size: %lu bytes", esp_get_minimum_free_heap_size());
    ESP_LOGI(TAG, "Largest free block: %lu bytes", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
}
