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

#include "multitask_debug.hpp"
#include "multitask_data.hpp"
#include "wrapper.hpp"
#include <esp_heap_caps.h>
#include <string.h>

#if MULTITASK_DEBUG_ENABLED

// 統計情報
static multitask_stats_t g_stats;
static SemaphoreHandle_t g_stats_mutex = NULL;

// タスク実行時間測定用
static uint32_t g_task_start_time = 0;
static uint32_t g_last_control_time = 0;
static uint32_t g_last_sensor_time = 0;

void multitask_debug_init(void)
{
    // 統計情報初期化
    memset(&g_stats, 0, sizeof(multitask_stats_t));
    
    // ミューテックス作成
    g_stats_mutex = xSemaphoreCreateMutex();
    if (g_stats_mutex == NULL) {
        ESPSerial.printf("Failed to create debug stats mutex\r\n");
        return;
    }
    
    // 初期ヒープサイズ記録
    g_stats.free_heap_size = esp_get_free_heap_size();
    g_stats.min_free_heap_size = g_stats.free_heap_size;
    
    ESPSerial.printf("Multitask debug system initialized\r\n");
}

void multitask_debug_task_start(const char* task_name)
{
    g_task_start_time = micros();
}

void multitask_debug_task_end(const char* task_name)
{
    uint32_t execution_time = micros() - g_task_start_time;
    uint32_t current_time = millis();
    
    if (g_stats_mutex == NULL) return;
    
    if (xSemaphoreTake(g_stats_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        // タスク別統計更新
        if (strcmp(task_name, "Control") == 0) {
            g_stats.control_task_count++;
            if (execution_time > g_stats.control_task_max_time_us) {
                g_stats.control_task_max_time_us = execution_time;
            }
            // 移動平均計算
            g_stats.control_task_avg_time_us = 
                (g_stats.control_task_avg_time_us * 0.95f) + (execution_time * 0.05f);
            
            // 制御周波数計算
            if (g_last_control_time != 0) {
                uint32_t interval = current_time - g_last_control_time;
                if (interval > 0) {
                    g_stats.control_frequency_hz = 1000.0f / interval;
                }
            }
            g_last_control_time = current_time;
        }
        else if (strcmp(task_name, "HighSpeedSensor") == 0) {
            g_stats.high_speed_sensor_count++;
            if (execution_time > g_stats.high_speed_sensor_max_time_us) {
                g_stats.high_speed_sensor_max_time_us = execution_time;
            }
            g_stats.high_speed_sensor_avg_time_us = 
                (g_stats.high_speed_sensor_avg_time_us * 0.95f) + (execution_time * 0.05f);
            
            // センサー周波数計算
            if (g_last_sensor_time != 0) {
                uint32_t interval = current_time - g_last_sensor_time;
                if (interval > 0) {
                    g_stats.sensor_frequency_hz = 1000.0f / interval;
                }
            }
            g_last_sensor_time = current_time;
        }
        else if (strcmp(task_name, "LowSpeedSensor") == 0) {
            g_stats.low_speed_sensor_count++;
            if (execution_time > g_stats.low_speed_sensor_max_time_us) {
                g_stats.low_speed_sensor_max_time_us = execution_time;
            }
            g_stats.low_speed_sensor_avg_time_us = 
                (g_stats.low_speed_sensor_avg_time_us * 0.95f) + (execution_time * 0.05f);
        }
        else if (strcmp(task_name, "Communication") == 0) {
            g_stats.communication_task_count++;
            if (execution_time > g_stats.communication_task_max_time_us) {
                g_stats.communication_task_max_time_us = execution_time;
            }
            g_stats.communication_task_avg_time_us = 
                (g_stats.communication_task_avg_time_us * 0.95f) + (execution_time * 0.05f);
        }
        else if (strcmp(task_name, "ControlTimer") == 0) {
            g_stats.control_timer_count++;
        }
        
        // ヒープサイズ更新
        uint32_t current_heap = esp_get_free_heap_size();
        g_stats.free_heap_size = current_heap;
        if (current_heap < g_stats.min_free_heap_size) {
            g_stats.min_free_heap_size = current_heap;
        }
        
        xSemaphoreGive(g_stats_mutex);
    }
}

void multitask_debug_print_stats(void)
{
    if (g_stats_mutex == NULL) return;
    
    if (xSemaphoreTake(g_stats_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        ESPSerial.printf("\r\n=== Multitask Statistics ===\r\n");
        ESPSerial.printf("Control Task: Count=%lu, Max=%luus, Avg=%luus, Freq=%.1fHz\r\n",
            g_stats.control_task_count,
            g_stats.control_task_max_time_us,
            g_stats.control_task_avg_time_us,
            g_stats.control_frequency_hz);
        
        ESPSerial.printf("High Speed Sensor: Count=%lu, Max=%luus, Avg=%luus, Freq=%.1fHz\r\n",
            g_stats.high_speed_sensor_count,
            g_stats.high_speed_sensor_max_time_us,
            g_stats.high_speed_sensor_avg_time_us,
            g_stats.sensor_frequency_hz);
        
        ESPSerial.printf("Low Speed Sensor: Count=%lu, Max=%luus, Avg=%luus\r\n",
            g_stats.low_speed_sensor_count,
            g_stats.low_speed_sensor_max_time_us,
            g_stats.low_speed_sensor_avg_time_us);
        
        ESPSerial.printf("Communication: Count=%lu, Max=%luus, Avg=%luus\r\n",
            g_stats.communication_task_count,
            g_stats.communication_task_max_time_us,
            g_stats.communication_task_avg_time_us);
        
        ESPSerial.printf("Control Timer: Count=%lu\r\n", g_stats.control_timer_count);
        
        ESPSerial.printf("Memory: Free=%lu bytes, Min=%lu bytes\r\n",
            g_stats.free_heap_size, g_stats.min_free_heap_size);
        
        ESPSerial.printf("Errors: Mutex timeouts=%lu, Task overruns=%lu\r\n",
            g_stats.mutex_timeout_count, g_stats.task_overrun_count);
        
        ESPSerial.printf("============================\r\n\r\n");
        
        xSemaphoreGive(g_stats_mutex);
    }
}

void multitask_debug_reset_stats(void)
{
    if (g_stats_mutex == NULL) return;
    
    if (xSemaphoreTake(g_stats_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memset(&g_stats, 0, sizeof(multitask_stats_t));
        g_stats.free_heap_size = esp_get_free_heap_size();
        g_stats.min_free_heap_size = g_stats.free_heap_size;
        xSemaphoreGive(g_stats_mutex);
        ESPSerial.printf("Debug statistics reset\r\n");
    }
}

multitask_stats_t* multitask_debug_get_stats(void)
{
    return &g_stats;
}

void multitask_debug_check_stack_usage(void)
{
    ESPSerial.printf("\r\n=== Task Stack Usage ===\r\n");
    
    if (control_task_handle != NULL) {
        UBaseType_t stack_remaining = uxTaskGetStackHighWaterMark(control_task_handle);
        ESPSerial.printf("Control Task: %u words remaining\r\n", stack_remaining);
    }
    
    if (high_speed_sensor_task_handle != NULL) {
        UBaseType_t stack_remaining = uxTaskGetStackHighWaterMark(high_speed_sensor_task_handle);
        ESPSerial.printf("High Speed Sensor: %u words remaining\r\n", stack_remaining);
    }
    
    if (low_speed_sensor_task_handle != NULL) {
        UBaseType_t stack_remaining = uxTaskGetStackHighWaterMark(low_speed_sensor_task_handle);
        ESPSerial.printf("Low Speed Sensor: %u words remaining\r\n", stack_remaining);
    }
    
    if (communication_task_handle != NULL) {
        UBaseType_t stack_remaining = uxTaskGetStackHighWaterMark(communication_task_handle);
        ESPSerial.printf("Communication: %u words remaining\r\n", stack_remaining);
    }
    
    if (control_timer_task_handle != NULL) {
        UBaseType_t stack_remaining = uxTaskGetStackHighWaterMark(control_timer_task_handle);
        ESPSerial.printf("Control Timer: %u words remaining\r\n", stack_remaining);
    }
    
    ESPSerial.printf("========================\r\n\r\n");
}

void multitask_debug_print_system_info(void)
{
    ESPSerial.printf("\r\n=== System Information ===\r\n");
    ESPSerial.printf("ESP32-S3 Multitask Flight Controller\r\n");
    ESPSerial.printf("FreeRTOS Version: %s\r\n", tskKERNEL_VERSION_NUMBER);
    ESPSerial.printf("CPU Frequency: %lu MHz\r\n", getCpuFrequencyMhz());
    ESPSerial.printf("Free Heap: %lu bytes\r\n", esp_get_free_heap_size());
    ESPSerial.printf("Largest Free Block: %lu bytes\r\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    ESPSerial.printf("Total Heap: %lu bytes\r\n", heap_caps_get_total_size(MALLOC_CAP_8BIT));
    ESPSerial.printf("Number of Tasks: %u\r\n", uxTaskGetNumberOfTasks());
    ESPSerial.printf("Tick Rate: %lu Hz\r\n", configTICK_RATE_HZ);
    ESPSerial.printf("==========================\r\n\r\n");
}

#else

// デバッグ無効時のダミー関数
void multitask_debug_init(void) {}
void multitask_debug_task_start(const char* task_name) {}
void multitask_debug_task_end(const char* task_name) {}
void multitask_debug_print_stats(void) {}
void multitask_debug_reset_stats(void) {}
multitask_stats_t* multitask_debug_get_stats(void) { return NULL; }
void multitask_debug_check_stack_usage(void) {}
void multitask_debug_print_system_info(void) {}

#endif // MULTITASK_DEBUG_ENABLED
