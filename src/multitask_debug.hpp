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

#ifndef MULTITASK_DEBUG_HPP
#define MULTITASK_DEBUG_HPP

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// デバッグ機能の有効/無効
#define MULTITASK_DEBUG_ENABLED 1

// タスク統計情報
typedef struct {
    uint32_t control_task_count;
    uint32_t high_speed_sensor_count;
    uint32_t low_speed_sensor_count;
    uint32_t communication_task_count;
    uint32_t control_timer_count;
    
    uint32_t control_task_max_time_us;
    uint32_t high_speed_sensor_max_time_us;
    uint32_t low_speed_sensor_max_time_us;
    uint32_t communication_task_max_time_us;
    
    uint32_t control_task_avg_time_us;
    uint32_t high_speed_sensor_avg_time_us;
    uint32_t low_speed_sensor_avg_time_us;
    uint32_t communication_task_avg_time_us;
    
    uint32_t mutex_timeout_count;
    uint32_t task_overrun_count;
    
    uint32_t free_heap_size;
    uint32_t min_free_heap_size;
    
    float control_frequency_hz;
    float sensor_frequency_hz;
} multitask_stats_t;

// デバッグ関数
void multitask_debug_init(void);
void multitask_debug_task_start(const char* task_name);
void multitask_debug_task_end(const char* task_name);
void multitask_debug_print_stats(void);
void multitask_debug_reset_stats(void);
multitask_stats_t* multitask_debug_get_stats(void);

// タスクスタック使用量チェック
void multitask_debug_check_stack_usage(void);

// システム情報表示
void multitask_debug_print_system_info(void);

// パフォーマンス測定マクロ
#if MULTITASK_DEBUG_ENABLED
#define MULTITASK_DEBUG_TASK_START(name) multitask_debug_task_start(name)
#define MULTITASK_DEBUG_TASK_END(name) multitask_debug_task_end(name)
#else
#define MULTITASK_DEBUG_TASK_START(name)
#define MULTITASK_DEBUG_TASK_END(name)
#endif

#endif // MULTITASK_DEBUG_HPP
