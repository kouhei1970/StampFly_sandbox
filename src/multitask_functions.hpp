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

#ifndef MULTITASK_FUNCTIONS_HPP
#define MULTITASK_FUNCTIONS_HPP

#include "multitask_data.hpp"
#include "flight_control.hpp"
#include "sensor.hpp"

// タスク関数プロトタイプ
void control_timer_task(void* parameter);
void control_task(void* parameter);
void high_speed_sensor_task(void* parameter);
void low_speed_sensor_task(void* parameter);
void communication_task(void* parameter);

// タスク作成・管理関数
esp_err_t create_all_tasks(void);
void delete_all_tasks(void);

// 制御関連の内部関数
void control_task_init(void);
void control_task_process(void);
void control_get_command(void);
void control_angle_control(void);
void control_rate_control(void);
void control_motor_output(void);

// センサー関連の内部関数
void high_speed_sensor_init(void);
void high_speed_sensor_process(void);
void low_speed_sensor_init(void);
void low_speed_sensor_process(void);

// 通信関連の内部関数
void communication_task_init(void);
void communication_task_process(void);

// ユーティリティ関数
uint32_t get_timestamp_us(void);
void task_delay_until_next_period(TickType_t* last_wake_time, TickType_t period_ticks);

#endif // MULTITASK_FUNCTIONS_HPP
