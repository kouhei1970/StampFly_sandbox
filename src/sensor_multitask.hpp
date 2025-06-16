/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * マルチタスク対応センサー処理ヘッダー
 */

#ifndef SENSOR_MULTITASK_HPP
#define SENSOR_MULTITASK_HPP

// 高速センサー読み取り（400Hz）- IMUのみ
// 戻り値: 処理時間（秒）
float sensor_read_high_speed(void);

// 低速センサー読み取り（50Hz以下）- ToF、磁気センサー、オプティカルフロー等
// 戻り値: 処理時間（秒）
float sensor_read_low_speed(void);

// フィルタリセット関数（モード変更時）
void sensor_reset_filters(void);

#endif // SENSOR_MULTITASK_HPP
