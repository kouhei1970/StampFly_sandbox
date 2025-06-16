/*
 * MIT License
 *
 * Copyright (c) 2025 Kouhei Ito
 * Copyright (c) 2025 M5Stack
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

#include "cli.hpp"
#include "sensor.hpp"
#include "imu.hpp"
#include "tof.hpp"
#include "mag.hpp"
#include "opt.hpp"
#include "flight_control.hpp"
#include "pid.hpp"
#include <string.h>

// グローバル変数
static char cli_buffer[CLI_BUFFER_SIZE];
static int cli_buffer_index = 0;
volatile bool streaming_enabled = false;
volatile uint32_t stream_interval_ms = 100;
static uint32_t last_stream_time = 0;

// ストリーミング出力フォーマット
typedef enum {
    STREAM_FORMAT_DEFAULT = 0,  // デフォルト形式
    STREAM_FORMAT_CSV,          // CSV形式（カンマ区切り）
    STREAM_FORMAT_TSV,          // TSV形式（タブ区切り）
    STREAM_FORMAT_TELEPLOT      // Teleplot形式
} stream_format_t;

static stream_format_t stream_format = STREAM_FORMAT_DEFAULT;

// オフセット計算用の状態管理
typedef struct {
    bool active;
    uint32_t start_time;
    uint32_t last_print_time;
    int target_samples;
    int current_samples;
} offset_calc_t;

static offset_calc_t offset_calc = {false, 0, 0, 0, 0};

// コマンド履歴機能
static char command_history[CLI_HISTORY_SIZE][CLI_BUFFER_SIZE];
static int history_count = 0;
static int history_index = -1;
static int current_history_pos = -1;
static char temp_buffer[CLI_BUFFER_SIZE]; // 編集中のコマンドを一時保存

// エスケープシーケンス処理用
static int escape_state = 0;
static char escape_buffer[8];
static int escape_index = 0;

// 利用可能なコマンド一覧
static const cli_command_t cli_commands[] = {
    {"help", cmd_help, "ヘルプを表示"},
    {"imu", cmd_imu, "IMUデータを表示 (加速度・ジャイロ)"},
    {"tof", cmd_tof, "ToFセンサーデータを表示 (距離)"},
    {"voltage", cmd_voltage, "バッテリー電圧を表示"},
    {"mag", cmd_mag, "磁気センサーデータを表示"},
    {"optical", cmd_optical, "オプティカルフローデータを表示/テスト [duration_sec]"},
    {"attitude", cmd_attitude, "姿勢角を表示 (Roll/Pitch/Yaw)"},
    {"all", cmd_all_sensors, "全センサーデータを表示"},
    {"status", cmd_status, "システム状態を表示"},
    {"stream", cmd_stream, "データストリーミング制御 [start/stop] [interval_ms]"},
    {"offset", cmd_offset, "センサーオフセット制御 [get/reset/start]"},
    {"mag_cal", cmd_mag_cal, "磁気センサー校正 [start/stop/reset/save]"},
    {"reset", cmd_reset, "システムリセット [ahrs/sensors/all]"},
    {"save", cmd_save, "設定保存 [offsets/mag_cal/pid/all]"},
    {"load", cmd_load, "設定読み込み [offsets/mag_cal/pid/all]"},
    {"pid", cmd_pid, "PIDゲイン制御 [get/set/save/load] [controller] [param] [value]"},
    {"history", cmd_history, "コマンド履歴を表示"},
    {NULL, NULL, NULL} // 終端
};

// コマンド履歴機能のヘルパー関数
void add_to_history(const char* command)
{
    if (strlen(command) == 0) return;
    
    // 同じコマンドが連続している場合は追加しない
    if (history_count > 0 && strcmp(command_history[(history_count - 1) % CLI_HISTORY_SIZE], command) == 0) {
        return;
    }
    
    // 履歴に追加
    strncpy(command_history[history_count % CLI_HISTORY_SIZE], command, CLI_BUFFER_SIZE - 1);
    command_history[history_count % CLI_HISTORY_SIZE][CLI_BUFFER_SIZE - 1] = '\0';
    history_count++;
    
    // 履歴インデックスをリセット
    current_history_pos = -1;
}

void clear_line()
{
    // より堅牢な行クリア処理
    ESPSerial.print("\r");           // カーソルを行の先頭に移動
    for (int i = 0; i < 80; i++) {   // 80文字分スペースで上書き
        ESPSerial.print(" ");
    }
    ESPSerial.print("\r");           // 再度行の先頭に移動
    ESPSerial.print("StampFly> ");   // プロンプトを表示
}

void clear_current_line_simple()
{
    // シンプルな行クリア（ANSI非対応端末用）
    ESPSerial.print("\r");
    for (int i = 0; i < 80; i++) {
        ESPSerial.print(" ");
    }
    ESPSerial.print("\r");
}

void display_current_buffer()
{
    ESPSerial.print(cli_buffer);
}

bool handle_arrow_keys(char c)
{
    // エスケープシーケンスの処理
    if (escape_state == 0 && c == 0x1B) { // ESC
        escape_state = 1;
        escape_index = 0;
        escape_buffer[escape_index++] = c;
        return true;
    } else if (escape_state == 1 && c == '[') {
        escape_state = 2;
        escape_buffer[escape_index++] = c;
        return true;
    } else if (escape_state == 2) {
        escape_buffer[escape_index++] = c;
        escape_buffer[escape_index] = '\0';
        escape_state = 0;
        
        if (c == 'A') { // 上矢印
            if (history_count > 0) {
                // 現在編集中のコマンドを保存
                if (current_history_pos == -1) {
                    strncpy(temp_buffer, cli_buffer, CLI_BUFFER_SIZE);
                    temp_buffer[CLI_BUFFER_SIZE - 1] = '\0';
                }
                
                // 履歴を遡る
                if (current_history_pos == -1) {
                    current_history_pos = (history_count - 1) % CLI_HISTORY_SIZE;
                } else {
                    int prev_pos = current_history_pos - 1;
                    if (prev_pos < 0) {
                        if (history_count >= CLI_HISTORY_SIZE) {
                            prev_pos = CLI_HISTORY_SIZE - 1;
                        } else {
                            prev_pos = history_count - 1;
                        }
                    }
                    
                    // 有効な履歴があるかチェック
                    int start_index = (history_count >= CLI_HISTORY_SIZE) ? 
                                     (history_count % CLI_HISTORY_SIZE) : 0;
                    if (prev_pos != start_index || history_count >= CLI_HISTORY_SIZE) {
                        current_history_pos = prev_pos;
                    }
                }
                
                // バッファをクリアして履歴のコマンドを設定
                clear_line();
                strncpy(cli_buffer, command_history[current_history_pos], CLI_BUFFER_SIZE);
                cli_buffer[CLI_BUFFER_SIZE - 1] = '\0';
                cli_buffer_index = strlen(cli_buffer);
                display_current_buffer();
            }
            return true;
        } else if (c == 'B') { // 下矢印
            if (current_history_pos != -1) {
                // 履歴を進める
                int next_pos = (current_history_pos + 1) % CLI_HISTORY_SIZE;
                int latest_pos = (history_count - 1) % CLI_HISTORY_SIZE;
                
                clear_line();
                
                if (current_history_pos == latest_pos) {
                    // 最新の履歴から編集中のコマンドに戻る
                    strncpy(cli_buffer, temp_buffer, CLI_BUFFER_SIZE);
                    current_history_pos = -1;
                } else {
                    // 次の履歴に移動
                    current_history_pos = next_pos;
                    strncpy(cli_buffer, command_history[current_history_pos], CLI_BUFFER_SIZE);
                }
                
                cli_buffer[CLI_BUFFER_SIZE - 1] = '\0';
                cli_buffer_index = strlen(cli_buffer);
                display_current_buffer();
            }
            return true;
        } else if (c == 'C') { // 右矢印
            // 将来の拡張用（カーソル移動）
            return true;
        } else if (c == 'D') { // 左矢印
            // 将来の拡張用（カーソル移動）
            return true;
        }
        
        return true;
    }
    
    return false;
}

void cli_init(void)
{
    ESPSerial.printf("\r\n=== StampFly CLI Terminal ===\r\n");
    ESPSerial.printf("コマンド一覧は 'help' を入力してください\r\n");
    ESPSerial.printf("矢印キー↑↓でコマンド履歴を使用できます\r\n");
    ESPSerial.print("StampFly> ");
    cli_buffer_index = 0;
    streaming_enabled = false;
    
    // 履歴を初期化
    history_count = 0;
    current_history_pos = -1;
    memset(command_history, 0, sizeof(command_history));
    memset(temp_buffer, 0, sizeof(temp_buffer));
}

void output_stream_data()
{
    switch (stream_format) {
        case STREAM_FORMAT_DEFAULT:
            ESPSerial.printf("STREAM: ");
            ESPSerial.printf("IMU[ax:%.3f,ay:%.3f,az:%.3f,gx:%.3f,gy:%.3f,gz:%.3f] ",
                           Accel_x, Accel_y, Accel_z, Roll_rate, Pitch_rate, Yaw_rate);
            ESPSerial.printf("ATT[r:%.2f,p:%.2f,y:%.2f] ",
                           Roll_angle*57.3, Pitch_angle*57.3, Yaw_angle*57.3);
            ESPSerial.printf("MAG[x:%.2f,y:%.2f,z:%.2f] ", Mx, My, Mz);
            ESPSerial.printf("OPT[vx:%.3f,vy:%.3f] ", Velocity_x, Velocity_y);
            ESPSerial.printf("TOF[%dmm] ", Range);
            ESPSerial.printf("VOLT[%.2fV] ", Voltage);
            ESPSerial.printf("ALT[%.3fm]\r\n", Altitude2);
            break;
            
        case STREAM_FORMAT_CSV:
            ESPSerial.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%d,%.2f,%.3f\r\n",
                           Accel_x, Accel_y, Accel_z, Roll_rate, Pitch_rate, Yaw_rate,
                           Roll_angle*57.3, Pitch_angle*57.3, Yaw_angle*57.3,
                           Mx, My, Mz, Velocity_x, Velocity_y, Range, Voltage, Altitude2);
            break;
            
        case STREAM_FORMAT_TSV:
            ESPSerial.printf("%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.3f\t%.3f\t%d\t%.2f\t%.3f\r\n",
                           Accel_x, Accel_y, Accel_z, Roll_rate, Pitch_rate, Yaw_rate,
                           Roll_angle*57.3, Pitch_angle*57.3, Yaw_angle*57.3,
                           Mx, My, Mz, Velocity_x, Velocity_y, Range, Voltage, Altitude2);
            break;
            
        case STREAM_FORMAT_TELEPLOT:
            uint32_t timestamp = millis();
            ESPSerial.printf(">accel_x:%u:%.3f\r\n", timestamp, Accel_x);
            ESPSerial.printf(">accel_y:%u:%.3f\r\n", timestamp, Accel_y);
            ESPSerial.printf(">accel_z:%u:%.3f\r\n", timestamp, Accel_z);
            ESPSerial.printf(">gyro_x:%u:%.3f\r\n", timestamp, Roll_rate);
            ESPSerial.printf(">gyro_y:%u:%.3f\r\n", timestamp, Pitch_rate);
            ESPSerial.printf(">gyro_z:%u:%.3f\r\n", timestamp, Yaw_rate);
            ESPSerial.printf(">roll:%u:%.2f\r\n", timestamp, Roll_angle*57.3);
            ESPSerial.printf(">pitch:%u:%.2f\r\n", timestamp, Pitch_angle*57.3);
            ESPSerial.printf(">yaw:%u:%.2f\r\n", timestamp, Yaw_angle*57.3);
            ESPSerial.printf(">mag_x:%u:%.2f\r\n", timestamp, Mx);
            ESPSerial.printf(">mag_y:%u:%.2f\r\n", timestamp, My);
            ESPSerial.printf(">mag_z:%u:%.2f\r\n", timestamp, Mz);
            ESPSerial.printf(">velocity_x:%u:%.3f\r\n", timestamp, Velocity_x);
            ESPSerial.printf(">velocity_y:%u:%.3f\r\n", timestamp, Velocity_y);
            ESPSerial.printf(">range:%u:%d\r\n", timestamp, Range);
            ESPSerial.printf(">voltage:%u:%.2f\r\n", timestamp, Voltage);
            ESPSerial.printf(">altitude:%u:%.3f\r\n", timestamp, Altitude2);
            break;
    }
}

void cli_process(void)
{
    // オプティカルテストの処理
    if (Optical_test.active) {
        uint32_t current_time = millis();
        
        // テスト終了チェック
        if (current_time >= Optical_test.start_time + Optical_test.duration_ms) {
            // テスト完了
            Optical_test.active = false;
            
            uint32_t actual_duration = current_time - Optical_test.start_time;
            float total_distance = sqrt(Optical_test.total_movement_x * Optical_test.total_movement_x + 
                                      Optical_test.total_movement_y * Optical_test.total_movement_y);
            
            ESPSerial.printf("\r\n=== テスト結果 ===\r\n");
            ESPSerial.printf("実行時間: %d ms\r\n", actual_duration);
            ESPSerial.printf("総読み取り: %d回 (%.1f Hz)\r\n", Optical_test.total_reads, 
                           Optical_test.total_reads * 1000.0f / actual_duration);
            ESPSerial.printf("有効データ: %d回 (%.1f%%)\r\n", Optical_test.valid_reads, 
                           Optical_test.valid_reads * 100.0f / Optical_test.total_reads);
            ESPSerial.printf("無効データ: %d回 (%.1f%%)\r\n", Optical_test.failed_reads, 
                           Optical_test.failed_reads * 100.0f / Optical_test.total_reads);
            ESPSerial.printf("品質不良: %d回 (%.1f%%)\r\n", Optical_test.quality_failed, 
                           Optical_test.quality_failed * 100.0f / Optical_test.total_reads);
            ESPSerial.printf("総移動量: X=%.6f m, Y=%.6f m\r\n", Optical_test.total_movement_x, Optical_test.total_movement_y);
            ESPSerial.printf("総移動距離: %.6f m\r\n", total_distance);
            ESPSerial.printf("平均速度: X=%.3f m/s, Y=%.3f m/s\r\n", 
                           Optical_test.total_movement_x / (actual_duration / 1000.0f), 
                           Optical_test.total_movement_y / (actual_duration / 1000.0f));
            ESPSerial.printf("最大速度: X=%.3f m/s, Y=%.3f m/s\r\n", Optical_test.max_velocity_x, Optical_test.max_velocity_y);
            ESPSerial.printf("使用CPI: %.1f\r\n", calculateCPI(Altitude2));
            ESPSerial.print("StampFly> ");
            return;
        }
        
        #if 0
        // サンプリング処理（5ms間隔）
        if (current_time - Optical_test.last_sample_time >= 5) {
            Optical_test.last_sample_time = current_time;
            
            int16_t dx, dy;
            uint8_t motion_status = readMotionCount(&dx, &dy);
            Optical_test.total_reads++;
            
            if (motion_status == 1) {
                // 有効なデータ
                Optical_test.valid_reads++;
                
                // PMW3901データシート準拠の移動量計算（高さベースCPI使用）
                float movement_x, movement_y;
                calculateMovementFromDelta(-dy, dx, &movement_x, &movement_y, Altitude2); // 軸変換、高度使用
                
                Optical_test.total_movement_x += movement_x;
                Optical_test.total_movement_y += movement_y;
                
                // 速度計算（10ms間隔）
                float velocity_x = movement_x / 0.01f;
                float velocity_y = movement_y / 0.01f;
                
                if (abs(velocity_x) > abs(Optical_test.max_velocity_x)) Optical_test.max_velocity_x = velocity_x;
                if (abs(velocity_y) > abs(Optical_test.max_velocity_y)) Optical_test.max_velocity_y = velocity_y;
                
            } else if (motion_status == 2) {
                // 品質不良
                Optical_test.quality_failed++;
            } else {
                // データなし
                Optical_test.failed_reads++;
            }
            
        }
        #endif
        
        // 1秒ごとに進行状況を表示
        if (current_time - Optical_test.last_print_time >= 1000) {
            Optical_test.last_print_time = current_time;
            uint32_t elapsed = (current_time - Optical_test.start_time) / 1000;
            uint32_t total_duration = Optical_test.duration_ms / 1000;
            ESPSerial.printf("進行: %d/%d秒 (読取: %d, 有効: %d, 不良 %d, 無効 %d)\r\n", 
                           elapsed, total_duration, 
                           Optical_test.total_reads, 
                           Optical_test.valid_reads, 
                           Optical_test.quality_failed, 
                           Optical_test.failed_reads );
        }
        
        // エンター入力でテスト停止
        if (ESPSerial.available()) {
            char c = ESPSerial.read();
            if (c == '\n' || c == '\r') {
                Optical_test.active = false;
                ESPSerial.printf("\r\nオプティカルテスト停止\r\n");
                ESPSerial.print("StampFly> ");
                return;
            }
        }
    }
    
    // オフセット計算の処理
    if (offset_calc.active) {
        uint32_t current_time = millis();
        
        // 5ms間隔でサンプリング
        if (current_time - offset_calc.start_time >= offset_calc.current_samples * 5) {
            sensor_calc_offset_avarage();
            offset_calc.current_samples++;
            
            // 100サンプルごとに進行状況を表示
            if (offset_calc.current_samples % 100 == 0) {
                ESPSerial.printf("進行状況: %d/%d\r\n", offset_calc.current_samples, offset_calc.target_samples);
            }
            
            // 目標サンプル数に達したら完了
            if (offset_calc.current_samples >= offset_calc.target_samples) {
                offset_calc.active = false;
                
                ESPSerial.printf("オフセット計算完了\r\n");
                
                // オフセット結果を表示
                ESPSerial.printf("=== センサーオフセット値 ===\r\n");
                ESPSerial.printf("ジャイロオフセット [rad/s]:\r\n");
                ESPSerial.printf("  Roll:  %.6f\r\n", Roll_rate_offset);
                ESPSerial.printf("  Pitch: %.6f\r\n", Pitch_rate_offset);
                ESPSerial.printf("  Yaw:   %.6f\r\n", Yaw_rate_offset);
                ESPSerial.printf("加速度Zオフセット [G]: %.6f\r\n", Accel_z_offset);
                ESPSerial.printf("オフセット計算回数: %d\r\n", Offset_counter);
                ESPSerial.print("StampFly> ");
                return;
            }
        }
        
        // エンター入力でオフセット計算停止
        if (ESPSerial.available()) {
            char c = ESPSerial.read();
            if (c == '\n' || c == '\r') {
                offset_calc.active = false;
                ESPSerial.printf("\r\nオフセット計算停止\r\n");
                ESPSerial.print("StampFly> ");
                return;
            }
        }
    }
    
    // ストリーミングモードの処理
    if (streaming_enabled) {
        uint32_t current_time = millis();
        if (current_time - last_stream_time >= stream_interval_ms) {
            last_stream_time = current_time;
            output_stream_data();
        }
        
        // エンター入力でストリーミング停止
        if (ESPSerial.available()) {
            char c = ESPSerial.read();
            if (c == '\n' || c == '\r') {
                streaming_enabled = false;
                ESPSerial.printf("\r\nストリーミング停止\r\n");
                ESPSerial.print("StampFly> ");
                return;
            }
        }
    }
    
    // シリアル入力の処理
    while (ESPSerial.available()) {
        char c = ESPSerial.read();
        
        // 矢印キー（エスケープシーケンス）の処理
        if (handle_arrow_keys(c)) {
            continue;
        }
        
        if (c == '\n' || c == '\r') {
            if (cli_buffer_index > 0) {
                cli_buffer[cli_buffer_index] = '\0';
                
                // 改行を明示的に出力
                ESPSerial.printf("\r\n");
                
                // コマンド履歴に追加
                add_to_history(cli_buffer);
                
                cli_execute_command(cli_buffer);
                cli_buffer_index = 0;
                
                // 履歴位置をリセット
                current_history_pos = -1;
                
                if (!streaming_enabled) {
                    ESPSerial.print("StampFly> ");
                }
            } else {
                // 空のコマンドの場合は改行してプロンプトを表示
                ESPSerial.printf("\r\n");
                if (!streaming_enabled) {
                    ESPSerial.print("StampFly> ");
                }
            }
            
            // \r\n の場合、次の文字が対になる改行文字なら1文字読み飛ばす
            if (ESPSerial.available()) {
                char next_char = ESPSerial.peek();
                if ((c == '\r' && next_char == '\n') || (c == '\n' && next_char == '\r')) {
                    ESPSerial.read(); // 対になる改行文字を読み飛ばす
                }
            }
        } else if (c == '\b' || c == 127) { // バックスペース
            if (cli_buffer_index > 0) {
                cli_buffer_index--;
                cli_buffer[cli_buffer_index] = '\0';
                ESPSerial.print("\b \b");
                
                // 履歴モードを終了（バックスペース時は画面更新しない）
                if (current_history_pos != -1) {
                    current_history_pos = -1;
                }
            }
        } else if (c >= 32 && c <= 126) { // 印刷可能文字
            if (cli_buffer_index < CLI_BUFFER_SIZE - 1) {
                // 履歴モードを終了
                if (current_history_pos != -1) {
                    current_history_pos = -1;
                    // 編集中のバッファを復元
                    strncpy(cli_buffer, temp_buffer, CLI_BUFFER_SIZE);
                    cli_buffer[CLI_BUFFER_SIZE - 1] = '\0';
                    cli_buffer_index = strlen(cli_buffer);
                    
                    // 画面を更新
                    clear_line();
                    display_current_buffer();
                }
                
                cli_buffer[cli_buffer_index++] = c;
                cli_buffer[cli_buffer_index] = '\0';
                ESPSerial.print(c);
            }
        }
    }
}

void cli_execute_command(char* command_line)
{
    char* argv[CLI_MAX_ARGS];
    int argc = 0;
    
    // コマンドラインを解析
    char* token = strtok(command_line, " \t");
    while (token != NULL && argc < CLI_MAX_ARGS) {
        argv[argc++] = token;
        token = strtok(NULL, " \t");
    }
    
    if (argc == 0) return;
    
    // コマンドを検索して実行
    for (int i = 0; cli_commands[i].command != NULL; i++) {
        if (strcmp(argv[0], cli_commands[i].command) == 0) {
            cli_commands[i].function(argc, argv);
            return;
        }
    }
    
    ESPSerial.printf("不明なコマンド: %s\r\n", argv[0]);
    ESPSerial.printf("'help' でコマンド一覧を表示\r\n");
}

void cmd_help(int argc, char* argv[])
{
    ESPSerial.printf("\r\n=== 利用可能なコマンド ===\r\n");
    for (int i = 0; cli_commands[i].command != NULL; i++) {
        ESPSerial.printf("%-10s : %s\r\n", cli_commands[i].command, cli_commands[i].description);
    }
    ESPSerial.printf("\r\n");
}

void cmd_imu(int argc, char* argv[])
{
    ESPSerial.printf("=== IMU データ ===\r\n");
    ESPSerial.printf("加速度 [G]: X=%.3f, Y=%.3f, Z=%.3f\r\n", Accel_x, Accel_y, Accel_z);
    ESPSerial.printf("角速度 [rad/s]: X=%.3f, Y=%.3f, Z=%.3f\r\n", Roll_rate, Pitch_rate, Yaw_rate);
    ESPSerial.printf("生データ加速度: X=%.3f, Y=%.3f, Z=%.3f\r\n", Accel_x_raw, Accel_y_raw, Accel_z_raw);
    ESPSerial.printf("生データ角速度: X=%.3f, Y=%.3f, Z=%.3f\r\n", Roll_rate_raw, Pitch_rate_raw, Yaw_rate_raw);
}

void cmd_tof(int argc, char* argv[])
{
    ESPSerial.printf("=== ToF センサー ===\r\n");
    ESPSerial.printf("下向き距離: %d mm (生データ: %d mm)\r\n", Range, RawRange);
    ESPSerial.printf("前向き距離: %d mm (生データ: %d mm)\r\n", RangeFront, RawRangeFront);
    ESPSerial.printf("フィルタ済み高度: %.3f m\r\n", Altitude);
    ESPSerial.printf("推定高度: %.3f m\r\n", Altitude2);
    ESPSerial.printf("高度速度: %.3f m/s\r\n", Alt_velocity);
}

void cmd_voltage(int argc, char* argv[])
{
    ESPSerial.printf("=== 電圧情報 ===\r\n");
    ESPSerial.printf("バッテリー電圧: %.2f V\r\n", Voltage);
    if (Under_voltage_flag > 0) {
        ESPSerial.printf("警告: 低電圧検出 (フラグ: %d)\r\n", Under_voltage_flag);
    }
}

void cmd_mag(int argc, char* argv[])
{
    ESPSerial.printf("=== 磁気センサー ===\r\n");
    ESPSerial.printf("磁場 [μT]: X=%.2f, Y=%.2f, Z=%.2f\r\n", Mx, My, Mz);
    ESPSerial.printf("オフセット: X=%.2f, Y=%.2f, Z=%.2f\r\n", Mx0, My0, Mz0);
    ESPSerial.printf("平均値: X=%.2f, Y=%.2f, Z=%.2f\r\n", Mx_ave, My_ave, Mz_ave);
}

void cmd_attitude(int argc, char* argv[])
{
    ESPSerial.printf("=== 姿勢角 ===\r\n");
    ESPSerial.printf("Roll:  %.2f° (%.4f rad)\r\n", Roll_angle * 57.2958, Roll_angle);
    ESPSerial.printf("Pitch: %.2f° (%.4f rad)\r\n", Pitch_angle * 57.2958, Pitch_angle);
    ESPSerial.printf("Yaw:   %.2f° (%.4f rad)\r\n", Yaw_angle * 57.2958, Yaw_angle);
}

void cmd_all_sensors(int argc, char* argv[])
{
    ESPSerial.printf("=== 全センサーデータ ===\r\n");
    
    // IMU
    ESPSerial.printf("IMU - 加速度[G]: X=%.3f, Y=%.3f, Z=%.3f\r\n", Accel_x, Accel_y, Accel_z);
    ESPSerial.printf("IMU - 角速度[rad/s]: X=%.3f, Y=%.3f, Z=%.3f\r\n", Roll_rate, Pitch_rate, Yaw_rate);
    
    // 姿勢角
    ESPSerial.printf("姿勢角[°]: Roll=%.2f, Pitch=%.2f, Yaw=%.2f\r\n", 
                   Roll_angle*57.3, Pitch_angle*57.3, Yaw_angle*57.3);
    
    // ToF
    ESPSerial.printf("ToF - 距離: 下=%dmm, 前=%dmm, 高度=%.3fm\r\n", Range, RangeFront, Altitude2);
    
    // 電圧
    ESPSerial.printf("電圧: %.2fV\r\n", Voltage);
    
    // 磁気センサー
    ESPSerial.printf("磁場[μT]: X=%.2f, Y=%.2f, Z=%.2f\r\n", Mx, My, Mz);
}

void cmd_status(int argc, char* argv[])
{
    ESPSerial.printf("=== システム状態 ===\r\n");
    ESPSerial.printf("動作モード: %d\r\n", Mode);
    ESPSerial.printf("稼働時間: %.2f秒\r\n", millis() / 1000.0);
    ESPSerial.printf("制御周期: %.6f秒\r\n", Control_period);
    ESPSerial.printf("オフセットカウンタ: %d\r\n", Offset_counter);
    
    if (OverG_flag) {
        ESPSerial.printf("警告: 過G検出 (%.2fG)\r\n", Over_g);
    }
    if (Range0flag > 0) {
        ESPSerial.printf("警告: 距離センサー異常 (フラグ: %d)\r\n", Range0flag);
    }
    if (Under_voltage_flag > 0) {
        ESPSerial.printf("警告: 低電圧 (フラグ: %d)\r\n", Under_voltage_flag);
    }
    
    ESPSerial.printf("ストリーミング: %s\r\n", streaming_enabled ? "有効" : "無効");
    if (streaming_enabled) {
        ESPSerial.printf("ストリーミング間隔: %dms\r\n", stream_interval_ms);
    }
}

void cmd_stream(int argc, char* argv[])
{
    if (argc < 2) {
        ESPSerial.printf("使用法: stream [start/stop/format] [interval_ms] [format]\r\n");
        ESPSerial.printf("  start [interval_ms] [format] - ストリーミング開始\r\n");
        ESPSerial.printf("  stop                         - ストリーミング停止\r\n");
        ESPSerial.printf("  format [format_type]         - 出力フォーマット設定\r\n");
        ESPSerial.printf("\r\n");
        ESPSerial.printf("フォーマット:\r\n");
        ESPSerial.printf("  default  - デフォルト形式 (STREAM: IMU[...] ATT[...] ...)\r\n");
        ESPSerial.printf("  csv      - CSV形式 (カンマ区切り)\r\n");
        ESPSerial.printf("  tsv      - TSV形式 (タブ区切り)\r\n");
        ESPSerial.printf("  teleplot - Teleplot形式 (>name:timestamp:value)\r\n");
        ESPSerial.printf("\r\n");
        ESPSerial.printf("現在の状態: %s\r\n", streaming_enabled ? "有効" : "無効");
        if (streaming_enabled) {
            ESPSerial.printf("間隔: %dms\r\n", stream_interval_ms);
        }
        const char* format_names[] = {"default", "csv", "tsv", "teleplot"};
        ESPSerial.printf("出力フォーマット: %s\r\n", format_names[stream_format]);
        ESPSerial.printf("\r\n");
        ESPSerial.printf("データ項目順序 (CSV/TSV):\r\n");
        ESPSerial.printf("ax,ay,az,gx,gy,gz,roll,pitch,yaw,mx,my,mz,vx,vy,range,voltage,altitude\r\n");
        return;
    }
    
    if (strcmp(argv[1], "start") == 0) {
        // 間隔の設定
        if (argc >= 3) {
            int interval = atoi(argv[2]);
            if (interval >= 10 && interval <= 10000) {
                stream_interval_ms = interval;
            } else {
                ESPSerial.printf("間隔は10-10000msの範囲で指定してください\r\n");
                return;
            }
        }
        
        // フォーマットの設定
        if (argc >= 4) {
            if (strcmp(argv[3], "default") == 0) {
                stream_format = STREAM_FORMAT_DEFAULT;
            } else if (strcmp(argv[3], "csv") == 0) {
                stream_format = STREAM_FORMAT_CSV;
                ESPSerial.printf("# CSV Header:\r\n");
                ESPSerial.printf("# ax,ay,az,gx,gy,gz,roll,pitch,yaw,mx,my,mz,range,voltage,altitude\r\n");
            } else if (strcmp(argv[3], "tsv") == 0) {
                stream_format = STREAM_FORMAT_TSV;
                ESPSerial.printf("# TSV Header:\r\n");
                ESPSerial.printf("# ax\tay\taz\tgx\tgy\tgz\troll\tpitch\tyaw\tmx\tmy\tmz\trange\tvoltage\taltitude\r\n");
            } else if (strcmp(argv[3], "teleplot") == 0) {
                stream_format = STREAM_FORMAT_TELEPLOT;
            } else {
                ESPSerial.printf("不明なフォーマット。default/csv/tsv/teleplot を指定してください\r\n");
                return;
            }
        }
        
        streaming_enabled = true;
        last_stream_time = millis();
        const char* format_names[] = {"default", "csv", "tsv", "teleplot"};
        ESPSerial.printf("ストリーミング開始 (間隔: %dms, フォーマット: %s)\r\n", 
                       stream_interval_ms, format_names[stream_format]);
        ESPSerial.printf("停止するにはエンターキーを押してください\r\n");
    } else if (strcmp(argv[1], "stop") == 0) {
        streaming_enabled = false;
        ESPSerial.printf("ストリーミング停止\r\n");
    } else if (strcmp(argv[1], "format") == 0) {
        if (argc < 3) {
            const char* format_names[] = {"default", "csv", "tsv", "teleplot"};
            ESPSerial.printf("現在の出力フォーマット: %s\r\n", format_names[stream_format]);
            ESPSerial.printf("使用法: stream format [default/csv/tsv/teleplot]\r\n");
            return;
        }
        
        if (strcmp(argv[2], "default") == 0) {
            stream_format = STREAM_FORMAT_DEFAULT;
            ESPSerial.printf("出力フォーマットをdefaultに設定しました\r\n");
        } else if (strcmp(argv[2], "csv") == 0) {
            stream_format = STREAM_FORMAT_CSV;
            ESPSerial.printf("出力フォーマットをCSVに設定しました\r\n");
        } else if (strcmp(argv[2], "tsv") == 0) {
            stream_format = STREAM_FORMAT_TSV;
            ESPSerial.printf("出力フォーマットをTSVに設定しました\r\n");
        } else if (strcmp(argv[2], "teleplot") == 0) {
            stream_format = STREAM_FORMAT_TELEPLOT;
            ESPSerial.printf("出力フォーマットをTeleplotに設定しました\r\n");
        } else {
            ESPSerial.printf("不明なフォーマット。default/csv/tsv/teleplot を指定してください\r\n");
        }
    } else {
        ESPSerial.printf("不明なオプション。start/stop/format を指定してください\r\n");
    }
}

void cmd_offset(int argc, char* argv[])
{
    if (argc < 2) {
        ESPSerial.printf("使用法: offset [get/reset/start/stop] [samples]\r\n");
        ESPSerial.printf("  get   - 現在のオフセット値を表示\r\n");
        ESPSerial.printf("  reset - オフセット値をリセット\r\n");
        ESPSerial.printf("  start [samples] - ノンブロッキングオフセット計算を開始\r\n");
        ESPSerial.printf("                    samples: 100-5000 (デフォルト: 800)\r\n");
        ESPSerial.printf("  stop  - 実行中のオフセット計算を停止\r\n");
        return;
    }
    
    if (strcmp(argv[1], "get") == 0) {
        ESPSerial.printf("=== センサーオフセット値 ===\r\n");
        ESPSerial.printf("ジャイロオフセット [rad/s]:\r\n");
        ESPSerial.printf("  Roll:  %.6f\r\n", Roll_rate_offset);
        ESPSerial.printf("  Pitch: %.6f\r\n", Pitch_rate_offset);
        ESPSerial.printf("  Yaw:   %.6f\r\n", Yaw_rate_offset);
        ESPSerial.printf("加速度Zオフセット [G]: %.6f\r\n", Accel_z_offset);
        ESPSerial.printf("オフセット計算回数: %d\r\n", Offset_counter);
    } else if (strcmp(argv[1], "reset") == 0) {
        sensor_reset_offset();
        ESPSerial.printf("センサーオフセットをリセットしました\r\n");
    } else if (strcmp(argv[1], "start") == 0) {
        if (offset_calc.active) {
            ESPSerial.printf("オフセット計算は既に実行中です\r\n");
            ESPSerial.printf("エンターキーで停止してから新しい計算を開始してください\r\n");
            return;
        }
        
        // サンプル数の指定（デフォルト800）
        int samples = 800;
        if (argc >= 3) {
            samples = atoi(argv[2]);
            if (samples < 100 || samples > 5000) {
                ESPSerial.printf("サンプル数は100-5000の範囲で指定してください\r\n");
                return;
            }
        }
        
        sensor_reset_offset();
        
        // ノンブロッキングオフセット計算を開始
        offset_calc.active = true;
        offset_calc.start_time = millis();
        offset_calc.target_samples = samples;
        offset_calc.current_samples = 0;
        
        ESPSerial.printf("ノンブロッキングオフセット計算を開始します (%dサンプル)\r\n", samples);
        ESPSerial.printf("機体を水平に静置してください...\r\n");
        ESPSerial.printf("停止するにはエンターキーを押してください\r\n");
    } else if (strcmp(argv[1], "stop") == 0) {
        if (offset_calc.active) {
            offset_calc.active = false;
            ESPSerial.printf("オフセット計算を停止しました\r\n");
        } else {
            ESPSerial.printf("オフセット計算は実行されていません\r\n");
        }
    } else {
        ESPSerial.printf("不明なオプション。get/reset/start を指定してください\r\n");
    }
}

void cmd_mag_cal(int argc, char* argv[])
{
    if (argc < 2) {
        ESPSerial.printf("使用法: mag_cal [start/stop/reset/save/status]\r\n");
        ESPSerial.printf("  start  - キャリブレーションを開始\r\n");
        ESPSerial.printf("  stop   - キャリブレーションを停止\r\n");
        ESPSerial.printf("  reset  - キャリブレーションパラメータをリセット\r\n");
        ESPSerial.printf("  save   - キャリブレーションパラメータを保存\r\n");
        ESPSerial.printf("  status - キャリブレーション状態を表示\r\n");
        return;
    }
    
    if (strcmp(argv[1], "start") == 0) {
        mag_sensor.setCalibrationMode(true);
        ESPSerial.printf("磁気センサーキャリブレーション開始\r\n");
        ESPSerial.printf("機体をゆっくりと全方向に回転させてください\r\n");
        ESPSerial.printf("300サンプル収集後、自動的に完了します\r\n");
    } else if (strcmp(argv[1], "stop") == 0) {
        mag_sensor.setCalibrationMode(false);
        ESPSerial.printf("磁気センサーキャリブレーション停止\r\n");
    } else if (strcmp(argv[1], "reset") == 0) {
        mag_sensor.resetCalibration();
        ESPSerial.printf("磁気センサーキャリブレーションをリセットしました\r\n");
    } else if (strcmp(argv[1], "save") == 0) {
        mag_sensor.saveCalibration();
        ESPSerial.printf("磁気センサーキャリブレーションを保存しました\r\n");
    } else if (strcmp(argv[1], "status") == 0) {
        ESPSerial.printf("キャリブレーションモード: %s\r\n", 
                       mag_sensor.isCalibrationMode() ? "有効" : "無効");
        ESPSerial.printf("現在の磁気データ:\r\n");
        ESPSerial.printf("  生データ: X=%.2f, Y=%.2f, Z=%.2f μT\r\n", Mx, My, Mz);
        ESPSerial.printf("  平均値:   X=%.2f, Y=%.2f, Z=%.2f μT\r\n", Mx_ave, My_ave, Mz_ave);
        ESPSerial.printf("  オフセット: X=%.2f, Y=%.2f, Z=%.2f μT\r\n", Mx0, My0, Mz0);
    } else {
        ESPSerial.printf("不明なオプション。start/stop/reset/save/status を指定してください\r\n");
    }
}

void cmd_reset(int argc, char* argv[])
{
    if (argc < 2) {
        ESPSerial.printf("使用法: reset [ahrs/sensors/all]\r\n");
        ESPSerial.printf("  ahrs    - AHRS（姿勢推定）をリセット\r\n");
        ESPSerial.printf("  sensors - センサーをリセット\r\n");
        ESPSerial.printf("  all     - 全システムをリセット\r\n");
        return;
    }
    
    if (strcmp(argv[1], "ahrs") == 0) {
        ahrs_reset();
        ESPSerial.printf("AHRSをリセットしました\r\n");
    } else if (strcmp(argv[1], "sensors") == 0) {
        sensor_reset_offset();
        ESPSerial.printf("センサーをリセットしました\r\n");
    } else if (strcmp(argv[1], "all") == 0) {
        ahrs_reset();
        sensor_reset_offset();
        ESPSerial.printf("全システムをリセットしました\r\n");
    } else {
        ESPSerial.printf("不明なオプション。ahrs/sensors/all を指定してください\r\n");
    }
}

void cmd_save(int argc, char* argv[])
{
    if (argc < 2) {
        ESPSerial.printf("使用法: save [offsets/mag_cal/pid/all]\r\n");
        ESPSerial.printf("  offsets - センサーオフセット値を保存\r\n");
        ESPSerial.printf("  mag_cal - 磁気センサーキャリブレーションを保存\r\n");
        ESPSerial.printf("  pid     - PIDゲインを保存\r\n");
        ESPSerial.printf("  all     - 全設定を保存\r\n");
        return;
    }
    
    if (strcmp(argv[1], "offsets") == 0) {
        save_sensor_offsets();
        ESPSerial.printf("センサーオフセット値を保存しました\r\n");
    } else if (strcmp(argv[1], "mag_cal") == 0) {
        mag_sensor.saveCalibration();
        ESPSerial.printf("磁気センサーキャリブレーションを保存しました\r\n");
    } else if (strcmp(argv[1], "pid") == 0) {
        save_pid_gains();
        ESPSerial.printf("PIDゲインを保存しました\r\n");
    } else if (strcmp(argv[1], "all") == 0) {
        save_sensor_offsets();
        mag_sensor.saveCalibration();
        save_pid_gains();
        ESPSerial.printf("全設定を保存しました\r\n");
        ESPSerial.printf("  - センサーオフセット値\r\n");
        ESPSerial.printf("  - 磁気センサーキャリブレーション\r\n");
        ESPSerial.printf("  - PIDゲイン\r\n");
    } else {
        ESPSerial.printf("不明なオプション。offsets/mag_cal/pid/all を指定してください\r\n");
    }
}

void cmd_pid(int argc, char* argv[])
{
    if (argc < 2) {
        ESPSerial.printf("使用法: pid [get/set/save/load/reset] [controller] [param] [value]\r\n");
        ESPSerial.printf("  get [controller]     - PIDゲインを表示\r\n");
        ESPSerial.printf("  set [controller] [param] [value] - PIDゲインを設定\r\n");
        ESPSerial.printf("  save                 - 現在のPIDゲインを保存\r\n");
        ESPSerial.printf("  load                 - 保存されたPIDゲインを読み込み\r\n");
        ESPSerial.printf("  reset                - PIDゲインをデフォルト値にリセット\r\n");
        ESPSerial.printf("\r\n");
        ESPSerial.printf("コントローラー:\r\n");
        ESPSerial.printf("  roll_rate, pitch_rate, yaw_rate\r\n");
        ESPSerial.printf("  roll_angle, pitch_angle\r\n");
        ESPSerial.printf("  altitude\r\n");
        ESPSerial.printf("\r\n");
        ESPSerial.printf("パラメータ: kp, ti, td, eta\r\n");
        return;
    }
    
    if (strcmp(argv[1], "get") == 0) {
        if (argc >= 3) {
            // 特定のコントローラーのゲインを表示
            if (strcmp(argv[2], "roll_rate") == 0) {
                ESPSerial.printf("=== Roll Rate PID ===\r\n");
                ESPSerial.printf("kp=%.3f, ti=%.3f, td=%.3f, eta=%.3f\r\n", 
                               Roll_rate_kp, Roll_rate_ti, Roll_rate_td, Roll_rate_eta);
            } else if (strcmp(argv[2], "pitch_rate") == 0) {
                ESPSerial.printf("=== Pitch Rate PID ===\r\n");
                ESPSerial.printf("kp=%.3f, ti=%.3f, td=%.3f, eta=%.3f\r\n", 
                               Pitch_rate_kp, Pitch_rate_ti, Pitch_rate_td, Pitch_rate_eta);
            } else if (strcmp(argv[2], "yaw_rate") == 0) {
                ESPSerial.printf("=== Yaw Rate PID ===\r\n");
                ESPSerial.printf("kp=%.3f, ti=%.3f, td=%.3f, eta=%.3f\r\n", 
                               Yaw_rate_kp, Yaw_rate_ti, Yaw_rate_td, Yaw_rate_eta);
            } else if (strcmp(argv[2], "roll_angle") == 0) {
                ESPSerial.printf("=== Roll Angle PID ===\r\n");
                ESPSerial.printf("kp=%.3f, ti=%.3f, td=%.3f, eta=%.3f\r\n", 
                               Rall_angle_kp, Rall_angle_ti, Rall_angle_td, Rall_angle_eta);
            } else if (strcmp(argv[2], "pitch_angle") == 0) {
                ESPSerial.printf("=== Pitch Angle PID ===\r\n");
                ESPSerial.printf("kp=%.3f, ti=%.3f, td=%.3f, eta=%.3f\r\n", 
                               Pitch_angle_kp, Pitch_angle_ti, Pitch_angle_td, Pitch_angle_eta);
            } else if (strcmp(argv[2], "altitude") == 0) {
                ESPSerial.printf("=== Altitude PID ===\r\n");
                ESPSerial.printf("kp=%.3f, ti=%.3f, td=%.3f, eta=%.3f\r\n", 
                               alt_kp, alt_ti, alt_td, alt_eta);
            } else {
                ESPSerial.printf("不明なコントローラー\r\n");
            }
        } else {
            // 全PIDゲインを表示
            ESPSerial.printf("=== 全PIDゲイン ===\r\n");
            ESPSerial.printf("Roll Rate:   kp=%.3f, ti=%.3f, td=%.3f, eta=%.3f\r\n", 
                           Roll_rate_kp, Roll_rate_ti, Roll_rate_td, Roll_rate_eta);
            ESPSerial.printf("Pitch Rate:  kp=%.3f, ti=%.3f, td=%.3f, eta=%.3f\r\n", 
                           Pitch_rate_kp, Pitch_rate_ti, Pitch_rate_td, Pitch_rate_eta);
            ESPSerial.printf("Yaw Rate:    kp=%.3f, ti=%.3f, td=%.3f, eta=%.3f\r\n", 
                           Yaw_rate_kp, Yaw_rate_ti, Yaw_rate_td, Yaw_rate_eta);
            ESPSerial.printf("Roll Angle:  kp=%.3f, ti=%.3f, td=%.3f, eta=%.3f\r\n", 
                           Rall_angle_kp, Rall_angle_ti, Rall_angle_td, Rall_angle_eta);
            ESPSerial.printf("Pitch Angle: kp=%.3f, ti=%.3f, td=%.3f, eta=%.3f\r\n", 
                           Pitch_angle_kp, Pitch_angle_ti, Pitch_angle_td, Pitch_angle_eta);
            ESPSerial.printf("Altitude:    kp=%.3f, ti=%.3f, td=%.3f, eta=%.3f\r\n", 
                           alt_kp, alt_ti, alt_td, alt_eta);
        }
    } else if (strcmp(argv[1], "set") == 0) {
        if (argc < 5) {
            ESPSerial.printf("使用法: pid set [controller] [param] [value]\r\n");
            return;
        }
        
        float value = atof(argv[4]);
        bool updated = false;
        
        // Roll Rate
        if (strcmp(argv[2], "roll_rate") == 0) {
            if (strcmp(argv[3], "kp") == 0) { Roll_rate_kp = value; updated = true; }
            else if (strcmp(argv[3], "ti") == 0) { Roll_rate_ti = value; updated = true; }
            else if (strcmp(argv[3], "td") == 0) { Roll_rate_td = value; updated = true; }
            else if (strcmp(argv[3], "eta") == 0) { Roll_rate_eta = value; updated = true; }
        }
        // Pitch Rate
        else if (strcmp(argv[2], "pitch_rate") == 0) {
            if (strcmp(argv[3], "kp") == 0) { Pitch_rate_kp = value; updated = true; }
            else if (strcmp(argv[3], "ti") == 0) { Pitch_rate_ti = value; updated = true; }
            else if (strcmp(argv[3], "td") == 0) { Pitch_rate_td = value; updated = true; }
            else if (strcmp(argv[3], "eta") == 0) { Pitch_rate_eta = value; updated = true; }
        }
        // Yaw Rate
        else if (strcmp(argv[2], "yaw_rate") == 0) {
            if (strcmp(argv[3], "kp") == 0) { Yaw_rate_kp = value; updated = true; }
            else if (strcmp(argv[3], "ti") == 0) { Yaw_rate_ti = value; updated = true; }
            else if (strcmp(argv[3], "td") == 0) { Yaw_rate_td = value; updated = true; }
            else if (strcmp(argv[3], "eta") == 0) { Yaw_rate_eta = value; updated = true; }
        }
        // Roll Angle
        else if (strcmp(argv[2], "roll_angle") == 0) {
            if (strcmp(argv[3], "kp") == 0) { Rall_angle_kp = value; updated = true; }
            else if (strcmp(argv[3], "ti") == 0) { Rall_angle_ti = value; updated = true; }
            else if (strcmp(argv[3], "td") == 0) { Rall_angle_td = value; updated = true; }
            else if (strcmp(argv[3], "eta") == 0) { Rall_angle_eta = value; updated = true; }
        }
        // Pitch Angle
        else if (strcmp(argv[2], "pitch_angle") == 0) {
            if (strcmp(argv[3], "kp") == 0) { Pitch_angle_kp = value; updated = true; }
            else if (strcmp(argv[3], "ti") == 0) { Pitch_angle_ti = value; updated = true; }
            else if (strcmp(argv[3], "td") == 0) { Pitch_angle_td = value; updated = true; }
            else if (strcmp(argv[3], "eta") == 0) { Pitch_angle_eta = value; updated = true; }
        }
        // Altitude
        else if (strcmp(argv[2], "altitude") == 0) {
            if (strcmp(argv[3], "kp") == 0) { alt_kp = value; updated = true; }
            else if (strcmp(argv[3], "ti") == 0) { alt_ti = value; updated = true; }
            else if (strcmp(argv[3], "td") == 0) { alt_td = value; updated = true; }
            else if (strcmp(argv[3], "eta") == 0) { alt_eta = value; updated = true; }
        }
        
        if (updated) {
            update_pid_controllers();
            ESPSerial.printf("%s %s を %.3f に設定しました\r\n", argv[2], argv[3], value);
        } else {
            ESPSerial.printf("不明なコントローラーまたはパラメータ\r\n");
        }
    } else if (strcmp(argv[1], "save") == 0) {
        save_pid_gains();
        ESPSerial.printf("PIDゲインを保存しました\r\n");
    } else if (strcmp(argv[1], "load") == 0) {
        load_pid_gains();
        update_pid_controllers();
        ESPSerial.printf("PIDゲインを読み込みました\r\n");
    } else if (strcmp(argv[1], "reset") == 0) {
        reset_pid_gains_to_default();
        update_pid_controllers();
        ESPSerial.printf("PIDゲインをデフォルト値にリセットしました\r\n");
    } else {
        ESPSerial.printf("不明なオプション。get/set/save/load/reset を指定してください\r\n");
    }
}

void cmd_load(int argc, char* argv[])
{
    if (argc < 2) {
        ESPSerial.printf("使用法: load [offsets/mag_cal/pid/all]\r\n");
        ESPSerial.printf("  offsets - センサーオフセット値を読み込み\r\n");
        ESPSerial.printf("  mag_cal - 磁気センサーキャリブレーションを読み込み\r\n");
        ESPSerial.printf("  pid     - PIDゲインを読み込み\r\n");
        ESPSerial.printf("  all     - 全設定を読み込み\r\n");
        return;
    }
    
    if (strcmp(argv[1], "offsets") == 0) {
        load_sensor_offsets();
        ESPSerial.printf("センサーオフセット値を読み込みました\r\n");
    } else if (strcmp(argv[1], "mag_cal") == 0) {
        mag_sensor.loadCalibration();
        ESPSerial.printf("磁気センサーキャリブレーションを読み込みました\r\n");
    } else if (strcmp(argv[1], "pid") == 0) {
        load_pid_gains();
        update_pid_controllers();
        ESPSerial.printf("PIDゲインを読み込みました\r\n");
    } else if (strcmp(argv[1], "all") == 0) {
        load_sensor_offsets();
        mag_sensor.loadCalibration();
        load_pid_gains();
        update_pid_controllers();
        ESPSerial.printf("全設定を読み込みました\r\n");
        ESPSerial.printf("  - センサーオフセット値\r\n");
        ESPSerial.printf("  - 磁気センサーキャリブレーション\r\n");
        ESPSerial.printf("  - PIDゲイン\r\n");
    } else {
        ESPSerial.printf("不明なオプション。offsets/mag_cal/pid/all を指定してください\r\n");
    }
}

void cmd_optical(int argc, char* argv[])
{
    if (argc >= 2) {
        // テスト期間を指定された場合
        if (strcmp(argv[1], "stop") == 0) {
            // テスト停止
            if (Optical_test.active) {
                Optical_test.active = false;
                ESPSerial.printf("オプティカルテストを停止しました\r\n");
            } else {
                ESPSerial.printf("オプティカルテストは実行されていません\r\n");
            }
            return;
        }
        
        int duration_sec = atoi(argv[1]);
        if (duration_sec < 1 || duration_sec > 60) {
            ESPSerial.printf("テスト期間は1-60秒の範囲で指定してください\r\n");
            ESPSerial.printf("または 'optical stop' でテストを停止\r\n");
            return;
        }
        
        if (Optical_test.active) {
            ESPSerial.printf("オプティカルテストは既に実行中です\r\n");
            ESPSerial.printf("'optical stop' で停止してから新しいテストを開始してください\r\n");
            return;
        }
        
        // ノンブロッキングテストを開始
        Optical_test.active = true;
        Optical_test.start_time = millis();
        Optical_test.duration_ms = duration_sec * 1000;
        Optical_test.last_print_time = Optical_test.start_time;
        Optical_test.last_sample_time = Optical_test.start_time;
        Optical_test.total_reads = 0;
        Optical_test.valid_reads = 0;
        Optical_test.failed_reads = 0;
        Optical_test.quality_failed = 0;
        Optical_test.total_movement_x = 0.0f;
        Optical_test.total_movement_y = 0.0f;
        Optical_test.max_velocity_x = 0.0f;
        Optical_test.max_velocity_y = 0.0f;
        
        ESPSerial.printf("=== PMW3901 実測テスト (%d秒) ===\r\n", duration_sec);
        ESPSerial.printf("現在のCPI: %.1f (高度: %.3fm)\r\n", calculateCPI(Altitude2), Altitude2);
        ESPSerial.printf("ノンブロッキングテスト開始...\r\n");
        ESPSerial.printf("停止するには 'optical stop' またはエンターキーを押してください\r\n");
        
    } else {
        // 現在の値を表示
        ESPSerial.printf("=== オプティカルフロー ===\r\n");
        ESPSerial.printf("移動量 [m]: X=%.6f, Y=%.6f\r\n", Optical_flow_x, Optical_flow_y);
        ESPSerial.printf("速度 [m/s]: X=%.3f, Y=%.3f\r\n", Velocity_x, Velocity_y);
        ESPSerial.printf("生Delta値: X=%d, Y=%d\r\n", deltaX, deltaY);
        ESPSerial.printf("現在の高度: %.3f m\r\n", Altitude2);
        ESPSerial.printf("計算CPI: %.1f\r\n", calculateCPI(Altitude2));
        ESPSerial.printf("\r\n");
        ESPSerial.printf("使用法:\r\n");
        ESPSerial.printf("  optical [duration_sec] - 指定秒数のノンブロッキングテスト実行\r\n");
        ESPSerial.printf("  optical stop           - 実行中のテストを停止\r\n");
        
        if (Optical_test.active) {
            uint32_t elapsed = (millis() - Optical_test.start_time) / 1000;
            uint32_t total_duration = Optical_test.duration_ms / 1000;
            ESPSerial.printf("現在テスト実行中: %d/%d秒経過\r\n", elapsed, total_duration);
        }
    }
}

void cmd_history(int argc, char* argv[])
{
    ESPSerial.printf("=== コマンド履歴 ===\r\n");
    if (history_count == 0) {
        ESPSerial.printf("履歴がありません\r\n");
        return;
    }
    
    int start_index = (history_count >= CLI_HISTORY_SIZE) ? 
                     (history_count % CLI_HISTORY_SIZE) : 0;
    int display_count = (history_count >= CLI_HISTORY_SIZE) ? 
                       CLI_HISTORY_SIZE : history_count;
    
    for (int i = 0; i < display_count; i++) {
        int index = (start_index + i) % CLI_HISTORY_SIZE;
        ESPSerial.printf("%2d: %s\r\n", i + 1, command_history[index]);
    }
    ESPSerial.printf("矢印キー↑↓で履歴を呼び出せます\r\n");
}
