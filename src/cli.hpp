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

#ifndef CLI_HPP
#define CLI_HPP

#include <Arduino.h>
#include "wrapper.hpp"

// CLI設定
#define CLI_BUFFER_SIZE 128
#define CLI_MAX_ARGS 10
#define CLI_HISTORY_SIZE 10

// ANSI エスケープシーケンス
#define ESC_SEQ_UP    "\x1B[A"
#define ESC_SEQ_DOWN  "\x1B[A"
#define ESC_SEQ_LEFT  "\x1B[D"
#define ESC_SEQ_RIGHT "\x1B[C"

// CLIコマンド構造体
typedef struct {
    const char* command;
    void (*function)(int argc, char* argv[]);
    const char* description;
} cli_command_t;

// CLI関数宣言
void cli_init(void);
void cli_process(void);
void cli_execute_command(char* command_line);
void cli_print_help(void);

// センサーコマンド関数
void cmd_imu(int argc, char* argv[]);
void cmd_tof(int argc, char* argv[]);
void cmd_voltage(int argc, char* argv[]);
void cmd_mag(int argc, char* argv[]);
void cmd_optical(int argc, char* argv[]);
void cmd_all_sensors(int argc, char* argv[]);
void cmd_attitude(int argc, char* argv[]);
void cmd_help(int argc, char* argv[]);
void cmd_status(int argc, char* argv[]);
void cmd_stream(int argc, char* argv[]);
void cmd_offset(int argc, char* argv[]);
void cmd_mag_cal(int argc, char* argv[]);
void cmd_reset(int argc, char* argv[]);
void cmd_save(int argc, char* argv[]);
void cmd_load(int argc, char* argv[]);
void cmd_pid(int argc, char* argv[]);
void cmd_history(int argc, char* argv[]);

// ストリーミング制御
extern volatile bool streaming_enabled;
extern volatile uint32_t stream_interval_ms;

#endif // CLI_HPP
