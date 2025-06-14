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

#ifndef WRAPPER_HPP
#define WRAPPER_HPP

#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_log.h>

// Serial wrapper
extern decltype(Serial)& ESPSerial;

// GPIO wrapper functions
void wrapper_pinMode(gpio_num_t pin, gpio_mode_t mode);
void wrapper_digitalWrite(int pin, uint32_t val);
int wrapper_digitalRead(gpio_num_t pin);
void wrapper_attachInterrupt(gpio_num_t pin, void (*isr)(void), gpio_int_type_t mode);
void wrapper_detachInterrupt(gpio_num_t pin);

// Serial wrapper functions
void wrapper_serial_begin(unsigned long baud);
void wrapper_serial_printf(const char* format, ...);
void wrapper_serial_println(const char* str);
void wrapper_serial_print(const char* str);

// Delay wrapper functions
void wrapper_delay(uint32_t ms);
void wrapper_delayMicroseconds(uint32_t us);

// Time wrapper functions
uint32_t wrapper_millis(void);
uint32_t wrapper_micros(void);

// GPIO mode definitions (avoiding conflicts with Arduino)
#define WRAPPER_OUTPUT GPIO_MODE_OUTPUT
#define WRAPPER_INPUT GPIO_MODE_INPUT
#define WRAPPER_INPUT_PULLUP GPIO_MODE_INPUT
#define WRAPPER_HIGH 1
#define WRAPPER_LOW 0
#define WRAPPER_FALLING GPIO_INTR_NEGEDGE
#define WRAPPER_RISING GPIO_INTR_POSEDGE
#define WRAPPER_CHANGE GPIO_INTR_ANYEDGE

#endif // WRAPPER_HPP
