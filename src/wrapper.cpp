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

#include "wrapper.hpp"
#include <stdarg.h>

// Serial wrapper
decltype(Serial)& ESPSerial = Serial;

// GPIO wrapper functions
void wrapper_pinMode(gpio_num_t pin, gpio_mode_t mode) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = mode;
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = (mode == GPIO_MODE_INPUT) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
}

void wrapper_digitalWrite(int pin, uint32_t val) {
    gpio_set_level((gpio_num_t)pin, val);
}

int wrapper_digitalRead(gpio_num_t pin) {
    return gpio_get_level(pin);
}

void wrapper_attachInterrupt(gpio_num_t pin, void (*isr)(void), gpio_int_type_t mode) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = mode;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    
    gpio_install_isr_service(0);
    gpio_isr_handler_add(pin, (gpio_isr_t)isr, NULL);
}

void wrapper_detachInterrupt(gpio_num_t pin) {
    gpio_isr_handler_remove(pin);
}

// Serial wrapper functions
void wrapper_serial_begin(unsigned long baud) {
    ESPSerial.begin(baud);
}

void wrapper_serial_printf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    ESPSerial.printf(format, args);
    va_end(args);
}

void wrapper_serial_println(const char* str) {
    ESPSerial.println(str);
}

void wrapper_serial_print(const char* str) {
    ESPSerial.print(str);
}

// Delay wrapper functions
void wrapper_delay(uint32_t ms) {
    delay(ms);
}

void wrapper_delayMicroseconds(uint32_t us) {
    delayMicroseconds(us);
}

// Time wrapper functions
uint32_t wrapper_millis(void) {
    return millis();
}

uint32_t wrapper_micros(void) {
    return micros();
}
