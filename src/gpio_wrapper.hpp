#ifndef GPIO_WRAPPER_HPP
#define GPIO_WRAPPER_HPP

#include <driver/gpio.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Arduino互換の定数定義（Arduino.hがインクルードされていない場合のみ）
#ifndef ARDUINO_H
#define OUTPUT GPIO_MODE_OUTPUT
#define INPUT GPIO_MODE_INPUT
#define INPUT_PULLUP GPIO_MODE_INPUT
#define HIGH 1
#define LOW 0
#define FALLING GPIO_INTR_NEGEDGE
#define RISING GPIO_INTR_POSEDGE
#define CHANGE GPIO_INTR_ANYEDGE
#endif

// Arduino互換の関数宣言
void pinMode(gpio_num_t pin, gpio_mode_t mode);
void digitalWrite(gpio_num_t pin, uint32_t val);
int digitalRead(gpio_num_t pin);
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);

// micros()関数はArduinoフレームワークと競合するため定義しない
// Arduinoのmicros()を使用する

// 割り込み関連
typedef void (*interrupt_handler_t)(void);
void attachInterrupt(gpio_num_t pin, interrupt_handler_t handler, gpio_int_type_t mode);
void detachInterrupt(gpio_num_t pin);

#endif // GPIO_WRAPPER_HPP
