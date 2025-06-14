#include "serial_wrapper.hpp"
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

// ArduinoフレームワークのUSB-CDC機能を使用
#include <Arduino.h>

// グローバルインスタンス
SerialWrapper StampFlySerial(UART_NUM_0);

SerialWrapper::SerialWrapper(uart_port_t port) : uart_port(port), initialized(false), use_usb_cdc(true) {
}

void SerialWrapper::begin(uint32_t baud_rate) {
    if (initialized) {
        return;
    }
    
    // ESP32-S3ではUSB-CDCを優先的に使用
    if (use_usb_cdc) {
        // ArduinoのUSB-CDCシリアルを初期化
        Serial.begin(baud_rate);
        // USB-CDCが利用可能になるまで少し待つ
        delay(100);
        initialized = true;
        return;
    }
    
    // フォールバック: UART使用
    uart_config_t uart_config = {
        .baud_rate = (int)baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };
    
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_port, &uart_config));
    
    // Set UART pins (using default pins for UART0: TX=1, RX=3)
    ESP_ERROR_CHECK(uart_set_pin(uart_port, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(uart_port, 1024, 1024, 0, NULL, 0));
    
    initialized = true;
}

void SerialWrapper::end() {
    if (initialized) {
        if (use_usb_cdc) {
            Serial.end();
        } else {
            uart_driver_delete(uart_port);
        }
        initialized = false;
    }
}

void SerialWrapper::print(const char* str) {
    if (!initialized || !str) {
        return;
    }
    
    if (use_usb_cdc) {
        Serial.print(str);
    } else {
        uart_write_bytes(uart_port, str, strlen(str));
    }
}

void SerialWrapper::println(const char* str) {
    if (!initialized) {
        return;
    }
    
    if (use_usb_cdc) {
        if (str) {
            Serial.println(str);
        } else {
            Serial.println();
        }
    } else {
        if (str) {
            uart_write_bytes(uart_port, str, strlen(str));
        }
        uart_write_bytes(uart_port, "\r\n", 2);
    }
}

void SerialWrapper::printf(const char* format, ...) {
    if (!initialized || !format) {
        return;
    }
    
    char buffer[512];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (len > 0 && len < (int)sizeof(buffer)) {
        if (use_usb_cdc) {
            Serial.print(buffer);
        } else {
            uart_write_bytes(uart_port, buffer, len);
        }
    }
}

size_t SerialWrapper::write(const uint8_t* buffer, size_t size) {
    if (!initialized || !buffer || size == 0) {
        return 0;
    }
    
    if (use_usb_cdc) {
        return Serial.write(buffer, size);
    } else {
        int written = uart_write_bytes(uart_port, buffer, size);
        return (written >= 0) ? written : 0;
    }
}

size_t SerialWrapper::write(uint8_t data) {
    return write(&data, 1);
}

int SerialWrapper::available() {
    if (!initialized) {
        return 0;
    }
    
    if (use_usb_cdc) {
        return Serial.available();
    } else {
        size_t available_bytes = 0;
        uart_get_buffered_data_len(uart_port, &available_bytes);
        return (int)available_bytes;
    }
}

int SerialWrapper::read() {
    if (!initialized) {
        return -1;
    }
    
    if (use_usb_cdc) {
        return Serial.read();
    } else {
        uint8_t data;
        int len = uart_read_bytes(uart_port, &data, 1, 0);
        return (len > 0) ? data : -1;
    }
}

size_t SerialWrapper::readBytes(uint8_t* buffer, size_t length) {
    if (!initialized || !buffer || length == 0) {
        return 0;
    }
    
    if (use_usb_cdc) {
        return Serial.readBytes(buffer, length);
    } else {
        int len = uart_read_bytes(uart_port, buffer, length, pdMS_TO_TICKS(1000));
        return (len >= 0) ? len : 0;
    }
}

void SerialWrapper::flush() {
    if (initialized) {
        if (use_usb_cdc) {
            Serial.flush();
        } else {
            uart_wait_tx_done(uart_port, pdMS_TO_TICKS(1000));
        }
    }
}
