#ifndef SERIAL_WRAPPER_HPP
#define SERIAL_WRAPPER_HPP

#include <driver/uart.h>
#include <esp_log.h>
#include <stdarg.h>
#include <stdio.h>

class SerialWrapper {
private:
    uart_port_t uart_port;
    bool initialized;
    bool use_usb_cdc;

public:
    SerialWrapper(uart_port_t port = UART_NUM_0);
    
    void begin(uint32_t baud_rate);
    void end();
    
    void print(const char* str);
    void println(const char* str);
    void printf(const char* format, ...);
    
    size_t write(const uint8_t* buffer, size_t size);
    size_t write(uint8_t data);
    
    int available();
    int read();
    size_t readBytes(uint8_t* buffer, size_t length);
    
    void flush();
};

// グローバルインスタンス
extern SerialWrapper StampFlySerial;

#endif // SERIAL_WRAPPER_HPP
