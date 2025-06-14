#include <Arduino.h>
#include <FastLED.h>
#include "flight_control.hpp"
#include "esp_log.h"

// ESP-IDFのログ出力をシリアルに転送する関数
static int esp_log_to_serial(const char *fmt, va_list args) {
  char buf[512];
  int ret = vsnprintf(buf, sizeof(buf), fmt, args);
  USBSerial.print(buf);
  return ret;
}

void setup() {  
  // ESP-IDFのログ出力をUSBSerialに転送する設定
  esp_log_level_set("*", ESP_LOG_VERBOSE);
  esp_log_set_vprintf(esp_log_to_serial);
  
  init_copter();
  delay(100);
}

void loop() {
  loop_400Hz();
}
