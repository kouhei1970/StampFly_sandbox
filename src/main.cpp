#include "flight_control.hpp"
#include "wrapper.hpp"
#include "esp_log.h"

void setup() {  
  // Serial初期化
  ESPSerial.begin(115200);
  
  // ESP-IDFのログ出力設定
  esp_log_level_set("*", ESP_LOG_VERBOSE);
  
  init_copter();
  delay(100);
}

void loop() {
  loop_400Hz();
}
