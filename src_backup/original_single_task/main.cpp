#include "flight_control.hpp"
#include "wrapper.hpp"
#include "multitask_data.hpp"
#include "multitask_functions.hpp"
#include "multitask_debug.hpp"
#include "esp_log.h"

void setup() {  
  // Serial初期化
  ESPSerial.begin(115200);
  
  // ESP-IDFのログ出力設定
  esp_log_level_set("*", ESP_LOG_VERBOSE);
  
  ESPSerial.printf("Starting StampFly Multitask System!\r\n");
  
  // 従来の初期化処理
  init_copter();
  
  // マルチタスクデータ構造初期化
  if (multitask_init() != ESP_OK) {
    ESPSerial.printf("Failed to initialize multitask data structures\r\n");
    while(1) {
      delay(1000);
    }
  }
  
  // デバッグシステム初期化
  multitask_debug_init();
  
  // 全タスク作成
  if (create_all_tasks() != ESP_OK) {
    ESPSerial.printf("Failed to create tasks\r\n");
    multitask_cleanup();
    while(1) {
      delay(1000);
    }
  }
  
  ESPSerial.printf("Multitask system initialized successfully!\r\n");
  ESPSerial.printf("FreeRTOS scheduler will start...\r\n");
  
  delay(100);
}

void loop() {
  // FreeRTOSマルチタスクシステムでは、loop()は使用しない
  // 全ての処理はタスクで実行される
  vTaskDelete(NULL); // このタスク（Arduino loop task）を削除
}
