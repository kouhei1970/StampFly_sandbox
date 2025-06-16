#ifndef OPT_HPP
#define OPT_HPP

/*! CPP guard */
//#ifdef __cplusplus
//extern "C" {
//#endif

#include <Arduino.h>
#include <stdint.h>

typedef struct {
    uint8_t chipid;
    uint8_t dipihc;
} optconfig_t;

// オプティカルフローデータ構造体
typedef struct {
    int16_t deltaX;
    int16_t deltaY;
    bool valid;        // データの有効性
    uint32_t timestamp; // タイムスタンプ（マイクロ秒）
    uint8_t motion_reg; // モーションレジスタの生値
} optical_flow_data_t;

// オプティカルテスト用の状態管理
typedef struct {
    bool active;
    uint32_t start_time;
    uint32_t duration_ms;
    uint32_t last_print_time;
    uint32_t last_sample_time;
    int total_reads;
    int valid_reads;
    int failed_reads;
    int quality_failed;
    float total_movement_x;
    float total_movement_y;
    float max_velocity_x;
    float max_velocity_y;
} optical_test_t;

extern optconfig_t optconfig;

uint8_t powerUp(optconfig_t* optconfig);
void initRegisters(void);
void registerWrite(uint8_t reg, uint8_t value);
uint8_t registerRead(uint8_t reg);

// PMW3901データシート準拠のモーションデータ読み取り（品質チェック付き）
uint8_t readMotionCount(int16_t *deltaX, int16_t *deltaY);

// PMW3901のCPI計算と移動量計算
float calculateCPI(float height_m);
void calculateMovementFromDelta(int16_t deltaX, int16_t deltaY, float *movement_x, float *movement_y, float height_m);

void enableFrameCaptureMode(void);
void readImage(uint8_t *image);


//#ifdef __cplusplus
//}
//#endif /* End of CPP guard */
#endif
