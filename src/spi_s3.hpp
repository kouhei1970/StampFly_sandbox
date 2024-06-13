#ifndef SPIS3_H
#define SPIS3_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif


#include <driver/spi_master.h>
#include "bmi2.h"
#include "bmi2_defs.h"


#define PIN_NUM_MISO (43)
#define PIN_NUM_MOSI (14)
#define PIN_NUM_CLK (44)
#define BMI_CS (46)
#define PMW_CS (12)

//SPIバスの設定
extern spi_bus_config_t buscfg;
// SPIデバイスの設定
extern spi_device_interface_config_t devcfg[2];
// SPIデバイスハンドラーを使って通信する
extern spi_device_handle_t bmi;
extern spi_device_handle_t pmw;

esp_err_t spi_init(void);
int8_t bmi2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t bmi2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif
