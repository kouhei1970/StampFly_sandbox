#ifndef I2C_HPP
#define I2C_HPP

#include <driver/i2c.h>
#include <esp_err.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SDA_IO           3
#define I2C_MASTER_SCL_IO           4
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

esp_err_t i2c_master_init(void);
esp_err_t i2c_master_deinit(void);

// 汎用I2C送信・受信関数
esp_err_t i2c_write_bytes(uint8_t dev_addr, const uint8_t *data, size_t len);
esp_err_t i2c_read_bytes(uint8_t dev_addr, uint8_t *data, size_t len);

// レジスタ指定付き書き込み（8bitアドレス）
esp_err_t i2c_write_reg8(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, size_t len);
esp_err_t i2c_read_reg8(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);

// レジスタ指定付き書き込み（16bitアドレス）
esp_err_t i2c_write_reg16(uint8_t dev_addr, uint16_t reg_addr, const uint8_t *data, size_t len);
esp_err_t i2c_read_reg16(uint8_t dev_addr, uint16_t reg_addr, uint8_t *data, size_t len);

// スキャン
uint8_t i2c_scan(void);

#ifdef __cplusplus
}
#endif

#endif // I2C_HPP
