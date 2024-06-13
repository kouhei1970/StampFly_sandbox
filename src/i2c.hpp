#ifndef I2C_HPP
#define I2C_HPP

#include <Arduino.h>
#include <Wire.h>

#define I2C_MASTER_SDA_IO 3
#define I2C_MASTER_SCL_IO 4
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

esp_err_t i2c_master_init(void);
uint8_t i2c_scan(void);




#endif