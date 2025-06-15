#include "vl53lx_platform.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_TIMEOUT_MS 1000

static const char *TAG = "VL53LX";

VL53LX_Error VL53LX_CommsInitialise(VL53LX_Dev_t *pdev, uint8_t comms_type, uint16_t comms_speed_khz) {
    ESP_LOGI(TAG, "I2C Comms Initialized: Addr=0x%02X, Speed=%dkHz", pdev->i2c_slave_address, comms_speed_khz);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_CommsClose(VL53LX_Dev_t *pdev) {
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WriteMulti(VL53LX_Dev_t *pdev, uint16_t index, uint8_t *pdata, uint32_t count) {
    uint8_t buf[2 + count];
    buf[0] = (index >> 8) & 0xFF;
    buf[1] = index & 0xFF;
    memcpy(&buf[2], pdata, count);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (pdev->i2c_slave_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buf, sizeof(buf), true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK) ? VL53LX_ERROR_NONE : VL53LX_ERROR_CONTROL_INTERFACE;
}

VL53LX_Error VL53LX_ReadMulti(VL53LX_Dev_t *pdev, uint16_t index, uint8_t *pdata, uint32_t count) {
    uint8_t addr_buf[2] = {(index >> 8) & 0xFF, index & 0xFF};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (pdev->i2c_slave_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, addr_buf, 2, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (pdev->i2c_slave_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, pdata, count, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK) ? VL53LX_ERROR_NONE : VL53LX_ERROR_CONTROL_INTERFACE;
}

VL53LX_Error VL53LX_WrByte(VL53LX_Dev_t *pdev, uint16_t index, uint8_t data) {
    return VL53LX_WriteMulti(pdev, index, &data, 1);
}

VL53LX_Error VL53LX_WrWord(VL53LX_Dev_t *pdev, uint16_t index, uint16_t data) {
    uint8_t buf[2] = {data >> 8, data & 0xFF};
    return VL53LX_WriteMulti(pdev, index, buf, 2);
}

VL53LX_Error VL53LX_WrDWord(VL53LX_Dev_t *pdev, uint16_t index, uint32_t data) {
    uint8_t buf[4] = {
        (data >> 24) & 0xFF,
        (data >> 16) & 0xFF,
        (data >> 8) & 0xFF,
        data & 0xFF
    };
    return VL53LX_WriteMulti(pdev, index, buf, 4);
}

VL53LX_Error VL53LX_RdByte(VL53LX_Dev_t *pdev, uint16_t index, uint8_t *pdata) {
    return VL53LX_ReadMulti(pdev, index, pdata, 1);
}

VL53LX_Error VL53LX_RdWord(VL53LX_Dev_t *pdev, uint16_t index, uint16_t *pdata) {
    uint8_t buf[2];
    VL53LX_Error status = VL53LX_ReadMulti(pdev, index, buf, 2);
    *pdata = (buf[0] << 8) | buf[1];
    return status;
}

VL53LX_Error VL53LX_RdDWord(VL53LX_Dev_t *pdev, uint16_t index, uint32_t *pdata) {
    uint8_t buf[4];
    VL53LX_Error status = VL53LX_ReadMulti(pdev, index, buf, 4);
    *pdata = ((uint32_t)buf[0] << 24) |
             ((uint32_t)buf[1] << 16) |
             ((uint32_t)buf[2] << 8) |
             (uint32_t)buf[3];
    return status;
}

VL53LX_Error VL53LX_WaitUs(VL53LX_Dev_t *pdev, int32_t wait_us) {
    ets_delay_us(wait_us);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitMs(VL53LX_Dev_t *pdev, int32_t wait_ms) {
    vTaskDelay(pdMS_TO_TICKS(wait_ms));
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTickCount(VL53LX_Dev_t *pdev, uint32_t *ptime_ms) {
    *ptime_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitValueMaskEx(
    VL53LX_Dev_t *pdev,
    uint32_t timeout_ms,
    uint16_t index,
    uint8_t value,
    uint8_t mask,
    uint32_t poll_delay_ms
) {
    uint8_t data = 0;
    uint32_t elapsed = 0;
    while (elapsed < timeout_ms) {
        if (VL53LX_RdByte(pdev, index, &data) != VL53LX_ERROR_NONE) {
            return VL53LX_ERROR_CONTROL_INTERFACE;
        }
        if ((data & mask) == (value & mask)) {
            return VL53LX_ERROR_NONE;
        }
        vTaskDelay(pdMS_TO_TICKS(poll_delay_ms));
        elapsed += poll_delay_ms;
    }
    return VL53LX_ERROR_TIME_OUT;
}

VL53LX_Error VL53LX_GetTimerFrequency(int32_t *ptimer_freq_hz) {
    *ptimer_freq_hz = 1000;
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTimerValue(int32_t *ptimer_count) {
    *ptimer_count = xTaskGetTickCount();
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioSetMode(uint8_t pin, uint8_t mode) {
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        .mode = (mode == 0) ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    return gpio_config(&cfg) == ESP_OK ? VL53LX_ERROR_NONE : VL53LX_ERROR_GPIO_NOT_EXISTING;
}

VL53LX_Error VL53LX_GpioSetValue(uint8_t pin, uint8_t value) {
    gpio_set_level(pin, value);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioGetValue(uint8_t pin, uint8_t *pvalue) {
    *pvalue = gpio_get_level(pin);
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioXshutdown(uint8_t value) {
    // 非推奨: 新設計では pdev を使った別関数を推奨
    return VL53LX_ERROR_NOT_IMPLEMENTED;
}

VL53LX_Error VL53LX_GpioCommsSelect(uint8_t value) {
    return VL53LX_ERROR_NOT_IMPLEMENTED;
}

VL53LX_Error VL53LX_GpioPowerEnable(uint8_t value) {
    return VL53LX_ERROR_NOT_IMPLEMENTED;
}

static void (*g_user_isr_cb)(void) = NULL;

static void gpio_int_isr_handler(void *arg) {
    if (g_user_isr_cb) g_user_isr_cb();
}

VL53LX_Error VL53LX_GpioInterruptEnable(void (*function)(void), uint8_t edge_type) {
    g_user_isr_cb = function;
    gpio_install_isr_service(0);
    // 実際にはどの GPIO に割り込みをかけるかはアプリ側で add してください
    return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioInterruptDisable(void) {
    gpio_uninstall_isr_service();
    g_user_isr_cb = NULL;
    return VL53LX_ERROR_NONE;
}
