#include <Arduino.h> 
#include "spi_s3.hpp"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
//#include "driver/gpio.h"
//#include "sdkconfig.h"

uint8_t Buffer[256];


//SPIバスの設定
spi_bus_config_t buscfg = {
    .mosi_io_num = PIN_NUM_MOSI,
    .miso_io_num = PIN_NUM_MISO,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4096*2,
};

// SPIデバイスの設定
spi_device_interface_config_t devcfg_bmi = {
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .mode = 0,
    .duty_cycle_pos = 128,  // default 128 = 50%/50% duty
    .cs_ena_pretrans = 0, // 0 not used
    .cs_ena_posttrans = 0,  // 0 not used
    .clock_speed_hz = SPI_MASTER_FREQ_8M,// 8,9,10,11,13,16,20,26,40,80
    .spics_io_num = BMI_CS,
    .flags = 0,  // 0 not used
    .queue_size = 10,// transactionのキュー数。1以上の値を入れておく。
    .pre_cb = NULL,// transactionが始まる前に呼ばれる関数をセットできる
    .post_cb = NULL,// transactionが完了した後に呼ばれる関数をセットできる
};

spi_device_interface_config_t devcfg_pmw = {
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .mode = 3,
    .duty_cycle_pos = 128,  // default 128 = 50%/50% duty
    .cs_ena_pretrans = 0, // 0 not used
    .cs_ena_posttrans = 0,  // 0 not used
    .clock_speed_hz = 2000000,// 8,9,10,11,13,16,20,26,40,80
    .spics_io_num = PMW_CS,
    .flags = 0,  // 0 not used
    .queue_size = 10,// transactionのキュー数。1以上の値を入れておく。
    .pre_cb = NULL,// transactionが始まる前に呼ばれる関数をセットできる
    .post_cb = NULL,// transactionが完了した後に呼ばれる関数をセットできる
};

// SPIデバイスハンドラーを使って通信する
spi_device_handle_t bmi;
spi_device_handle_t pmw;

esp_err_t spi_init(void)
{
    esp_err_t ret = ESP_OK;
    //Initialize the SPI bus
    pinMode(46, OUTPUT);//CSを設定
    digitalWrite(46, 1);//CSをHIGH
    pinMode(12, OUTPUT);//CSを設定
    digitalWrite(12, 1);//CSをHIGH

    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO) ;   
    if(ret != ESP_OK) return ret;
    ret = spi_bus_add_device(SPI2_HOST, &devcfg_bmi, &bmi);
    if(ret != ESP_OK) return ret;
    ret = spi_bus_add_device(SPI2_HOST, &devcfg_pmw, &pmw);
    if(ret != ESP_OK) return ret;
    return ret;
}


/*!
 * SPI read function
 */
int8_t bmi2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    // 読み込み
    spi_transaction_t trans;
    esp_err_t ret=0;

    memset(&trans, 0, sizeof(trans)); // 構造体をゼロで初期化
    
    Buffer[0]=reg_addr|0x80;
    //3Byte書き込み、読み込み
    //trans.flags = SPI_TRANS_CS_KEEP_ACTIVE|SPI_TRANS_USE_TXDATA;
    //trans.flags = SPI_TRANS_USE_TXDATA;
    trans.tx_buffer =Buffer;
    trans.rx_buffer =reg_data;
    trans.length = 8+len*8;
    ret=spi_device_polling_transmit(bmi, &trans);
    uint16_t index = 0;
    while(index<len)
    {
        reg_data[index]=reg_data[index+1];
        index++;
    }
    assert(ret==ESP_OK);
    return ret;
}

/*!
 * SPI write function map to COINES platform
 */
int8_t bmi2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    spi_transaction_t trans;
    esp_err_t ret;
    //uint8_t buffer[3];
    uint8_t tmp;

    Buffer[0] = reg_addr&0b01111111;
    memcpy(&Buffer[1], reg_data, len);
    memset(&trans, 0, sizeof(trans)); // 構造体をゼロで初期化
    
    //buffer[0]=0x7C;
    //buffer[1]=0;
    //buffer[2]=4;

    // アドレス+データの書き込み
    trans.tx_buffer = Buffer;
    trans.rx_buffer = NULL;
    trans.length = 8+(len)*8;
    trans.rxlength = 0;

    //書き込み
    ret = spi_device_polling_transmit(bmi, &trans);
    assert(ret==ESP_OK);
    
    //spi_device_release_bus(spidev);

    return ret;
}

/*!
 * SPI read function
 */
int8_t spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t rxbuffer[8];
    

    // 読み込み
    spi_transaction_t trans;
    esp_err_t ret=0;

    memset(&trans, 0, sizeof(trans)); // 構造体をゼロで初期化
    reg_addr &= ~0x80u;
    Buffer[0]=reg_addr;
    Buffer[1]=0;
    //3Byte書き込み、読み込み
    //trans.flags = SPI_TRANS_CS_KEEP_ACTIVE|SPI_TRANS_USE_TXDATA;
    //trans.flags = SPI_TRANS_USE_TXDATA;
    trans.tx_buffer =Buffer;
    trans.rx_buffer =rxbuffer;
    trans.length = 16;
    ret=spi_device_polling_transmit(pmw, &trans);
    reg_data[0]=rxbuffer[1];
    assert(ret==ESP_OK);
    return ret;
}

/*!
 * SPI write function map to COINES platform
 */
int8_t spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    spi_transaction_t trans;
    esp_err_t ret;

    reg_addr |= 0x80u;
    Buffer[0] = reg_addr;
    memcpy(&Buffer[1], reg_data, len);
    memset(&trans, 0, sizeof(trans)); // 構造体をゼロで初期化
    
    // アドレス+データの書き込み
    trans.tx_buffer = Buffer;
    trans.rx_buffer = NULL;
    trans.length = 16;
    //trans.rxlength = 0;

    //書き込み
    ret = spi_device_polling_transmit(pmw, &trans);
    assert(ret==ESP_OK);
 
    //spi_device_release_bus(spidev);

    return ret;
}
