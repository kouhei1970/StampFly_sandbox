/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "tof.hpp"
#include "wrapper.hpp"
#include "esp_attr.h"
#include "pid.hpp"
#include <esp_timer.h>

VL53LX_Dev_t tof_front;
VL53LX_Dev_t tof_bottom;

VL53LX_DEV ToF_front  = &tof_front;
VL53LX_DEV ToF_bottom = &tof_bottom;

Filter tof_bottom_filter;
Filter tof_front_filter;

volatile uint8_t ToF_bottom_data_ready_flag;
#if 1
void IRAM_ATTR tof_int() {
    ToF_bottom_data_ready_flag = 1;
}
#endif

int16_t tof_bottom_get_range() {
    int16_t raw_range = tof_range_get(ToF_bottom);
    return raw_range;
    // サンプリング時間は0.0025秒（400Hz）と仮定
    //return (int16_t)tof_bottom_filter.update((float)raw_range, 0.0025f);
}

int16_t tof_front_get_range() {
    int16_t raw_range = tof_range_get(ToF_front);
    return raw_range;
    // サンプリング時間は0.0025秒（400Hz）と仮定
    //return (int16_t)tof_front_filter.update((float)raw_range, 0.0025f);
}

void tof_init(void) {
    uint8_t byteData;
    uint16_t wordData;
    uint8_t new_address;

    ToF_bottom->comms_speed_khz   = 400;
    ToF_bottom->i2c_slave_address = 0x29;
    ToF_bottom->xshut_pin = XSHUT_BOTTOM; // GPIO for bottom ToF XSHUT
    ToF_bottom->int_pin = INT_BOTTOM;     // GPIO for bottom ToF interrupt

    ToF_front->comms_speed_khz   = 400;
    ToF_front->i2c_slave_address = 0x29;
    ToF_front->xshut_pin = XSHUT_FRONT; // GPIO for front ToF XSHUT
    ToF_front->int_pin = INT_FRONT;     // GPIO for front ToF interrupt

    // USBSerial.printf("#tof_i2c_init_status:%d\r\n",vl53lx_i2c_init());

    // ToF Pin Initialize
    wrapper_pinMode((gpio_num_t)XSHUT_BOTTOM, WRAPPER_OUTPUT);
    wrapper_pinMode((gpio_num_t)XSHUT_FRONT, WRAPPER_OUTPUT);
    wrapper_pinMode((gpio_num_t)INT_BOTTOM, WRAPPER_INPUT);
    wrapper_pinMode((gpio_num_t)INT_FRONT, WRAPPER_INPUT);
    wrapper_pinMode((gpio_num_t)USER_A, WRAPPER_INPUT_PULLUP);

    // フィルタの初期化
    tof_bottom_filter.set_parameter(0.005f, 0.0025f); // 時定数0.05秒
    tof_front_filter.set_parameter(0.005f, 0.0025f);  // 時定数0.05秒
    
    // ToF Disable
    wrapper_digitalWrite((gpio_num_t)XSHUT_BOTTOM, WRAPPER_LOW);
    wrapper_digitalWrite((gpio_num_t)XSHUT_FRONT, WRAPPER_LOW);
    wrapper_delay(50);

    // Front ToF I2C address to 0x2A
    new_address = 0x2A; // New I2C address for front ToF

    //Front ToF Enable
    wrapper_digitalWrite((gpio_num_t)XSHUT_FRONT, WRAPPER_HIGH);
    wrapper_delay(50);

    ESPSerial.printf("Front ToF Address Change status %d\n\r",VL53LX_SetDeviceAddress( ToF_front, new_address<<1 ));
    ToF_front->i2c_slave_address = new_address; // Set new I2C address for front ToF

    // Bttom ToF Enable
    wrapper_digitalWrite((gpio_num_t)XSHUT_BOTTOM, WRAPPER_HIGH);
    wrapper_delay(50);

    // Bttom ToF setting
    ESPSerial.printf("#1 WaitDeviceBooted Status:%d\n\r", VL53LX_WaitDeviceBooted(ToF_bottom));
    ESPSerial.printf("#1 DataInit Status:%d\n\r", VL53LX_DataInit(ToF_bottom));
    ESPSerial.printf("#1 Range setting  Status:%d\n\r", VL53LX_SetDistanceMode(ToF_bottom, VL53LX_DISTANCEMODE_MEDIUM));
    ESPSerial.printf("#1 SetMeasurementTimingBuget Status:%d\n\r",
                     VL53LX_SetMeasurementTimingBudgetMicroSeconds(ToF_bottom, 33000));
    ESPSerial.printf("#1 RdByte Status:%d\n\r", VL53LX_RdByte(ToF_bottom, 0x010F, &byteData));
    ESPSerial.printf("#1 VL53LX Model_ID: %02X\n\r", byteData);
    ESPSerial.printf("#1 RdByte Status:%d\n\r", VL53LX_RdByte(ToF_bottom, 0x0110, &byteData));
    ESPSerial.printf("#1 VL53LX Module_Type: %02X\n\r", byteData);
    ESPSerial.printf("#1 RdWord Status:%d\n\r", VL53LX_RdWord(ToF_bottom, 0x010F, &wordData));
    ESPSerial.printf("#1 VL53LX: %04X\n\r", wordData);

    // Front ToF Setting
    ESPSerial.printf("#2 WaitDeviceBooted Status:%d\n\r", VL53LX_WaitDeviceBooted(ToF_front));
    ESPSerial.printf("#2 DataInit Status:%d\n\r", VL53LX_DataInit(ToF_front));
    //ESPSerial.printf("#2 Range setting  Status:%d\n\r", VL53LX_SetDistanceMode(ToF_front, VL53LX_DISTANCEMODE_LONG));
    ESPSerial.printf("#2 Range setting  Status:%d\n\r", VL53LX_SetDistanceMode(ToF_front, VL53LX_DISTANCEMODE_MEDIUM));
    ESPSerial.printf("#2 SetMeasurementTimingBuget Status:%d\n\r",
                     VL53LX_SetMeasurementTimingBudgetMicroSeconds(ToF_front, 33000));
    ESPSerial.printf("#2 RdByte Status:%d\n\r", VL53LX_RdByte(ToF_front, 0x010F, &byteData));
    ESPSerial.printf("#2 VL53LX Model_ID: %02X\n\r", byteData);
    ESPSerial.printf("#2 RdByte Status:%d\n\r", VL53LX_RdByte(ToF_front, 0x0110, &byteData));
    ESPSerial.printf("#2 VL53LX Module_Type: %02X\n\r", byteData);
    ESPSerial.printf("#2 RdWord Status:%d\n\r", VL53LX_RdWord(ToF_front, 0x010F, &wordData));
    ESPSerial.printf("#2 VL53LX: %04X\n\r", wordData);

    attachInterrupt((gpio_num_t)INT_BOTTOM, &tof_int, FALLING);

    VL53LX_ClearInterruptAndStartMeasurement(ToF_bottom);
    wrapper_delay(100);
    ESPSerial.printf("#Start Measurement Status:%d\n\r", VL53LX_StartMeasurement(ToF_bottom));
}

int16_t tof_range_get(VL53LX_DEV dev) {
    static int16_t last_valid_range = 10; // 初期値として10mmを設定
    int16_t range_ave = 0;
    uint8_t count = 0;
    bool valid_measurement = false;

    VL53LX_MultiRangingData_t MultiRangingData;
    VL53LX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;

    // マルチレンジングデータの取得
    VL53LX_GetMultiRangingData(dev, pMultiRangingData);
    uint8_t no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
    
    // 有効な測定値の平均を計算
    if (no_of_object_found > 0) {
        int32_t sum = 0;
        count = 0;
        
        for (uint8_t j = 0; j < no_of_object_found; j++) {
            if (MultiRangingData.RangeData[j].RangeStatus == VL53LX_RANGESTATUS_RANGE_VALID) {
                sum += MultiRangingData.RangeData[j].RangeMilliMeter;
                count++;
            }
        }
        
        if (count > 0) {
            range_ave = sum / count; // 有効な測定値の平均
            last_valid_range = range_ave; // 有効な測定値を保存
            valid_measurement = true;
        }
    }
    
    // 次の測定を開始
    VL53LX_ClearInterruptAndStartMeasurement(dev);
    
    // 有効な測定がなかった場合は前回の有効な測定値を返す
    return valid_measurement ? range_ave : last_valid_range;
}

void tof_test_ranging(VL53LX_DEV dev) {
    uint8_t status     = 0;
    uint8_t data_ready = 0;
    int16_t range;
    VL53LX_MultiRangingData_t MultiRangingData;
    VL53LX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;

    if (status == 0) {
        status = VL53LX_ClearInterruptAndStartMeasurement(dev);
    }
    wrapper_delay(100);
    ESPSerial.printf("#Start Measurement Status:%d\n\r", VL53LX_StartMeasurement(dev));

    ESPSerial.printf("#Count ObjNo Status Range Signal(Mcps) Ambient(Mcps)\n\r");

    uint64_t st, now, old, end;
    uint16_t count;
    st  = esp_timer_get_time();
    now = st;
    old = st;

    count = 0;
    ESPSerial.printf("Start!\n\r");
    while (count < 500) {
        // VL53LX_WaitMeasurementDataReady(dev);
        // if(digitalRead(INT_BOTTOM)==0)

        VL53LX_GetMeasurementDataReady(dev, &data_ready);
        if (data_ready == 1) {
            data_ready = 0;
            count++;
            VL53LX_GetMultiRangingData(dev, pMultiRangingData);
            old                        = now;
            now                        = esp_timer_get_time();
            uint8_t no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
            ESPSerial.printf("%7.4f %7.4f ", (float)(now - st) * 1e-6, (float)(now - old) * 1e-6);
            ESPSerial.printf("%5d ", pMultiRangingData->StreamCount);
            ESPSerial.printf("%1d ", no_of_object_found);
            for (uint8_t j = 0; j < no_of_object_found; j++) {
                if (j != 0) ESPSerial.printf("\n\r                     ");
                ESPSerial.printf("%d %5d %2.2f %2.2f ", MultiRangingData.RangeData[j].RangeStatus,
                                 MultiRangingData.RangeData[j].RangeMilliMeter,
                                 MultiRangingData.RangeData[j].SignalRateRtnMegaCps / 65536.0,
                                 MultiRangingData.RangeData[j].AmbientRateRtnMegaCps / 65536.0);
            }

            VL53LX_ClearInterruptAndStartMeasurement(dev);
            end = esp_timer_get_time();
            ESPSerial.printf("%8.6f", (float)(end - now) * 1.0e-6);
            ESPSerial.printf("\n\r");
        }
    }
    ESPSerial.printf("End!\n\r");
}
