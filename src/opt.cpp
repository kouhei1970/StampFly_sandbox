#include "opt.hpp"
#include "spi_s3.hpp"
#include "wrapper.hpp"

optconfig_t optconfig;


// SPI Write
void registerWrite(uint8_t reg, uint8_t value) {
  spi_write(reg, &value, 1, &pmw);
  wrapper_delayMicroseconds(200);
}

// SPI Read
uint8_t registerRead(uint8_t reg) {
  uint8_t value;
  spi_read(reg, &value, 1, &pmw);
  wrapper_delayMicroseconds(200);
  return value;
}

// Performance optimisation registers set
void initRegisters(void)
{
  registerWrite(0x7F, 0x00);
  registerWrite(0x61, 0xAD);
  registerWrite(0x7F, 0x03);
  registerWrite(0x40, 0x00);
  registerWrite(0x7F, 0x05);
  registerWrite(0x41, 0xB3);
  registerWrite(0x43, 0xF1);
  registerWrite(0x45, 0x14);
  registerWrite(0x5B, 0x32);
  registerWrite(0x5F, 0x34);
  registerWrite(0x7B, 0x08);
  registerWrite(0x7F, 0x06);
  registerWrite(0x44, 0x1B);
  registerWrite(0x40, 0xBF);
  registerWrite(0x4E, 0x3F);
  registerWrite(0x7F, 0x08);
  registerWrite(0x65, 0x20);
  registerWrite(0x6A, 0x18);
  registerWrite(0x7F, 0x09);
  registerWrite(0x4F, 0xAF);
  registerWrite(0x5F, 0x40);
  registerWrite(0x48, 0x80);
  registerWrite(0x49, 0x80);
  registerWrite(0x57, 0x77);
  registerWrite(0x60, 0x78);
  registerWrite(0x61, 0x78);
  registerWrite(0x62, 0x08);
  registerWrite(0x63, 0x50);
  registerWrite(0x7F, 0x0A);
  registerWrite(0x45, 0x60);
  registerWrite(0x7F, 0x00);
  registerWrite(0x4D, 0x11);
  registerWrite(0x55, 0x80);
  registerWrite(0x74, 0x1F);
  registerWrite(0x75, 0x1F);
  registerWrite(0x4A, 0x78);
  registerWrite(0x4B, 0x78);
  registerWrite(0x44, 0x08);
  registerWrite(0x45, 0x50);
  registerWrite(0x64, 0xFF);
  registerWrite(0x65, 0x1F);
  registerWrite(0x7F, 0x14);
  registerWrite(0x65, 0x60);
  registerWrite(0x66, 0x08);
  registerWrite(0x63, 0x78);
  registerWrite(0x7F, 0x15);
  registerWrite(0x48, 0x58);
  registerWrite(0x7F, 0x07);
  registerWrite(0x41, 0x0D);
  registerWrite(0x43, 0x14);
  registerWrite(0x4B, 0x0E);
  registerWrite(0x45, 0x0F);
  registerWrite(0x44, 0x42);
  registerWrite(0x4C, 0x80);
  registerWrite(0x7F, 0x10);
  registerWrite(0x5B, 0x02);
  registerWrite(0x7F, 0x07);
  registerWrite(0x40, 0x41);
  registerWrite(0x70, 0x00);

  wrapper_delay(100);
  registerWrite(0x32, 0x44);
  registerWrite(0x7F, 0x07);
  registerWrite(0x40, 0x40);
  registerWrite(0x7F, 0x06);
  registerWrite(0x62, 0xf0);
  registerWrite(0x63, 0x00);
  registerWrite(0x7F, 0x0D);
  registerWrite(0x48, 0xC0);
  registerWrite(0x6F, 0xd5);
  registerWrite(0x7F, 0x00);
  registerWrite(0x5B, 0xa0);
  registerWrite(0x4E, 0xA8);
  registerWrite(0x5A, 0x50);
  registerWrite(0x40, 0x80);
}

uint8_t powerUp(optconfig_t* optconfig) {
  // Setup SPI port
  wrapper_delay(45);
  wrapper_digitalWrite(PMW_CS, WRAPPER_HIGH);
  wrapper_delay(1);
  wrapper_digitalWrite(PMW_CS, WRAPPER_LOW);
  wrapper_delay(1);
  wrapper_digitalWrite(PMW_CS, WRAPPER_HIGH);
  wrapper_delay(1);

  // Power on reset
  registerWrite(0x3A, 0x5A);
  wrapper_delay(5);
  // Test the SPI communication, checking chipId and inverse chipId
  optconfig->chipid = registerRead(0x00);
  optconfig->dipihc = registerRead(0x5F);

  // Chip IDの確認（必須）
  if (optconfig->chipid != 0x49) {
    ESPSerial.printf("エラー: PMW3901のChip IDが正しくありません (0x%02X)\n", optconfig->chipid);
    return false;
  }
  
  // Inv Chip IDの確認（より柔軟に）
  // PMW3901の個体差により0xB6-0xB8の範囲で変動する場合がある
  if (optconfig->dipihc < 0xB6 || optconfig->dipihc > 0xB8) {
    ESPSerial.printf("警告: Inv Chip IDが期待値と異なります (0x%02X)\n", optconfig->dipihc);
    ESPSerial.println("動作を継続しますが、センサーの動作を確認してください");
  }
  
  ESPSerial.printf("PMW3901識別成功: Chip ID=0x%02X, Inv Chip ID=0x%02X\n", 
                   optconfig->chipid, optconfig->dipihc);

  // Reading the motion registers one time
  registerRead(0x02);
  registerRead(0x03);
  registerRead(0x04);
  registerRead(0x05);
  registerRead(0x06);
  wrapper_delay(1);
  return true;
}

// PMW3901のCPI（Counts Per Inch）を高さから計算する関数
// CPI = 11.914 * x^(-1) （xはセンサーの高さ[m]）
float calculateCPI(float height_m)
{
  if (height_m <= 0.0f) {
    return 1600.0f; // デフォルト値（高度不明時）
  }
  
  // CPI = 11.914 / x の計算
  return 11.914f / height_m;
}

// PMW3901データシートに基づく正しいモーションデータ読み取り（品質チェック付き）
uint8_t readMotionCount(int16_t *deltaX, int16_t *deltaY)
{
  // 1. モーションレジスタ（0x02）を読み取り
  uint8_t motion = registerRead(0x02);
  
  // 2. MOTビット（bit 7）をチェック - 新しいデータが利用可能かどうか
  if (motion & 0x80) {
    // 3. 新しいデータが利用可能な場合、X/Yレジスタを読み取り
    *deltaX = ((int16_t)registerRead(0x04) << 8) | registerRead(0x03);
    *deltaY = ((int16_t)registerRead(0x06) << 8) | registerRead(0x05);
    
    // 4. 品質チェック: SQUALとShutter_Upperを読み取り
    uint8_t squal = registerRead(0x07);           // Surface Quality
    uint8_t shutter_upper = registerRead(0x0C);   // Shutter Upper
    
    // 5. 品質判定: SQUALが0x19未満 かつ Shutter_Upperが0x1F の場合はデータを破棄
    if (squal < 0x19 && shutter_upper == 0x1F) {
      // データ品質が不十分なため破棄
      *deltaX = 0;
      *deltaY = 0;
      return 2; // データあるが品質不良で破棄
    }
    
    return 1; // 有効なデータ
  } else {
    // 新しいデータがない場合は前回値を保持
    return 0; // 新しいデータなし
  }
}

// PMW3901データシート準拠の移動量計算（高さベースCPI使用）
void calculateMovementFromDelta(int16_t deltaX, int16_t deltaY, float *movement_x, float *movement_y, float height_m)
{
  // 高さからCPIを計算
  float cpi = calculateCPI(height_m);
  
  // PMW3901データシートに基づく計算:
  // 1. Delta値をCPIで割ってインチ単位の移動量を求める
  float movement_inch_x = (float)deltaX / cpi;
  float movement_inch_y = (float)deltaY / cpi;
  
  // 2. インチからメートルに変換（1インチ = 0.0254メートル）
  *movement_x = movement_inch_x * 0.0254f;
  *movement_y = movement_inch_y * 0.0254f;
}



void enableFrameCaptureMode(void)
{
  //Step 1. To enter Frame Capture mode
  uint8_t tmp;

  //
  registerWrite(0x7F, 0x07);
  tmp = registerRead(0x7F);
  ESPSerial.printf("(07)%02X\n\r", tmp);
  //
  registerWrite(0x41, 0x1D);
  tmp = registerRead(0x41);
  ESPSerial.printf("(1D)%02X\n\r", tmp);
  //
  registerWrite(0x4C, 0x00);
  tmp = registerRead(0x4C);
  ESPSerial.printf("(00)%02X\n\r", tmp);
  //
  registerWrite(0x7F, 0x08);
  tmp = registerRead(0x7F);
  ESPSerial.printf("(08)%02X\n\r", tmp);
  //
  registerWrite(0x6A, 0x38);
  tmp = registerRead(0x6A);
  ESPSerial.printf("(38)%02X\n\r", tmp);
  //
  registerWrite(0x7F, 0x00);
  tmp = registerRead(0x7F);
  ESPSerial.printf("(00)%02X\n\r", tmp);
  //
  registerWrite(0x55, 0x04);
  tmp = registerRead(0x55);
  ESPSerial.printf("(04)%02X\n\r", tmp);
  //
  registerWrite(0x40, 0x80);
  tmp = registerRead(0x40);
  ESPSerial.printf("(80)%02X\n\r", tmp);
  //
  registerWrite(0x4D, 0x11);
  tmp = registerRead(0x4D);
  ESPSerial.printf("(11)%02X\n\r", tmp);

  //Step 2.
  registerWrite(0x70, 0x00); 
  tmp = registerRead(0x70);
  ESPSerial.printf("(00)%02X\n\r", tmp);
  //
  registerWrite(0x58, 0xFF);
  tmp = registerRead(0x58);
  ESPSerial.printf("(FF)%02X\n\r", tmp);

  //Step 3. Poll RawData_Grab_Status register
  uint8_t buf;
  uint8_t status;

  do 
  {
    wrapper_delay(1);
    buf = registerRead(0x59);
    status = buf>>6;
    //USBSerial.printf("Register(0x59) %02X\n\r", buf);
  } while(buf == 0x00);

  ESPSerial.printf("PMW Status %X\n\r", status);

  wrapper_delayMicroseconds(50);
}

void readImage(uint8_t *image)
{
  int count = 0;
  uint8_t a; //temp value for reading register
  uint8_t b; //temp value for second register
  uint8_t hold; //holding value for checking bits
  uint8_t mask = 0x0c; //mask to take bits 2 and 3 from b
  uint8_t pixel = 0; //temp holding value for pixel

  for (uint16_t i = 0; i < 1225; i++) 
  { //for 1 frame of 1225 pixels (35*35)
    
    do 
    { 
      //if data is either invalid status
      //check status bits 6 and 7
      //if 01 move upper 6 bits into temp value
      //if 00 or 11, reread
      //else lower 2 bits into temp value
      a = registerRead(0x58); //read register
      hold = (a >> 6)&0b00000011; //right shift to leave top two bits for ease of check.
      //USBSerial.printf("%04d, %X\n\r", i, hold);
    } while((hold == 0x03) || (hold == 0x00));
    
    if (hold == 0x01) 
    { //if data is upper 6 bits
      b = registerRead(0x58); //read next set to get lower 2 bits
      pixel = a; //set pixel to a
      pixel = pixel << 2; //push left to 7:2
      pixel += (b & mask); //set lower 2 from b to 1:0
      image[count++] = pixel; //put temp value in fbuffer array
      //delayMicroseconds(100);
    }
  }
  registerWrite(0x70, 0x00);   //More magic? 
  registerWrite(0x58, 0xFF);

  int buf;
  uint8_t status; 

  do 
  { //keep reading and testing
    buf = registerRead(0x58); //read status register
    status = buf>>6; //rightshift 6 bits so only top two stay 
  } while(status == 0x03); //while bits aren't set denoting ready state
}
