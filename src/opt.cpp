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

  if (optconfig->chipid != 0x49 && optconfig->dipihc != 0xB8) return false;

  // Reading the motion registers one time
  registerRead(0x02);
  registerRead(0x03);
  registerRead(0x04);
  registerRead(0x05);
  registerRead(0x06);
  wrapper_delay(1);
  return true;
}

void readMotionCount(int16_t *deltaX, int16_t *deltaY)
{
  registerRead(0x02);
  *deltaX = ((int16_t)registerRead(0x04) << 8) | registerRead(0x03);
  *deltaY = ((int16_t)registerRead(0x06) << 8) | registerRead(0x05);
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
