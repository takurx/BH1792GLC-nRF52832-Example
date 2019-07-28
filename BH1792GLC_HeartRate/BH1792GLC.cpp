/*****************************************************************************
  BH1792GLC.cpp

 Copyright (c) 2016 ROHM Co.,Ltd.

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
******************************************************************************/
#include <avr/pgmspace.h>
#include <Arduino.h>
#include <Wire.h>
#include "BH1792GLC.h"

BH1792GLC::BH1792GLC(void)
{

}

byte BH1792GLC::init(void)
{
  byte rc;
  unsigned char reg;
  unsigned char val[3];

  rc = read(BH1792GLC_PART_ID, &reg, sizeof(reg));
  if (rc != 0) {
    Serial.println(F("Can't access BH1792GLC"));
    return (rc);
  }
  Serial.print(F("BH1792GLC Part ID Value = 0x"));
  Serial.println(reg, HEX);

  if (reg != BH1792GLC_PID_VAL) {
    Serial.println(F("Can't find BH1792GLC"));
    return (rc);
  }

  rc = read(BH1792GLC_MANUFACTURER_ID, &reg, sizeof(reg));
  if (rc != 0) {
    Serial.println(F("Can't access BH1792GLC"));
    return (rc);
  }
  Serial.print(F("BH1792GLC MANUFACTURER ID Register Value = 0x"));
  Serial.println(reg, HEX);

  if (reg != BH1792GLC_MID_VAL) {
    Serial.println(F("Can't find BH1792GLC"));
    return (rc);
  }

  val[0] = BH1792GLC_MEAS_CONTROL1_VAL;
  val[1] = BH1792GLC_MEAS_CONTROL2_VAL;
  val[2] = BH1792GLC_MEAS_START_VAL;  
  rc = write(BH1792GLC_MEAS_CONTROL1, val, sizeof(val));
  if (rc != 0) {
    Serial.println("Can't write BH1792GLC MEAS_CONTROL1-MEAS_START register");
  }
  
  return (rc);
}

byte BH1792GLC::get_rawval(unsigned char *data)
{
  byte rc;

  rc = read(BH1792GLC_DATAOUT_LEDOFF, data, 4);
  if (rc != 0) {
    Serial.println(F("Can't get BH1792GLC value"));
  }

  return (rc);
}

byte BH1792GLC::get_val(unsigned short *data)
{
  byte rc;
  unsigned char val[4];

  rc = get_rawval(val);
  if (rc != 0) {
    return (rc);
  }

  data[0] = ((unsigned short)val[1] << 8) | (val[0]);
  data[1] = ((unsigned short)val[3] << 8) | (val[2]);

  return (rc);  
}

byte BH1792GLC::write(unsigned char memory_address, unsigned char *data, unsigned char size)
{
  byte rc;

  Wire.beginTransmission(BH1792GLC_DEVICE_ADDRESS);
  Wire.write(memory_address);
  Wire.write(data, size);
  rc = Wire.endTransmission(true);
  return (rc);
}

byte BH1792GLC::read(unsigned char memory_address, unsigned char *data, int size)
{
  byte rc;
  unsigned char cnt;

  Wire.beginTransmission(BH1792GLC_DEVICE_ADDRESS);
  Wire.write(memory_address);
  rc = Wire.endTransmission(false);
  if (rc != 0) {
    return (rc);
  }

  Wire.requestFrom(BH1792GLC_DEVICE_ADDRESS, size, true);
  cnt = 0;
  while(Wire.available()) {
    data[cnt] = Wire.read();
    cnt++;
  }

  return (0);
}

