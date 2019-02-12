/*****************************************************************************
  BH1792GLC.ino

 Copyright (c) 2017 ROHM Co.,Ltd.

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
#include <avr/sleep.h>
#include <Wire.h>
#include <FlexiTimer2.h>

extern "C" {
#include <bh1792.h>
}

bh1792_t      m_bh1792;
bh1792_data_t m_bh1792_dat;

void timer_isr(void);
void bh1792_isr(void);
int32_t i2c_write(uint8_t slv_adr, uint8_t reg_adr, uint8_t *reg, uint8_t reg_size);
int32_t i2c_read(uint8_t slv_adr, uint8_t reg_adr, uint8_t *reg, uint8_t reg_size);
void error_check(int32_t ret, String msg);

void setup() {
  int32_t ret = 0;

  // Sleep Mode
  set_sleep_mode(SLEEP_MODE_IDLE);

  // Serial Port
  Serial.begin(115200);
  while (!Serial);

  // I2C
  Wire.begin();
  Wire.setClock(400000L);

  // BH1792
  m_bh1792.fnWrite      = i2c_write;
  m_bh1792.fnRead       = i2c_read;
  ret = bh1792_Init(&m_bh1792);
  error_check(ret, "bh1792_Init");

  m_bh1792.prm.sel_adc  = BH1792_PRM_SEL_ADC_GREEN;
  m_bh1792.prm.msr      = BH1792_PRM_MSR_SINGLE;//BH1792_PRM_MSR_1024HZ;
  m_bh1792.prm.led_en   = (BH1792_PRM_LED_EN1_0 << 1) | BH1792_PRM_LED_EN2_0;
  m_bh1792.prm.led_cur1 = BH1792_PRM_LED_CUR1_MA(1);
  m_bh1792.prm.led_cur2 = BH1792_PRM_LED_CUR2_MA(0);
  m_bh1792.prm.ir_th    = 0xFFFC;
  m_bh1792.prm.int_sel  = BH1792_PRM_INT_SEL_SGL;//BH1792_PRM_INT_SEL_WTM;
  ret = bh1792_SetParams();
  error_check(ret, "bh1792_SetParams");

  Serial.println(F("GDATA(@LED_ON),GDATA(@LED_OFF)"));

  ret = bh1792_StartMeasure();
  error_check(ret, "bh1792_StartMeasure");

  attachInterrupt(0, bh1792_isr, LOW);

  FlexiTimer2::stop();
  if (m_bh1792.prm.msr <= BH1792_PRM_MSR_1024HZ) {
    FlexiTimer2::set(2000, 5.0/10000, timer_isr);    // 1Hz timer
  } else {
    FlexiTimer2::set(250, 1.0/8000, timer_isr);      // 32Hz timer
  }
  FlexiTimer2::start();
}

void loop() {
  sleep_mode();
}

void timer_isr(void) {
  int32_t ret = 0;
  uint8_t tmp_eimsk;

  tmp_eimsk = EIMSK;
  EIMSK = 0;
  interrupts();

  if (m_bh1792.prm.msr <= BH1792_PRM_MSR_1024HZ) {
    ret = bh1792_SetSync();
    error_check(ret, "bh1792_SetSync");

    if (m_bh1792.sync_seq < 3) {
      if (m_bh1792.sync_seq == 1) {
        tmp_eimsk = 0;
      } else {
        ret = bh1792_ClearFifoData();
        error_check(ret, "bh1792_ClearFifoData");

        tmp_eimsk = bit(INT0);
      }
    }
  } else {
    ret = bh1792_StartMeasure();
    error_check(ret, "bh1792_StartMeasure");
  }

  noInterrupts();
  EIMSK |= tmp_eimsk;
}

void bh1792_isr(void) {
  int32_t ret = 0;
  uint8_t i   = 0;

  EIMSK = 0;
  interrupts();

  ret = bh1792_GetMeasData(&m_bh1792_dat);
  error_check(ret, "bh1792_GetMeasData");

  if(m_bh1792.prm.msr <= BH1792_PRM_MSR_1024HZ) {
    for (i = 0; i < m_bh1792_dat.fifo_lev; i++) {
      Serial.print(m_bh1792_dat.fifo[i].on, DEC);
      Serial.print(F(","));
      Serial.println(m_bh1792_dat.fifo[i].off, DEC);
    }
  } else {
    if(m_bh1792.prm.sel_adc == BH1792_PRM_SEL_ADC_GREEN) {
      Serial.print(m_bh1792_dat.green.on, DEC);
      Serial.print(F(","));
      Serial.println(m_bh1792_dat.green.off, DEC);
    } else {
      Serial.print(m_bh1792_dat.ir.on, DEC);
      Serial.print(F(","));
      Serial.println(m_bh1792_dat.ir.off, DEC);
    }
  }

  noInterrupts();
  EIMSK = bit(INT0);
}

// Note:  I2C access should be completed within 0.5ms
int32_t i2c_write(uint8_t slv_adr, uint8_t reg_adr, uint8_t *reg, uint8_t reg_size)
{
  byte rc;

  if (m_bh1792.prm.msr <= BH1792_PRM_MSR_1024HZ) {
    if((slv_adr != BH1792_SLAVE_ADDR) || (reg_adr != BH1792_ADDR_MEAS_SYNC)) {
      while(FlexiTimer2::count == 1999);
    }
  }

  Wire.beginTransmission(slv_adr);
  Wire.write(reg_adr);
  Wire.write(reg, reg_size);
  rc = Wire.endTransmission(true);

  return rc;
}

// Note:  I2C access should be completed within 0.5ms
int32_t i2c_read(uint8_t slv_adr, uint8_t reg_adr, uint8_t *reg, uint8_t reg_size)
{
  byte    rc;
  uint8_t cnt;

  if (m_bh1792.prm.msr <= BH1792_PRM_MSR_1024HZ) {
    while(FlexiTimer2::count == 1999);
  }

  Wire.beginTransmission(slv_adr);
  Wire.write(reg_adr);
  rc = Wire.endTransmission(false);
  if (rc == 0) {
    Wire.requestFrom((int32_t)slv_adr, (int32_t)reg_size, true);
    cnt = 0;
    while(Wire.available()) {
      reg[cnt] = Wire.read();
      cnt++;
    }
    if(cnt < reg_size) {
      rc = 4;
    }
  }

  return rc;
}

void error_check(int32_t ret, String msg)
{
  if(ret < 0) {
    msg = "Error: " + msg;
    msg += " function";
    Serial.println(msg);
    Serial.print("ret = ");
    Serial.println(ret, DEC);
    if(ret == BH1792_I2C_ERR) {
      Serial.print("i2c_ret = ");
      Serial.println(m_bh1792.i2c_err, DEC);
    }
    while(1);
  }
}
