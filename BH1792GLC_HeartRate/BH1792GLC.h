/*****************************************************************************
  BH1792GLC.h

 Copyright (c) 2016 ROHM Co.,Ltd.
 Copyright (c) 2019 Yoshihiro Nakagawa a.k.a. takurx, JJT1BC

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
#ifndef _BH1792GLC_H_
#define _BH1792GLC_H_

#define BH1792GLC_DEVICE_ADDRESS      (0x5B)    // 7bit Addrss
#define BH1792GLC_MID_VAL             (0xE0)
#define BH1792GLC_PID_VAL             (0x0E)

#define BH1792GLC_MANUFACTURER_ID     (0x0F)
#define BH1792GLC_PART_ID             (0x10)
#define BH1792GLC_MEAS_CONTROL1       (0x41)
#define BH1792GLC_MEAS_CONTROL2       (0x42)
#define BH1792GLC_MEAS_CONTROL3       (0x43)
#define BH1792GLC_MEAS_START          (0x47)
#define BH1792GLC_DATAOUT_LEDOFF      (0x54)

#define BH1792GLC_MEAS_CONTROL1_RDY                     (1 << 7)
#define BH1792GLC_MEAS_CONTROL1_SEL_ADC                 (0 << 4)
//#define BH1792GLC_MEAS_CONTROL1_LED_LIGHTING_FREQ_128HZ (0 << 2)    // BH1792GLC not exist
//#define BH1792GLC_MEAS_CONTROL1_RCYCLE_32HZ             (2 << 0)    // BH1792GLC not exist
#define BH1792GLC_MEAS_CONTROL1_SINGLE_MEASURE_MODE     (0 << 2)

#define BH1792GLC_MEAS_CONTROL2_LED_EN1_00              (0 << 6)
//#define BH1792GLC_MEAS_CONTROL2_LED_ON_TIME_0_3MS       (0 << 5)    // BH1792GLC not exist
#define BH1792GLC_MEAS_CONTROL2_LED_CURRENT_10MA        (10 << 0)

#define BH1792GLC_MEAS_CONTROL3_LED_EN2_0               (0 << 7)
#define BH1792GLC_MEAS_CONTROL3_LED_CURRENT_10MA        (10 << 0)

#define BH1792GLC_MEAS_START_MEAS_ST                    (1 << 0)

#define BH1792GLC_MEAS_CONTROL1_VAL   (BH1792GLC_MEAS_CONTROL1_RDY | BH1792GLC_MEAS_CONTROL1_SEL_ADC | BH1792GLC_MEAS_CONTROL1_SINGLE_MEASURE_MODE)
#define BH1792GLC_MEAS_CONTROL2_VAL   (BH1792GLC_MEAS_CONTROL2_LED_EN1_00 | BH1792GLC_MEAS_CONTROL2_LED_CURRENT_10MA)
#define BH1792GLC_MEAS_CONTROL3_VAL   (BH1792GLC_MEAS_CONTROL3_LED_EN2_0 | BH1792GLC_MEAS_CONTROL2_LED_CURRENT_10MA)
#define BH1792GLC_MEAS_START_VAL      (BH1792GLC_MEAS_START_MEAS_ST)

class BH1792GLC
{
  public:
      BH1792GLC(void);
    byte init(void);
    byte get_rawval(unsigned char *data);
    byte get_val(unsigned short *data);
    byte write(unsigned char memory_address, unsigned char *data, unsigned char size);
    byte read(unsigned char memory_address, unsigned char *data, int size);
};

#endif // _BH1792GLC_H_
