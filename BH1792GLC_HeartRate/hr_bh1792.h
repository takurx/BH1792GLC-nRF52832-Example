/////////////////////////////////////////////////////////////////////////////////
// hr_bh1792.h
// 
// Copyright (c) 2016 ROHM Co.,Ltd.
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
/////////////////////////////////////////////////////////////////////////////////

#ifndef HR_BH1792_H
#define HR_BH1792_H

#include <string.h>
#include <sysDef.h>
#include <typeDef.h>
#include <bh1792.h>
#include <pw_bh1792.h>
#include <pwCalc.h>
#include <touchDet.h>
#include <lxCtrl.h>
#include <heartRate.h>

//===============================================================================
//  Constants
//===============================================================================


//===============================================================================
//  Type Definition
//===============================================================================


//===============================================================================
//  Extern Global Variables
//===============================================================================


//===============================================================================
//  Public Function Prototypes
//===============================================================================
uint16_t hr_bh1792_Init(void);
uint16_t hr_bh1792_StartMeasure(void);
//uint16_t hr_bh1792_OneMoreStartMeasure(void);
uint16_t hr_bh1792_GetMeasureData(u16_pair_t *s_pwData);
//uint16_t hr_bh1792_Calc(uint8_t cnt_freq);
//uint16_t hr_bh1792_Calc(uint8_t cnt_freq, u16_pair_t *s_pwData_test, uint8_t *s_is_wearing_test, uint8_t *is_updated_led_test, float32_t *pw_test);
uint16_t hr_bh1792_Calc(uint8_t cnt_freq, u16_pair_t *s_pwData_test, float32_t *pw_test);
void     hr_bh1792_GetData(uint8_t *bpm, uint8_t *wearing);


//===============================================================================
//  Macro
//===============================================================================



#endif  // HR_BH1792_H
