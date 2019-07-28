/////////////////////////////////////////////////////////////////////////////////
// hr_bh1792.c
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


#include <hr_bh1792.h>
//#include <typeDef.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//  Global Variables


//  Static Variables
static uint8_t    s_is_wearing;
static u16_pair_t s_pwData;
static TD_STATE   s_td_stat;
static hrParam    s_hrPrm;

// Local Function Prototype


/////////////////////////////////////////////////////////////////////////////////
//  Public Functions
/////////////////////////////////////////////////////////////////////////////////

//===============================================================================
// @brief  Initialize Heart Rate
// 
// @param[in]    : None
// @param[out]   : None
// @param[inout] : None
// @retval       : uint16_t
//                   ERROR_NONE                    => No error
//                   ERROR_PW_EXIST_SENSOR         => No PW Sensor exists
//                   ERROR_PW_I2C                  => I2C error with PW Sensor
//                   ERROR_BH1792_PRM_CTRL1_FREQ   => about LED emitting frequency
//                   ERROR_BH1792_PRM_CTRL1_RCYCLE => about RCYCLE
//                   ERROR_BH1792_PRM_CTRL2_EN     => about LED en
//                   ERROR_BH1792_PRM_CTRL2_ONTIME => about LED emitting time
//                   ERROR_BH1792_PRM_CTRL2_CUR    => about LED driver current
//                   ERROR_BH1792_PRM_TYPE         => Undefined Parameter
//                   ERROR_PWCALC_MA_PARAM         => Moving Average  on pwCalc_Init function
//                   ERROR_HR_MA_PARAM             => Moving Average  on hr_Init function
//===============================================================================
uint16_t hr_bh1792_Init(void)
{
    uint16_t ret16 = ERROR_NONE;

    memset(&s_pwData, 0, sizeof(u16_pair_t));

    ret16 = pw_Init();
    if (ret16 == ERROR_NONE) {
        touchDet_Init(&s_td_stat);
        ret16 = lxCtrl_Init();
        if (ret16 == ERROR_NONE) {
            ret16 = pwCalc_Init();
            if (ret16 == ERROR_NONE) {
                ret16 = hr_Init(&s_hrPrm);
                if (ret16 != ERROR_NONE)
                {
                    NRF_LOG_INFO("hr_Init error");
                }
            }
            else
            {
                NRF_LOG_INFO("pwCalc_Init error");
            }
        }
        else
        {
            NRF_LOG_INFO("lxCtrl_Init error");
        }
    }
    else
    {
        NRF_LOG_INFO("pw_Init error");
    }

    return (ret16);
}

//===============================================================================
// @brief Start Measurement of Heart Rate
// 
// @param[in]    : None
// @param[out]   : None
// @param[inout] : None
// @retval       : uint16_t
//                   ERROR_NONE              => No error
//                   ERROR_PW_I2C            => I2C error with PW Sensor
//===============================================================================
uint16_t hr_bh1792_StartMeasure(void)
{
    uint16_t ret16 = ERROR_NONE;

    //ret16 = pw_StartMeasure();
    ret16 = bh1792_StartMeasure();

    return (ret16);
}

//===============================================================================
// @brief One More Start Measurement of Heart Rate
// 
// @param[in]    : None
// @param[out]   : None
// @param[inout] : None
// @retval       : uint16_t
//                   ERROR_NONE              => No error
//                   ERROR_PW_I2C            => I2C error with PW Sensor
//===============================================================================
/*
uint16_t hr_bh1792_OneMoreStartMeasure(void)
{
    uint16_t ret16 = ERROR_NONE;

    ret16 = pw_OneMoreStartMeasure();

    return (ret16);
}
*/

//===============================================================================
// @brief Calculate Heart Rate and Touch Detection
// Need to repeat in a period for RCYCLE parameter.
// 
// @param[in]    : None
// @param[out]   : None
// @param[inout] : None
// @retval       : uint16_t
//                   ERROR_NONE                    => No error
//                   ERROR_PW_I2C                  => I2C error with PW Sensor
//                   ERROR_PW_NOT_MEASURE          => Must start the measurement
//                   ERROR_BH1792_PRM_CTRL1_FREQ   => about LED emitting frequency
//                   ERROR_BH1792_PRM_CTRL1_RCYCLE => about RCYCLE
//                   ERROR_BH1792_PRM_CTRL2_EN     => about LED en
//                   ERROR_BH1792_PRM_CTRL2_ONTIME => about LED emitting time
//                   ERROR_BH1792_PRM_CTRL2_CUR    => about LED driver current
//                   ERROR_BH1792_PRM_TYPE         => Undefined Parameter
//===============================================================================
//static float32_t s_pw;
//uint16_t hr_bh1792_Calc(uint8_t cnt_freq)
//uint16_t hr_bh1792_Calc(uint8_t cnt_freq, u16_pair_t *s_pwData_test, uint8_t *s_is_wearing_test, uint8_t *is_updated_led_test, float32_t *pw_test)
//uint16_t hr_bh1792_Calc(uint8_t cnt_freq, u16_pair_t *data, float32_t *pw_test)
uint16_t hr_bh1792_Calc(uint8_t cnt_freq, bh1792_data_t *bh1792_dat, u16_pair_t *data, float32_t *pw_test)
{
	uint8_t   meas_hr        = 0U;
    uint8_t   is_updated_led = 0U;
    uint16_t  ret16          = ERROR_NONE;
    float32_t pw             = 0.0F;

    // Measure
    /*
    if ((s_td_stat == td_state_w2) || (s_td_stat == td_state_w4)) {
        if(cnt_freq == 0U) {
            meas_hr = 1U;
        }
        else {
            meas_hr = 0U;
        }
    }
    else {
        meas_hr = 1U;
    }
    */
    meas_hr = 1U;

    if (meas_hr == 1U) {
        //ret16 = pw_GetMeasureData(&s_pwData);
        //data->on = s_pwData.on;
        //data->off = s_pwData.off;
        data->on = bh1792_dat->green.on;
        data->off = bh1792_dat->green.off;
        //s_pwData = bh1792_dat->green;
        s_pwData.on = bh1792_dat->green.on;
        s_pwData.off = bh1792_dat->green.off;
        if (ret16 == ERROR_NONE) {
            pwCalc(&s_pwData, &pw);
            
            *pw_test = pw;
            //*pw_test = 2.222;
            //*pw_test = (float32_t)(s_pwData.on);
            touchDet(&s_pwData, pw, &s_td_stat);
            if (s_td_stat == td_state_w5) {
                s_is_wearing = HR_WEARING;
            }
            else {
                s_is_wearing = HR_NOT_WEARING;
            }

            //s_is_wearing = HR_WEARING;

            ret16 = lxCtrl(&s_pwData, s_td_stat, &is_updated_led);
            //*is_updated_led_test = is_updated_led;
            //*s_is_wearing_test = s_is_wearing;
            //s_is_wearing_test = (uint8_t *)(s_td_stat);
            ret16 = ERROR_NONE;
            if (ret16 == ERROR_NONE) {
                if ((s_is_wearing == HR_WEARING) && (is_updated_led == 0U)) {
                    hr_CalcBPM(pw, &s_hrPrm);
                }
                else {
                    pwCalc_Clear();
                    ret16 = hr_Clear(&s_hrPrm, s_is_wearing);
                }
            }
        }
        else
        {
            *pw_test = 5.555;
        }
        
    }
    //*pw_test = 5.555;
    *pw_test = (float32_t)(s_pwData.on);

    return (ret16);
}


uint16_t hr_bh1792_GetMeasureData(u16_pair_t *s_pwData_test)
{
    uint16_t  ret16          = ERROR_NONE;

    ret16 = pw_GetMeasureData(s_pwData_test);

    return (ret16);
}

//===============================================================================
// @brief  Get Heart Rate value and wearing status
// 
// @param[in]    : None
// @param[out]   : uint8_t *bpm          => Pointer to Heart Rate[Units: bpm]
//               : uint8_t *wearing      => Pointer to Wearing or Not-Wearing
// @param[inout] : None
// @retval       : void
//===============================================================================
void hr_bh1792_GetData(uint8_t *bpm, uint8_t *wearing)
{
  *bpm     = s_hrPrm.bpm;
  *wearing = s_is_wearing;
}




/////////////////////////////////////////////////////////////////////////////////
//  Local Function
/////////////////////////////////////////////////////////////////////////////////
