/////////////////////////////////////////////////////////////////////////////////
// pw_bh1792.c
// 
// Copyright (c) 2016 ROHM Co.,Ltd.
// Copyright (c) 2019 Yoshihiro Nakagawa a.k.a. takurx, JJT1BC
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

#include <sysDef.h>
#include <bh1792.h>
#include <pw_bh1792.h>


//  Global Variables


//  Static Variables
static uint8_t is_measured;    // Measurement flag

//static uint8_t s_pw_freq;      // LED_LIGHTING_FREQ parameter
//static uint8_t s_pw_rCycle;    // RCYCLE            parameter
static uint8_t s_pw_mode;        // MEASURE_MODE       parameter
static uint8_t s_pw_cur1;        // LED1_CURRENT       parameter
//static uint8_t s_pw_onTime;    // LED_ON_TIME       parameter
static uint8_t s_pw_en1;         // LED1_EN            parameter
static uint8_t s_pw_cur2;        // LED2_CURRENT       parameter
static uint8_t s_pw_en2;         // LED2_EN            parameter


// Local Function Prototype
static uint16_t errorCode_8toU16(int8_t code);


/////////////////////////////////////////////////////////////////////////////////
//  Public Functions
/////////////////////////////////////////////////////////////////////////////////

//===============================================================================
// @brief  Initialize Pulse Wave Sensor
// Initialize BH1792, and set the default configuration parameters.
// 
// @param[in]    : None
// @param[out]   : None
// @param[inout] : None
// @retval       : uint16_t
//                   ERROR_NONE              => No error
//                   ERROR_PW_EXIST_SENSOR   => No PW Sensor exists
//                   ERROR_PW_I2C            => I2C error with PW Sensor
//===============================================================================
uint16_t pw_Init(void)
{
    int8_t   ret8  = BH1792_SUCCESS;
    uint16_t ret16 = ERROR_NONE;

    is_measured  = 0U;
    //s_pw_rCycle  = BH1792_PRM_CTRL1_RCYCLE_32HZ;
    //s_pw_freq    = BH1792_PRM_CTRL1_FREQ_128HZ;
    //s_pw_cur     = BH1792_PRM_LED_CUR1_MA(0);
    s_pw_mode     = BH1792_PRM_MSR_SINGLE;
    s_pw_cur1     = BH1792_PRM_LED_CUR1_MA(0);
    //s_pw_onTime  = BH1792_PRM_CTRL2_ONTIME_0_3MS;
    //s_pw_en      = BH1792_PRM_CTRL2_EN_NONE;
    s_pw_en1      = BH1792_PRM_LED_EN1_0;
    s_pw_cur2     = BH1792_PRM_LED_CUR2_MA(0);
    s_pw_en2      = BH1792_PRM_LED_EN2_0;

    ret8  = bh1792_Init();
    ret16 = errorCode_8toU16(ret8);
    
    return (ret16);
}

//===============================================================================
// @brief Start Measurement of Pulse Wave
// 
// @param[in]    : None
// @param[out]   : None
// @param[inout] : None
// @retval       : uint16_t
//                   ERROR_NONE              => No error
//                   ERROR_PW_I2C            => I2C error with PW Sensor
//===============================================================================
uint16_t pw_StartMeasure(void)
{
    int8_t     ret8  = BH1792_SUCCESS;
    uint16_t   ret16 = ERROR_NONE;
    uint8_t    reg[7];
    //MEAS_CTRL1 ctrl1 = { s_pw_rCycle, s_pw_freq, 0, BH1792_PRM_CTRL1_RDY };
    //MEAS_CTRL2 ctrl2 = { s_pw_cur, 0, s_pw_onTime, s_pw_en };
    MEAS_CTRL1 ctrl1 = { s_pw_mode, 0, BH1792_PRM_CTRL1_SEL_ADC, 0, BH1792_PRM_CTRL1_RDY };
    MEAS_CTRL2 ctrl2 = { s_pw_cur1, s_pw_en1 };
    MEAS_CTRL3 ctrl3 = { s_pw_cur2, 0, s_pw_en2 };
    MEAS_CTRL4_LOW ctrl4_low = {0xFC};
    MEAS_CTRL4_HIGH ctrl4_high = {0xFF};
    MEAS_CTRL5 ctrl5 = {0x03U, 0};
    MEAS_START start = { BH1792_PRM_MEAS_ST, 0 };
    uint8_t    reg1;
    
    ret8 = bh1792_SoftReset();
    if (ret8 == BH1792_SUCCESS) {
        // BH1792_MEAS_CTRL1 Config Parameters
        reg[0] = *(uint8_t *)&ctrl1;
        
        // BH1792_MEAS_CTRL2 Config Parameters
        reg[1] = *(uint8_t *)&ctrl2;
        
        // BH1792_MEAS_CTRL3 Config Parameters
        reg[2] = *(uint8_t *)&ctrl3;
        
        // BH1792_MEAS_CTRL3 Config Parameters
        reg[3] = *(uint8_t *)&ctrl4_low;

        // BH1792_MEAS_CTRL3 Config Parameters
        reg[4] = *(uint8_t *)&ctrl4_high;

        // BH1792_MEAS_CTRL3 Config Parameters
        reg[5] = *(uint8_t *)&ctrl5;

        // BH1792_MEAS_START Config Parameters
        reg[6] = *(uint8_t *)&start;

        // Command Send
        ret8 = bh1792_Write(BH1792_MEAS_CTRL1, &reg, sizeof(reg));
        if (ret8 == BH1792_SUCCESS) {
            is_measured = 1U;
        }
    }
    
    ret16 = errorCode_8toU16(ret8);
    
    return (ret16);
}

//===============================================================================
// @brief One More Start Measurement of Pulse Wave
// 
// @param[in]    : None
// @param[out]   : None
// @param[inout] : None
// @retval       : uint16_t
//                   ERROR_NONE              => No error
//                   ERROR_PW_I2C            => I2C error with PW Sensor
//===============================================================================
/*
uint16_t pw_OneMoreStartMeasure(void)
{
    int8_t     ret8  = BH1792_SUCCESS;
    uint16_t   ret16 = ERROR_NONE;

    MEAS_START start = { BH1792_PRM_MEAS_ST, 0 };
    uint8_t    reg1;
    
    // BH1792_MEAS_START Config Parameters
    reg1 = *(uint8_t *)&start;
    
    // Command Send
    ret8 = bh1792_Write(BH1792_MEAS_START, (uint8_t *)reg1, sizeof(reg1));
    if (ret8 == BH1792_SUCCESS) {
        is_measured = 1U;
    }
    
    ret16 = errorCode_8toU16(ret8);
    
    return (ret16);
}
*/

//===============================================================================
// @brief Stop Measurement of Pulse Wave
// 
// @param[in]    : None
// @param[out]   : None
// @param[inout] : None
// @retval       : uint16_t
//                   ERROR_NONE              => No error
//                   ERROR_PW_I2C            => I2C error with PW Sensor
//===============================================================================
uint16_t pw_StopMeasure(void)
{
    int8_t   ret8  = BH1792_SUCCESS;
    uint16_t ret16 = ERROR_NONE;

    ret8 = bh1792_SoftReset();
    if (ret8 == BH1792_SUCCESS) {
        is_measured = 0U;
    }

    ret16 = errorCode_8toU16(ret8);
    
    return (ret16);
}

//===============================================================================
// @brief  Read Measurement Result
// Need to read repeatedly in a period for RCYCLE parameter.
// 
// @param[in]    : None
// @param[out]   : u16_pair_t *data          => Pointer to BH1792 Raw Data
// @param[inout] : None
// @retval       : uint16_t
//                   ERROR_NONE              => No error
//                   ERROR_PW_I2C            => I2C error with PW Sensor
//                   ERROR_PW_NOT_MEASURE    => Must start the measurement
//===============================================================================
uint16_t pw_GetMeasureData(u16_pair_t *data)
{
    int8_t   ret8  = BH1792_SUCCESS;
    uint16_t ret16 = ERROR_NONE;
    uint8_t  reg[5] = {0};
    
    if (is_measured == 0U) {
        ret16 = ERROR_PW_NOT_MEASURE;
    }
    else {
        ret8 = bh1792_Read(BH1792_DATAOUT_LEDOFF_LSBS, &reg[0], sizeof(reg));
        if (ret8 == BH1792_SUCCESS){
            data->off    = ((uint16_t)reg[1] << 8) | (uint16_t)reg[0];
            data->on     = ((uint16_t)reg[3] << 8) | (uint16_t)reg[2];
        }
        /*
        else
        {
            data->off    = 0xFFFF;
            data->on     = 0xFFFF;
        }
        */
        ret16 = errorCode_8toU16(ret8);
    }

    return (ret16);
}

//===============================================================================
// @brief Set Pulse Wave parameters
// 
// @param[in]    : uint8_t type              => Parameter Type
//                 uint8_t value             => Parameter Value
// @param[out]   : None
// @param[inout] : None
// @retval       : uint16_t
//                   System Error Code for BH1792
//===============================================================================
uint16_t pw_SetParam(uint8_t type, uint8_t value)
{
    uint16_t ret16 = ERROR_NONE;
    
    if (is_measured == 1U) {
        ret16 = ERROR_PW_MEASURING;
    }
    else {
        switch(type) {
            case BH1792_PRM_CTRL1_MSR:
                if ((value != BH1792_PRM_CTRL1_MSR_NOT_SET_VAL) && (value >= BH1792_PRM_CTRL1_RCYCLE_32HZ && value <= BH1792_PRM_MSR_SINGLE)) 
                {
                    ret16 =  ERROR_BH1792_PRM_CTRL1_MSR;
                }
                else {
                    s_pw_mode = value;
                }
                break;
            case BH1792_PRM_CTRL2_EN1:
                if (value > BH1792_PRM_CTRL2_EN1_LED1_LED2) {
                    ret16 =  ERROR_BH1792_PRM_CTRL2_EN1;
                }
                else {
                    s_pw_en1 = value;
                }
                break;
            case BH1792_PRM_CTRL2_CUR_LED1:
                if (value > BH1792_PRM_CTRL2_CUR_60MA) {
                    ret16 =  ERROR_BH1792_PRM_CTRL2_CUR_LED1;
                }
                else {
                    s_pw_cur1 = value;
                }
                break;
            case BH1792_PRM_CTRL3_EN2:
                if (value > BH1792_PRM_CTRL3_EN2_ON_LED3) {
                    ret16 =  ERROR_BH1792_PRM_CTRL3_EN2;
                }
                else {
                    s_pw_en2 = value;
                }
                break;
            case BH1792_PRM_CTRL3_CUR_LED2:
                if (value > BH1792_PRM_CTRL2_CUR_60MA) {
                    ret16 =  ERROR_BH1792_PRM_CTRL3_CUR_LED2;
                }
                else {
                    s_pw_cur2 = value;
                }
                break;
            default:
                ret16 = ERROR_BH1792_PRM_TYPE;
                break;
        }
    }
    
    return (ret16);
}

//===============================================================================
// @brief Get Pulse Wave parameters
// 
// @param[in]    : uint8_t type              => Parameter Type
//                 void    *value            => Pointer to Parameter Value
// @param[out]   : None
// @param[inout] : None
// @retval       : uint16_t
//                   System Error Code for BH1792
//===============================================================================
uint16_t pw_GetParam(uint8_t type, uint8_t *value)
{
    uint16_t ret16 = ERROR_NONE;
    
    switch(type) {
        case BH1792_PRM_CTRL1_MSR:
            *value = s_pw_mode;
            break;
        case BH1792_PRM_CTRL2_EN1:
            *value = s_pw_en1;
            break;
        case BH1792_PRM_CTRL2_CUR_LED1:
            *value = s_pw_cur1;
            break;
        case BH1792_PRM_CTRL3_EN2:
            *value = s_pw_en2;
            break;
        case BH1792_PRM_CTRL3_CUR_LED2:
            *value = s_pw_cur2;
            break;
        default:
            ret16 = ERROR_BH1792_PRM_TYPE;
            break;
    }
    
    return (ret16);
}


/////////////////////////////////////////////////////////////////////////////////
//  Local Function
/////////////////////////////////////////////////////////////////////////////////

//===============================================================================
// @brief Exchange 8bit-signed error code to 16bit-unsigned
// 
// @param[in]    : int8_t code               => BH1792 Driver Error Code
// @param[out]   : None
// @param[inout] : None
// @retval       : uint16_t
//                   System Error Code
//===============================================================================
static uint16_t errorCode_8toU16(int8_t code)
{
    uint16_t ret16 = ERROR_NONE;

    switch (code) {
        case BH1792_SUCCESS:             ret16 = ERROR_NONE; break;
        case BH1792_RC_NO_EXIST:       ret16 = ERROR_PW_EXIST_SENSOR; break;
        case BH1792_RC_I2C_ERR:        ret16 = ERROR_PW_I2C; break;
        default:                       ret16 = ERROR_PW_UNKNOWN; break;
    }

    return (ret16);
}
