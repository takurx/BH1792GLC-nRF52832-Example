/////////////////////////////////////////////////////////////////////////////////
// bh1792.h
// 
// Copyright (c) 2016-17 ROHM Co.,Ltd.
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

#ifndef BH1792_H
#define BH1792_H

//#include <type_def.h>
#include <typeDef.h>

//==============================================================================
//  Constants
//==============================================================================
// Error Codes
#define BH1792_SUCCESS                  (0)      // BH1792 operation is successful
#define BH1792_I2C_ERR                  (-1)     // I2C error with BH1792
#define BH1792_NOT_EXIST                (-2)     // BH1792 does not exist
#define BH1792_FIFO_FULL                (-3)     // BH1792 FIFO is full

// Slave Address
#define BH1792_SLAVE_ADDR               (0x5BU)  // 7bit Slave Address

// Register Address
#define BH1792_ADDR_MANUFACTURERID      (0x0FU)  // R   : Manufacturer ID
#define BH1792_ADDR_PARTID              (0x10U)  // R   : PART ID
#define BH1792_ADDR_RESET               (0x40U)  // R/W : RESET
#define BH1792_ADDR_MEAS_CTRL1          (0x41U)  // R/W : Measurement Control1
#define BH1792_ADDR_MEAS_CTRL2          (0x42U)  // R/W : Measurement Control2
#define BH1792_ADDR_MEAS_CTRL3          (0x43U)  // R/W : Measurement Control3
#define BH1792_ADDR_MEAS_CTRL4_LSBS     (0x44U)  // R/W : Lower byte of Measurement Control4
#define BH1792_ADDR_MEAS_CTRL4_MSBS     (0x45U)  // R/W : Upper byte of Measurement Control4
#define BH1792_ADDR_MEAS_CTRL5          (0x46U)  // R/W : Measurement Control5
#define BH1792_ADDR_MEAS_START          (0x47U)  // R/W : Measurement Start
#define BH1792_ADDR_MEAS_SYNC           (0x48U)  // W   : Measurement Synchronization
#define BH1792_ADDR_FIFO_LEV            (0x4BU)  // R   : FIFO Level
#define BH1792_ADDR_FIFO_DATA0_LSBS     (0x4CU)  // R   : Lower byte of FIFO Data0
#define BH1792_ADDR_FIFO_DATA0_MSBS     (0x4DU)  // R   : Upper byte of FIFO Data0
#define BH1792_ADDR_FIFO_DATA1_LSBS     (0x4EU)  // R   : Lower byte of FIFO Data1
#define BH1792_ADDR_FIFO_DATA1_MSBS     (0x4FU)  // R   : Upper byte of FIFO Data1
#define BH1792_ADDR_IRDATA_LEDOFF_LSBS  (0x50U)  // R   : Lower byte of IRDATA LEDOFF
#define BH1792_ADDR_IRDATA_LEDOFF_MSBS  (0x51U)  // R   : Upper byte of IRDATA LEDOFF
#define BH1792_ADDR_IRDATA_LEDON_LSBS   (0x52U)  // R   : Lower byte of IRDATA LEDON
#define BH1792_ADDR_IRDATA_LEDON_MSBS   (0x53U)  // R   : Upper byte of IRDATA LEDON
#define BH1792_ADDR_GDATA_LEDOFF_LSBS   (0x54U)  // R   : Lower byte of GDATA LEDOFF
#define BH1792_ADDR_GDATA_LEDOFF_MSBS   (0x55U)  // R   : Upper byte of GDATA LEDOFF
#define BH1792_ADDR_GDATA_LEDON_LSBS    (0x56U)  // R   : Lower byte of GDATA LEDON
#define BH1792_ADDR_GDATA_LEDON_MSBS    (0x57U)  // R   : Upper byte of GDATA LEDON
#define BH1792_ADDR_INT_CLEAR           (0x58U)  // R   : Interrupt Clear

// Register Parameters
// MANUFACTURER ID (0Fh)
#define BH1792_PRM_MANUFACTURERID       (0xE0U)

// PART ID (10h)
#define BH1792_PRM_PARTID               (0x0EU)

// RESET (40h)
// 7bit:   SWRESET
#define BH1792_PRM_SWRESET              (0x01U)  // Software reset is performed

// MEAS_CTRL1 (41h)
// 7bit:   RDY
#define BH1792_PRM_RDY                  (0x01U)  // OSC block is active
// 6bit:   SEL_LED
#define BH1792_PRM_SEL_LED              (0x00U)  // LED1=Green, LED2=Green, LED3=IR
// 4bit:   SEL_ADC
#define BH1792_PRM_SEL_ADC_GREEN        (0x00U)  // ADC1=Green
#define BH1792_PRM_SEL_ADC_IR           (0x01U)  // ADC2=IR
// 2-0bit: MSR
#define BH1792_PRM_MSR_32HZ             (0x00U)  // LED_Freq= 128Hz, OUTPUT_Freq=  32Hz, FIFO=Stored
#define BH1792_PRM_MSR_128HZ            (0x01U)  // LED_Freq= 128Hz, OUTPUT_Freq= 128Hz, FIFO=Stored
#define BH1792_PRM_MSR_64HZ             (0x02U)  // LED_Freq= 256Hz, OUTPUT_Freq=  64Hz, FIFO=Stored
#define BH1792_PRM_MSR_256HZ            (0x03U)  // LED_Freq= 256Hz, OUTPUT_Freq= 256Hz, FIFO=Stored
#define BH1792_PRM_MSR_1024HZ           (0x05U)  // LED_Freq=1024Hz, OUTPUT_Freq=1024Hz, FIFO=Stored
#define BH1792_PRM_MSR_NON_SYNC         (0x06U)  // LED_Freq=   4Hz, OUTPUT_Freq=   4Hz, FIFO=Not Stored
#define BH1792_PRM_MSR_SINGLE           (0x07U)  // LED_Freq=   -Hz, OUTPUT_Freq=   -Hz, FIFO=Not Stored

// MEAS_CTRL2 (42h)
// 7-6bit: LED_EN1
#define BH1792_PRM_LED_EN1_0            (0x00U)  // Selection of LED Driver Mode
#define BH1792_PRM_LED_EN1_1            (0x01U)
#define BH1792_PRM_LED_EN1_2            (0x02U)
#define BH1792_PRM_LED_EN1_3            (0x03U)
// 5-0bit: LED_CURRENT1
#define BH1792_PRM_LED_CUR1_MIN         (0x00U)  // LED1 MIN Current  :  0mA
#define BH1792_PRM_LED_CUR1_MAX         (0x3FU)  // LED1 MAX Current  : 63mA
#define BH1792_PRM_LED_CUR1_MA(x)       (x)      // LED1 Current Range: 0-63mA

// MEAS_CTRL3 (43h)
// 7bit:   LED_EN2
#define BH1792_PRM_LED_EN2_0            (0x00U)  // Selection of LED Driver Mode
#define BH1792_PRM_LED_EN2_1            (0x01U)
// 5-0bit: LED_CURRENT2
#define BH1792_PRM_LED_CUR2_MIN         (0x00U)  // LED2 MIN Current  :  0mA
#define BH1792_PRM_LED_CUR2_MAX         (0x3FU)  // LED2 MAX Current  : 63mA
#define BH1792_PRM_LED_CUR2_MA(x)       (x)      // LED2 Current Range: 0-63mA

// MEAS_CTRL5 (46h)
// 1-0bit: INT_SEL
#define BH1792_PRM_INT_SEL_DISABLE      (0x00U)  // No interrupt
#define BH1792_PRM_INT_SEL_WTM          (0x01U)  // FIFO watermark interrupt
#define BH1792_PRM_INT_SEL_IR           (0x02U)  // IR Threshold interrupt of Non-Sync Measurement Mode
#define BH1792_PRM_INT_SEL_SGL          (0x03U)  // Completion interrupt of Single Measurement Mode

// MEAS_START (47h)
// 0bit: MEAS_ST
#define BH1792_PRM_MEAS_ST              (0x01U)  // Measurement start

// MEAS_SYNC (48h)
// 0bit: MEAS_SYNC
#define BH1792_PRM_MEAS_SYNC            (0x01U)  // Measurement synchronization

// FIFO_LEVEL (4Bh)
// 5-0bit: FIFO_LEV
#define BH1792_PRM_FIFO_LEV_EMPTY       (0x00U)  // FIFO Length = 0  (Empty)
#define BH1792_PRM_FIFO_LEV_WTM         (0x20U)  // FIFO Length = 32 (Watermark)
#define BH1792_PRM_FIFO_LEV_FULL        (0x23U)  // FIFO Length = 35 (Full)

// Paramter Type
//#define BH1792_PRM_CTRL1_FREQ          (0U)
//#define BH1792_PRM_CTRL1_RCYCLE        (1U)
#define BH1792_PRM_CTRL1_MSR           (0U)
#define BH1792_PRM_CTRL2_EN1           (1U)
//#define BH1792_PRM_CTRL2_ONTIME        (3U)
#define BH1792_PRM_CTRL2_CUR_LED1      (2U)
#define BH1792_PRM_CTRL3_EN2           (3U)
#define BH1792_PRM_CTRL3_CUR_LED2      (4U)

//===============================================================================
//  Type Definitions
//===============================================================================
// BH1792 Measurment Data
/*
typedef struct {
    uint16_t       on;
    uint16_t       off;
} u16_pair_t;
*/
//extern typedef struct u16_pair_t; // pair of 16bit unsigned integer type

typedef struct {
    uint32_t       on;
    uint32_t       off;
} u32_pair_t;


// BH1792 Data
typedef struct {
    u16_pair_t     ir;
    u16_pair_t     green;
    u16_pair_t     fifo[35];
    u16_pair_t     fifo_lpf[35];
    uint8_t        fifo_lev;
} bh1792_data_t;

// Function Pointer
// I2C Write
typedef int32_t (*wr_func)(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint8_t size);
// I2C Read
typedef int32_t (*rd_func)(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint8_t size);

// BH1792 Register Parameters
typedef struct {
    uint8_t        sel_adc;
    uint8_t        msr;
    uint8_t        led_en;       // Combination of LED_ENx:  (LED_EN1[1:0] << 1) | LED_EN2
    uint8_t        led_cur1;
    uint8_t        led_cur2;
    uint16_t       ir_th;
    uint8_t        int_sel;
} bh1792_prm_t;

// BH1792 Moving Average Parameters
typedef struct {
    u16_pair_t     buf[8];
    u32_pair_t     sum;
    int8_t         pos;
    int8_t         len;
    int8_t         num;
} bh1792_maPrm_t;

// BH1792 Configuration
typedef struct {
    bh1792_prm_t   prm;
    bh1792_maPrm_t ma_prm;
    int32_t        i2c_err;
    int8_t         is_measuring;
    int8_t         sync_seq;
    wr_func        fnWrite;
    rd_func        fnRead;
} bh1792_t;


//===============================================================================
//  Extern Global Variables
//===============================================================================


//===============================================================================
//  Public Function Prototypes
//===============================================================================
#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

extern int32_t bh1792_Init(bh1792_t *pBH1792);
extern int32_t bh1792_SetParams(void);
extern int32_t bh1792_StartMeasure(void);
extern int32_t bh1792_StopMeasure(void);
extern int32_t bh1792_GetMeasData(bh1792_data_t *data);
extern int32_t bh1792_SetSync(void);
extern int32_t bh1792_ClearFifoData(void);

#ifdef __cplusplus
}
#endif  // __cplusplus


//===============================================================================
//  Macros
//===============================================================================



#endif  // BH1792_H
