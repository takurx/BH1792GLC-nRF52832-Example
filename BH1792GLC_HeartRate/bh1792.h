/////////////////////////////////////////////////////////////////////////////////
// bh1792.h
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
#ifndef _BH1792_H_
#define _BH1792_H_

#include <typeDef.h>


//===============================================================================
//  Constants
//===============================================================================
// Retern Value
#define BH1792_RC_OK                   (0)      // No error
#define BH1792_RC_NO_EXIST             (-1)     // No BH1792 exists
#define BH1792_RC_I2C_ERR              (-2)     // I2C error with BH1792

// Slave Address
#define BH1792_SLAVE_ADDRESS           (0x5BU)  // Slave Address

// Register Address
#define BH1792_PARTID                  (0x10U)  // R   : PART ID
#define BH1792_RESET                   (0x40U)  // W   : RESET
#define BH1792_MEAS_CTRL1              (0x41U)  // R/W : Measurement Control1
#define BH1792_MEAS_CTRL2              (0x42U)  // R/W : Measurement Control2
#define BH1792_MEAS_CTRL3              (0x43U)  // R/W : Measurement Control3
#define BH1792_ADDR_MEAS_CTRL4_LSBS    (0x44U)  // R/W : Lower byte of Measurement Control4
#define BH1792_ADDR_MEAS_CTRL4_MSBS    (0x45U)  // R/W : Upper byte of Measurement Control4
#define BH1792_ADDR_MEAS_CTRL5         (0x46U)  // R/W : Measurement Control5
#define BH1792_MEAS_START              (0x47U)  // R/W : Measurement Start
#define BH1792_ADDR_MEAS_SYNC          (0x48U)  // W   : Measurement Synchronization
#define BH1792_ADDR_FIFO_LEV           (0x4BU)  // R   : FIFO Level
#define BH1792_ADDR_FIFO_DATA0_LSBS    (0x4CU)  // R   : Lower byte of FIFO Data0
#define BH1792_ADDR_FIFO_DATA0_MSBS    (0x4DU)  // R   : Upper byte of FIFO Data0
#define BH1792_ADDR_FIFO_DATA1_LSBS    (0x4EU)  // R   : Lower byte of FIFO Data1
#define BH1792_ADDR_FIFO_DATA1_MSBS    (0x4FU)  // R   : Upper byte of FIFO Data1
#define BH1792_ADDR_IRDATA_LEDOFF_LSBS (0x50U)  // R   : Lower byte of IRDATA LEDOFF
#define BH1792_ADDR_IRDATA_LEDOFF_MSBS (0x51U)  // R   : Upper byte of IRDATA LEDOFF
#define BH1792_ADDR_IRDATA_LEDON_LSBS  (0x52U)  // R   : Lower byte of IRDATA LEDON
#define BH1792_ADDR_IRDATA_LEDON_MSBS  (0x53U)  // R   : Upper byte of IRDATA LEDON
#define BH1792_DATAOUT_LEDOFF_LSBS     (0x54U)  // R   : Low  byte of DATAOUT LEDOFF
#define BH1792_DATAOUT_LEDOFF_MSBS     (0x55U)  // R   : High byte of DATAOUT LEDOFF
#define BH1792_DATAOUT_LEDON_LSBS      (0x56U)  // R   : Low  byte of DATAOUT LEDON
#define BH1792_DATAOUT_LEDON_MSBS      (0x57U)  // R   : High byte of DATAOUT LEDON
#define BH1792_ADDR_INT_CLEAR          (0x58U)  // R   : Interrupt Clear
#define BH1792_MANUFACTURERID          (0x0FU)  // R   : Manufacturer ID

// Register Value
#define BH1792_PARTID_VAL              (0x0EU)
#define BH1792_MANUFACTURERID_VAL      (0xE0U)

// Config Parameters
// BH1792_RESET
// 7bit: SWRESET
#define BH1792_PRM_SWRESET             (0x01U)  // Software reset is performed

// BH1792_MEAS_CTRL1
// 7bit: RDY
#define BH1792_PRM_CTRL1_RDY           (0x01U)  // OSC block is active
// 2bit: LED_LIGHTING_FREQ
// Select LED emitting frequency
//#define BH1792_PRM_CTRL1_FREQ_128HZ    (0x00U)
//#define BH1792_PRM_CTRL1_FREQ_64HZ     (0x01U)
// 4bit: SEL_ADC
#define BH1792_PRM_CTRL1_SEL_ADC       (0x00U)
// 2-0bit: RCYCLE
// Select Data reading frequency
//#define BH1792_PRM_CTRL1_RCYCLE_64HZ   (0x01U)
//#define BH1792_PRM_CTRL1_RCYCLE_32HZ   (0x02U)
// 2-0bit: MSR
#define BH1792_PRM_CTRL1_RCYCLE_32HZ        (0x00U)
#define BH1792_PRM_CTRL1_RCYCLE_128HZ       (0x01U)
#define BH1792_PRM_CTRL1_RCYCLE_64HZ        (0x02U)
#define BH1792_PRM_CTRL1_RCYCLE_256HZ       (0x03U)
#define BH1792_PRM_CTRL1_MSR_NOT_SET_VAL    (0x04U)
#define BH1792_PRM_CTRL1_RCYCLE_1024HZ      (0x05U)
#define BH1792_PRM_CTRL1_ASYNCHRONOUS_4HZ   (0x06U)
#define BH1792_PRM_CTRL1_SINGLE_MEASURE     (0x07U)

// BH1792_MEAS_CTRL2
// 7-6bit: LED_EN1
// Select LED1:3 driver mode
#define BH1792_PRM_CTRL2_EN1_NONE       (0x00U) // LED1: Pulsed,   LED2: Pulsed,    LED3: Pulsed
#define BH1792_PRM_CTRL2_EN1_LED1       (0x01U) // LED1: ON,       LED2: OFF,       LED3: OFF
#define BH1792_PRM_CTRL2_EN1_LED2       (0x02U) // LED1: OFF,      LED2: ON/OFF,    LED3: OFF/ON
#define BH1792_PRM_CTRL2_EN1_LED1_LED2  (0x03U) // LED1: ON,       LED2: ON/OFF,    LED3: OFF/ON
// 5-0bit: LED_CURRENT
// Select LED driver current
#define BH1792_PRM_CTRL2_CUR_0MA        (0x00U)
#define BH1792_PRM_CTRL2_CUR_1MA        (0x01U)
#define BH1792_PRM_CTRL2_CUR_2MA        (0x02U)
#define BH1792_PRM_CTRL2_CUR_3MA        (0x03U)
#define BH1792_PRM_CTRL2_CUR_6MA        (0x06U)
#define BH1792_PRM_CTRL2_CUR_10MA       (0x0AU)
#define BH1792_PRM_CTRL2_CUR_20MA       (0x14U)
#define BH1792_PRM_CTRL2_CUR_30MA       (0x1DU)
#define BH1792_PRM_CTRL2_CUR_60MA       (0x3CU)

// BH1792_MEAS_CTRL3
// 7bit: LED_EN2
// Select LED2:3 driver mode
#define BH1792_PRM_CTRL3_EN2_OFF_LED3   (0x00U) // LED1: ON/OFF,   LED2: ON,        LED3: OFF
#define BH1792_PRM_CTRL3_EN2_ON_LED3    (0x01U) // LED1: ON/OFF,   LED2: OFF,       LED3: ON
// 5-0bit: LED_CURRENT
// Select LED driver current
#define BH1792_PRM_CTRL3_CUR_0MA       (0x00U)
#define BH1792_PRM_CTRL3_CUR_1MA       (0x01U)
#define BH1792_PRM_CTRL3_CUR_2MA       (0x02U)
#define BH1792_PRM_CTRL3_CUR_3MA       (0x03U)
#define BH1792_PRM_CTRL3_CUR_6MA       (0x06U)
#define BH1792_PRM_CTRL3_CUR_10MA      (0x0AU)
#define BH1792_PRM_CTRL3_CUR_20MA      (0x14U)
#define BH1792_PRM_CTRL3_CUR_30MA      (0x1DU)
#define BH1792_PRM_CTRL3_CUR_60MA      (0x3CU)

// BH1792_MEAS_START
// 0bit: MEAS_ST
#define BH1792_PRM_MEAS_ST             (0x01U)  // Measurement start

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
//  Type Definition
//===============================================================================
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
int8_t bh1792_Init(void);
int8_t bh1792_SoftReset(void);
// Please implement the I2C I/F in the following functions
int8_t bh1792_Write(uint8_t adr, uint8_t *data, uint8_t size);
int8_t bh1792_Read (uint8_t adr, uint8_t *data, uint8_t size);


//===============================================================================
//  Macro
//===============================================================================



#endif  // _BH1792_H_
