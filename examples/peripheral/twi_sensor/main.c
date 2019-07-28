/**
 * The 3-Clause BSD License
 * Copyright 2019 takurx
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be
 * used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include <stdbool.h>
#include "nrf.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include <bh1792.h>
#include <hr_bh1792.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_timer.h"
#include "nrf_drv_clock.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

#ifdef BSP_BUTTON_0
    #define PIN_IN BSP_BUTTON_0
#endif
#ifndef PIN_IN
    #error "Please indicate input pin"
#endif

#ifdef BSP_LED_0
    #define PIN_OUT BSP_LED_0
#endif
#ifndef PIN_OUT
    #error "Please indicate output pin"
#endif

APP_TIMER_DEF(m_bh1792glc_timer_id);
//#define BH1792GLC_MEAS_INTERVAL         APP_TIMER_TICKS(1000)   //1 Hz Timer
//#define BH1792GLC_MEAS_INTERVAL         APP_TIMER_TICKS(25)       //40 Hz Timer
#define BH1792GLC_MEAS_INTERVAL         APP_TIMER_TICKS(31)       //32.258 Hz Timer

/* Indicates if operation on TWI has ended (when received). */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

volatile static bool twi_tx_done = false;
volatile static bool twi_rx_done = false;

bh1792_t      m_bh1792;
bh1792_data_t m_bh1792_dat;

int32_t i2c_write(uint8_t slv_adr, uint8_t reg_adr, uint8_t *reg, uint8_t reg_size);
int32_t i2c_read(uint8_t slv_adr, uint8_t reg_adr, uint8_t *reg, uint8_t reg_size);

#define BH1792_TWI_TIMEOUT 			10000 
#define BH1792_TWI_BUFFER_SIZE     	8 // 8byte = tx max(7) + addr(1)

uint8_t twi_tx_buffer[BH1792_TWI_BUFFER_SIZE];


/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            switch(p_event->xfer_desc.type)
            {
                case NRF_DRV_TWI_XFER_TX:
                    twi_tx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_TXTX:
                    twi_tx_done = true;
                    break;
                case NRF_DRV_TWI_XFER_RX:
                    twi_rx_done = true;
                    m_xfer_done = true;
                    break;
                case NRF_DRV_TWI_XFER_TXRX:
                    twi_rx_done = true;
                    m_xfer_done = true;
                    break;
                default:
                    break;
            }
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            break;
        default:
            break;
    }
}


/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;
    int32_t ret = 0;

    const nrf_drv_twi_config_t twi_bh1792glc_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    NRF_LOG_INFO("before nrf_drv_twi_init.");
    err_code = nrf_drv_twi_init(&m_twi, &twi_bh1792glc_config, twi_handler, NULL);
    NRF_LOG_INFO("finished nrf_drv_twi_init.");
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);

    // BH1792
    m_bh1792.fnWrite      = i2c_write;
    m_bh1792.fnRead       = i2c_read;
    ret = bh1792_Reg_Init(&m_bh1792);
    NRF_LOG_INFO("finished bh1792_Reg_Init.");
    //error_check(ret, "bh1792_Reg_Init");

    ret = hr_bh1792_Init();
    NRF_LOG_INFO("finished hr_bh1792_Init.");
    //error_check(ret, "hr_bh1792_Init");

    m_bh1792.prm.sel_adc  = BH1792_PRM_SEL_ADC_GREEN;
    m_bh1792.prm.msr      = BH1792_PRM_MSR_SINGLE;//BH1792_PRM_MSR_1024HZ;
    m_bh1792.prm.led_en   = (BH1792_PRM_LED_EN1_0 << 1) | BH1792_PRM_LED_EN2_0;
    m_bh1792.prm.led_cur1 = BH1792_PRM_LED_CUR1_MA(1);
    m_bh1792.prm.led_cur2 = BH1792_PRM_LED_CUR2_MA(0);
    m_bh1792.prm.ir_th    = 0xFFFC;
    m_bh1792.prm.int_sel  = BH1792_PRM_INT_SEL_SGL;//BH1792_PRM_INT_SEL_WTM;
    NRF_LOG_INFO("before bh1792_SetParams.");
    ret = bh1792_SetParams();
    //error_check(ret, "bh1792_SetParams");
    NRF_LOG_INFO("finished bh1792_SetParams.");

    //NRF_LOG_INFO("GDATA(@LED_ON),GDATA(@LED_OFF)\n");

    //ret = bh1792_StartMeasure();
    //error_check(ret, "bh1792_StartMeasure");
    //NRF_LOG_INFO("finished bh1792_StartMeasure.");

    ret = hr_bh1792_StartMeasure();
    //error_check(ret, "hr_bh1792_StartMeasure");
    NRF_LOG_INFO("finished Hr_bh1792_StartMeasure.");
}


static void timer_isr(void * p_context)
{
    //UNUSED_PARAMETER(p_context);
    //NRF_LOG_INFO("timer_isr.");
    
    int32_t ret = 0;

    nrf_drv_gpiote_in_event_disable(ARDUINO_10_PIN);

    // became else root, m_bh1792.prm.msr = BH1792_PRM_MSR_SINGLE
    /*
    if (m_bh1792.prm.msr <= BH1792_PRM_MSR_1024HZ) {
      ret = bh1792_SetSync();
      //error_check(ret, "bh1792_SetSync");

      if (m_bh1792.sync_seq < 3) {
        if (m_bh1792.sync_seq == 1) {
          //tmp_eimsk = 0;
        } else {
          ret = bh1792_ClearFifoData();
          //error_check(ret, "bh1792_ClearFifoData");

          //tmp_eimsk = bit(INT0);
        }
      }
    } else {
    */
    
      //m_bh1792.prm.led_cur1 = BH1792_PRM_LED_CUR1_MA(0);
      //m_bh1792.prm.led_cur2 = BH1792_PRM_LED_CUR2_MA(0);
      pw_GetParam(BH1792_PRM_CTRL2_CUR_LED1, &(m_bh1792.prm.led_cur1));
      pw_GetParam(BH1792_PRM_CTRL3_CUR_LED2, &(m_bh1792.prm.led_cur2));
      ret = bh1792_SetParams();
      //ret = bh1792_StartMeasure();
      //error_check(ret, "bh1792_StartMeasure");
      ret = hr_bh1792_StartMeasure();
      //error_check(ret, "hr_bh1792_StartMeasure");
    /*
    }
    */

    nrf_drv_gpiote_in_event_enable(ARDUINO_10_PIN, true);
}



static uint8_t    s_cnt_freq = 0;

void bh1792_isr(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    int32_t ret = 0;
    //uint8_t i   = 0;
    u16_pair_t s_pwData_test;
    float32_t pw_test;
    static uint8_t  bpm     = 0U;
    static uint8_t  wearing = 0U;

    nrf_drv_gpiote_in_event_disable(ARDUINO_10_PIN);

    //ret = bh1792_GetMeasData(&m_bh1792_dat);
    //error_check(ret, "bh1792_GetMeasData");
    //ret = hr_bh1792_Calc(s_cnt_freq);
    //s_pwData_test = m_bh1792_dat.green;
    ret = hr_bh1792_Calc(s_cnt_freq, &m_bh1792_dat, &s_pwData_test, &pw_test);
    s_cnt_freq++;
    if (s_cnt_freq >= 31)
    {
        s_cnt_freq = 0;
        hr_bh1792_GetData(&bpm, &wearing);
        //NRF_LOG_RAW_INFO("%d, %d\n", bpm, wearing);
    }
    NRF_LOG_RAW_INFO("%d, %d, %d, %d, ", bpm, wearing, s_pwData_test.on, s_pwData_test.off);
    NRF_LOG_RAW_INFO("" NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(pw_test));
    
    //ret = hr_bh1792_Calc(s_cnt_freq, &s_pwData_test, &pw_test);
    //error_check(ret, "hr_bh1792_Calc");

    // became else root, m_bh1792.prm.msr = BH1792_PRM_MSR_SINGLE
    /*
    if(m_bh1792.prm.msr <= BH1792_PRM_MSR_1024HZ) {
      for (i = 0; i < m_bh1792_dat.fifo_lev; i++) {
        NRF_LOG_INFO("%d", m_bh1792_dat.fifo[i].on);
        NRF_LOG_INFO(",");
        NRF_LOG_INFO("%d\n", m_bh1792_dat.fifo[i].off);
      }
    } else {
      if(m_bh1792.prm.sel_adc == BH1792_PRM_SEL_ADC_GREEN) {
        */
        //NRF_LOG_RAW_INFO("%d,%d,%d,%d\n", m_bh1792_dat.green.on, m_bh1792_dat.green.off, m_bh1792_dat.ir.on, m_bh1792_dat.ir.off)
        //NRF_LOG_RAW_INFO("%d,%d\n", m_bh1792_dat.green.on, m_bh1792_dat.green.off)
        /*
      } else {
        NRF_LOG_RAW_INFO("%d,%d\n", m_bh1792_dat.ir.on, m_bh1792_dat.ir.off)
      }
    }
    */
    nrf_drv_gpiote_in_event_enable(ARDUINO_10_PIN, true);
}


// Note:  I2C access should be completed within 0.5ms
int32_t i2c_write(uint8_t slv_adr, uint8_t reg_adr, uint8_t *reg, uint8_t reg_size)
{
    ret_code_t err_code;

    /*
    // m_bh1792.prm.msr      = BH1792_PRM_MSR_SINGLE, none
    if (m_bh1792.prm.msr <= BH1792_PRM_MSR_1024HZ) {
      if((slv_adr != BH1792_SLAVE_ADDR) || (reg_adr != BH1792_ADDR_MEAS_SYNC)) {
        while(FlexiTimer2::count == 1999);
      }
    }
    */

    uint32_t timeout = BH1792_TWI_TIMEOUT;

    twi_tx_buffer[0] = reg_adr;
    memcpy(&twi_tx_buffer[1], &reg[0], reg_size);
    
    err_code = nrf_drv_twi_tx(&m_twi, slv_adr, &twi_tx_buffer[0], reg_size + 1, false);
    if(err_code != NRF_SUCCESS) return err_code;
    while((!twi_tx_done) && --timeout) ;
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_tx_done = false;

    //return rc;   //rc is return value that arduino, Wire endTransmission, rc:0 is normal
    return 0;
}


// Note:  I2C access should be completed within 0.5ms
int32_t i2c_read(uint8_t slv_adr, uint8_t reg_adr, uint8_t *reg, uint8_t reg_size)
{
    ret_code_t err_code;

    /*
    // m_bh1792.prm.msr      = BH1792_PRM_MSR_SINGLE, none
    if (m_bh1792.prm.msr <= BH1792_PRM_MSR_1024HZ) {
      while(FlexiTimer2::count == 1999);
    }
    */

    uint32_t timeout = BH1792_TWI_TIMEOUT;

    err_code = nrf_drv_twi_tx(&m_twi, slv_adr, &reg_adr, 1, false);
    if(err_code != NRF_SUCCESS) return err_code;

    while((!twi_tx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_tx_done = false;

    err_code = nrf_drv_twi_rx(&m_twi, slv_adr, reg, reg_size);
    if(err_code != NRF_SUCCESS) return err_code;

    timeout = BH1792_TWI_TIMEOUT;
    while((!twi_rx_done) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    twi_rx_done = false;

    //return rc;  //rc:0 is normal, rc:4 is error. but in nrf5 when case of error, already return
    return 0;
}


/*
void error_check(int32_t ret, String msg)
{
  if(ret < 0) {
    msg = "Error: " + msg;
    msg += " function";
    NRF_LOG_INFO("%s\n", msg);
    NRF_LOG_INFO("ret = ");
    NRF_LOG_INFO("%d", ret);
    if(ret == BH1792_I2C_ERR) {
      NRF_LOG_INFO("i2c_ret = ");
      NRF_LOG_INFO("%d\n", m_bh1792.i2c_err);
    }
    while(1);
  }
}
*/


int8_t bh1792_Write(uint8_t adr, uint8_t *data, uint8_t size)
{
  int8_t rc  = 0;
  int8_t ret = 0;
  
  rc = i2c_write(BH1792_SLAVE_ADDR, adr, data, size);
  if (rc == 0) {
    ret = BH1792_SUCCESS;
  } else {
    ret = BH1792_NOT_EXIST;
  }

  return (ret);
}

int8_t bh1792_Read(uint8_t adr, uint8_t *data, uint8_t size)
{
  int8_t rc  = 0;
  int8_t ret = 0;

  rc = i2c_read(BH1792_SLAVE_ADDR, adr, data, size);
  if (rc == 0) {
    ret = BH1792_SUCCESS;
  } else {
    ret = BH1792_NOT_EXIST;
  }
  
  return (ret);
}

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    nrf_drv_gpiote_out_toggle(PIN_OUT);
}


/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    // LED1
    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config);
    APP_ERROR_CHECK(err_code);

    // Button1
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_IN, true);

    // bh1792glc, arudino_10_pin
    nrf_drv_gpiote_in_config_t in_config_bh1792 = GPIOTE_CONFIG_IN_SENSE_HITOLO(true); // interrupt when falling edge
    in_config_bh1792.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(ARDUINO_10_PIN, &in_config_bh1792, bh1792_isr);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(ARDUINO_10_PIN, true);
}


/**@brief Function for handling the BH1792GLC measurement timer timeout.
 *
 * @details This function will be called each time BH1792GLC measurement timer expires.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
static void bh1792glc_meas_timeout_handler(void * p_context)
{
    //NRF_LOG_INFO("bh1792glc measure timer interrupt.");
    timer_isr(p_context);    
}


/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.                                
    err_code = app_timer_create(&m_bh1792glc_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                bh1792glc_meas_timeout_handler);
                                
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_bh1792glc_timer_id, BH1792GLC_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function starting the internal LFCLK oscillator.
 *
 * @details This is needed by RTC1 which is used by the Application Timer
 *          (When SoftDevice is enabled the LFCLK is always running and this is not needed).
 */
static void lfclk_request(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}


/**
 * @brief Function for main application entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    lfclk_request();

    gpio_init();
    timers_init();
    
    NRF_LOG_INFO("TWI sensor example started.");
    NRF_LOG_FLUSH();
    twi_init();
    NRF_LOG_INFO("finished twi init.");
    application_timers_start();
    NRF_LOG_INFO("application_timers start.");

    while (true)
    {    
        do
        {
            __WFE();
        }while (m_xfer_done == false);
        
        NRF_LOG_FLUSH();
        m_xfer_done = false;
    }
}
