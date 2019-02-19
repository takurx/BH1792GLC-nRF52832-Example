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
#include "nrf_drv_gpiote.h"
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include <bh1792.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_timer.h"

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

/* Common addresses definition for temperature sensor. */
#define LM75B_ADDR          (0x90U >> 1)
// #define BH1792GLC_ADDR      0x5BU -> bh1792.h BH1792_SLAVE_ADDR

#define LM75B_REG_TEMP      0x00U
#define LM75B_REG_CONF      0x01U
#define LM75B_REG_THYST     0x02U
#define LM75B_REG_TOS       0x03U

/* Mode for LM75B. */
#define NORMAL_MODE 0U

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
//APP_TIMER_DEF(m_battery_timer_id);                                                  /**< Battery timer. */
//BLE_BAS_DEF(m_bas);  

#define BH1792GLC_MEAS_INTERVAL         APP_TIMER_TICKS(32)
//#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(2000)                       /**< Battery level measurement interval (ticks). */
//#define MIN_BATTERY_LEVEL               81                                          /**< Minimum battery level as returned by the simulated measurement function. */
//#define MAX_BATTERY_LEVEL               100                                         /**< Maximum battery level as returned by the simulated measurement function. */
//#define BATTERY_LEVEL_INCREMENT         1  

/* Indicates if operation on TWI has ended. */
//static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from temperature sensor. */
//static uint8_t m_sample;

/**
 * @brief Function for setting active mode on MMA7660 accelerometer.
 */
/*
void LM75B_set_mode(void)
{
    ret_code_t err_code;

    // Writing to LM75B_REG_CONF "0" set temperature sensor in NORMAL mode. 
    uint8_t reg[2] = {LM75B_REG_CONF, NORMAL_MODE};
    err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg, sizeof(reg), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    // Writing to pointer byte. 
    reg[0] = LM75B_REG_TEMP;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg, 1, false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}
*/

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
/*
__STATIC_INLINE void data_handler(uint8_t temp)
{
    NRF_LOG_INFO("Temperature: %d Celsius degrees.", temp);
}
*/

/**
 * @brief TWI events handler.
 */
/*
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(m_sample);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}
*/

bh1792_t      m_bh1792;
bh1792_data_t m_bh1792_dat;

int32_t i2c_write(uint8_t slv_adr, uint8_t reg_adr, uint8_t *reg, uint8_t reg_size);
int32_t i2c_read(uint8_t slv_adr, uint8_t reg_adr, uint8_t *reg, uint8_t reg_size);

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;
    int32_t ret = 0;
  
    /*
    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };
    
    err_code = nrf_drv_twi_init(&m_twi, &twi_lm75b_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);
    */

    const nrf_drv_twi_config_t twi_bh1792glc_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    //err_code = nrf_drv_twi_init(&m_twi, &twi_bh1792glc_config, twi_handler, NULL);
    err_code = nrf_drv_twi_init(&m_twi, &twi_bh1792glc_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    // BH1792
    //m_bh1792.fnWrite      = i2c_write;
    //m_bh1792.fnRead       = i2c_read;
    ret = bh1792_Init(&m_bh1792);
    //error_check(ret, "bh1792_Init");

    m_bh1792.prm.sel_adc  = BH1792_PRM_SEL_ADC_GREEN;
    m_bh1792.prm.msr      = BH1792_PRM_MSR_SINGLE;//BH1792_PRM_MSR_1024HZ;
    m_bh1792.prm.led_en   = (BH1792_PRM_LED_EN1_0 << 1) | BH1792_PRM_LED_EN2_0;
    m_bh1792.prm.led_cur1 = BH1792_PRM_LED_CUR1_MA(1);
    m_bh1792.prm.led_cur2 = BH1792_PRM_LED_CUR2_MA(0);
    m_bh1792.prm.ir_th    = 0xFFFC;
    m_bh1792.prm.int_sel  = BH1792_PRM_INT_SEL_SGL;//BH1792_PRM_INT_SEL_WTM;
    ret = bh1792_SetParams();
    //error_check(ret, "bh1792_SetParams");

    NRF_LOG_INFO("GDATA(@LED_ON),GDATA(@LED_OFF)\n");

    ret = bh1792_StartMeasure();
    //error_check(ret, "bh1792_StartMeasure");

    //attachInterrupt(0, bh1792_isr, LOW);

    //FlexiTimer2::stop();
    /*
    if (m_bh1792.prm.msr <= BH1792_PRM_MSR_1024HZ) {
      //FlexiTimer2::set(2000, 5.0/10000, timer_isr);    // 1Hz timer
    } else {
      //FlexiTimer2::set(250, 1.0/8000, timer_isr);      // 32Hz timer
    }
    //FlexiTimer2::start();
    */

    nrf_drv_twi_enable(&m_twi);
}

//void timer_isr(void)
static void timer_isr(void * p_context)
{
    int32_t ret = 0;
    //uint8_t tmp_eimsk;

    //tmp_eimsk = EIMSK; //EIMSK Enable Interrupt MaSK register, set:1 enable, set:0 disable
    //EIMSK = 0; //EIMSK Enable Interrupt MaSK register, set:1 enable, set:0 disable
    //interrupts(); // enable interrupt

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
      ret = bh1792_StartMeasure();
      //error_check(ret, "bh1792_StartMeasure");
    }

    //noInterrupts(); // disable interrupt
    //EIMSK |= tmp_eimsk; // undo Enable Interrupt MaSK register
}

//void bh1792_isr(void)
void bh1792_isr(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    int32_t ret = 0;
    uint8_t i   = 0;

    //EIMSK = 0; //EIMSK Enable Interrupt MaSK register, set:1 enable, set:0 disable
    //interrupts(); // enable interrupt

    ret = bh1792_GetMeasData(&m_bh1792_dat);
    //error_check(ret, "bh1792_GetMeasData");

    if(m_bh1792.prm.msr <= BH1792_PRM_MSR_1024HZ) {
      for (i = 0; i < m_bh1792_dat.fifo_lev; i++) {
        NRF_LOG_INFO("%d", m_bh1792_dat.fifo[i].on);
        NRF_LOG_INFO(",");
        NRF_LOG_INFO("%d\n", m_bh1792_dat.fifo[i].off);
      }
    } else {
      if(m_bh1792.prm.sel_adc == BH1792_PRM_SEL_ADC_GREEN) {
        NRF_LOG_INFO("%d", m_bh1792_dat.green.on);
        NRF_LOG_INFO(",");
        NRF_LOG_INFO("%d\n", m_bh1792_dat.green.off);
      } else {
        NRF_LOG_INFO("%d", m_bh1792_dat.ir.on);
        NRF_LOG_INFO(",");
        NRF_LOG_INFO("%d\n", m_bh1792_dat.ir.off);
      }
    }

    //noInterrupts(); // disable interrupt
    //EIMSK = bit(INT0); // set INT0 Enable Interrupt MaSK register
}

// Note:  I2C access should be completed within 0.5ms
int32_t i2c_write(uint8_t slv_adr, uint8_t reg_adr, uint8_t *reg, uint8_t reg_size)
{
    //byte rc;
    uint8_t rc;
    ret_code_t err_code;

    /*
    if (m_bh1792.prm.msr <= BH1792_PRM_MSR_1024HZ) {
      if((slv_adr != BH1792_SLAVE_ADDR) || (reg_adr != BH1792_ADDR_MEAS_SYNC)) {
        while(FlexiTimer2::count == 1999);
      }
    }

    Wire.beginTransmission(slv_adr);
    Wire.write(reg_adr);
    Wire.write(reg, reg_size);
    rc = Wire.endTransmission(true);
    */

    err_code = nrf_drv_twi_tx(&m_twi, slv_adr, reg, reg_size, false);
    APP_ERROR_CHECK(err_code);

    //return rc;
    return 0;
}

// Note:  I2C access should be completed within 0.5ms
int32_t i2c_read(uint8_t slv_adr, uint8_t reg_adr, uint8_t *reg, uint8_t reg_size)
{
    //byte rc;
    //uint8_t rc;
    //uint8_t cnt;
    ret_code_t err_code;

    /*
    if (m_bh1792.prm.msr <= BH1792_PRM_MSR_1024HZ) {
      while(FlexiTimer2::count == 1999);
    }
    */

    /*
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
    */
    err_code = nrf_drv_twi_rx(&m_twi, slv_adr, reg, reg_size);
    APP_ERROR_CHECK(err_code);

    //return rc;
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

/**
 * @brief Function for reading data from temperature sensor.
 */
/*
static void read_sensor_data()
{
    m_xfer_done = false;

    // Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature.
    ret_code_t err_code = nrf_drv_twi_rx(&m_twi, LM75B_ADDR, &m_sample, sizeof(m_sample));
    APP_ERROR_CHECK(err_code);
}
*/

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

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
static void bh1792glc_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    ret_code_t err_code;
    //uint8_t  battery_level;

    NRF_LOG_INFO("\r\nbh1792glc measure timer interrupt.");
    /*
    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) &&
        (err_code != NRF_ERROR_FORBIDDEN)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
    */
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
    /*
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
                                */
                                /*
    err_code = app_timer_create(&m_bh1792glc_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                bh1792glc_meas_timeout_handler);
                                */

    err_code = app_timer_create(&m_bh1792glc_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_isr);
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

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    
    timers_init();
    gpio_init();

    NRF_LOG_INFO("\r\nTWI sensor example started.");
    NRF_LOG_FLUSH();
    twi_init();
    application_timers_start();
    //LM75B_set_mode();

    while (true)
    {
        // Do nothing.
        /*
        //nrf_delay_ms(500);

        do
        {
            __WFE();
        }while (m_xfer_done == false);

        //read_sensor_data();
        NRF_LOG_FLUSH();
        */
    }
}

/** @} */
