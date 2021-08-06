/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
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
 * @defgroup main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 * This file has been modified for IO of the sensor LIS3DH
 *
 */

#include <math.h>
#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_clock.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif


/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Timer define */
APP_TIMER_DEF(m_timer);

/*         Sensor defines       */
#define LIS3DH_ADDRESS  0x18

#define LIS3DH_TEMP_CFG_REG           0x1F
#define LIS3DH_CTRL_REG1              0x20
#define LIS3DH_CTRL_REG2              0x21
#define LIS3DH_CTRL_REG3              0x22
#define LIS3DH_CTRL_REG4              0x23
#define LIS3DH_CTRL_REG5              0x24
#define LIS3DH_CTRL_REG6              0x25
#define LIS3DH_REFERENCE              0x26
#define LIS3DH_STATUS_REG2            0x27
#define LIS3DH_OUT_X_L                0x28
#define LIS3DH_OUT_X_H                0x29
#define LIS3DH_OUT_Y_L                0x2A
#define LIS3DH_OUT_Y_H                0x2B
#define LIS3DH_OUT_Z_L                0x2C
#define LIS3DH_OUT_Z_H                0x2D
#define LIS3DH_FIFO_CTRL_REG          0x2E
#define LIS3DH_FIFO_SRC_REG           0x2F
#define LIS3DH_INT1_CFG               0x30
#define LIS3DH_INT1_SRC               0x31
#define LIS3DH_INT1_THS               0x32
#define LIS3DH_INT1_DURATION          0x33
#define LIS3DH_CLICK_CFG              0x38
#define LIS3DH_CLICK_SRC              0x39
#define LIS3DH_CLICK_THS              0x3A
#define LIS3DH_TIME_LIMIT             0x3B
#define LIS3DH_TIME_LATENCY           0x3C
#define LIS3DH_TIME_WINDOW            0x3D

#define LIS3DH_1_3KHZ_NORMAL_MODE      0x97
#define LIS3DH_400HZ_NORMAL_MODE       0x77
#define LIS3DH_1HZ_NORMAL_MODE         0x17

#define VIBRATION_ON_THRESHOLD         500


/*   data buffer defines and global variables */
#define DATA_BUFF_SIZE                 10

uint16_t g_data_buff[DATA_BUFF_SIZE] = {0};
uint8_t  g_data_buff_index = 0;
uint8_t  g_data_buff_full_flag = 0;


/* Functions forward declaration    */
static void read_sensor_to_buff(void);
static void sensor_set_mode(void);
static float calculate_stddev(uint16_t *p_data, uint8_t num_data_points);
static void twi_init (void);
static void lfclk_init(void);
static void led_init(void);
static void led_1_turn_ON(void);
static void led_1_turn_OFF(void);
static void timer_start(void);
static void timer_handler(void * p_context);

/**
 * @brief TWI initialization.
 */
static void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

/**
 * @brief Function for intializing low frequency clock
 */

static void lfclk_init(void)
{
    uint32_t err_code;
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

/**
 * @brief Function for creating and starting a 200ms timer
 */
static void timer_start(void)
{
    ret_code_t err_code;

    err_code = app_timer_create(&m_timer, APP_TIMER_MODE_REPEATED, timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_timer, APP_TIMER_TICKS(200), NULL);
    APP_ERROR_CHECK(err_code);
}


/**
 * @brief Function for handling the timer run
 *     sensor read, led handling and data processing is done in this context
 */
static void timer_handler(void * p_context)
{
    float vib_intensity_calc = 0;
    
    read_sensor_to_buff();

    if(1 == g_data_buff_full_flag)
    {
        g_data_buff_full_flag = 0;
        vib_intensity_calc = calculate_stddev(g_data_buff,DATA_BUFF_SIZE);
        
        NRF_LOG_INFO( "Vibration intensity value using standard deviation of z-axis data: " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(vib_intensity_calc));
        NRF_LOG_FLUSH();

        /*Note since multiple samples are processed for the vibration intensity,
            the hysterisis is inherent*/
        if(vib_intensity_calc > VIBRATION_ON_THRESHOLD)
        {
            led_1_turn_ON();
        }
        else
        {
            led_1_turn_OFF();
        }
    }
}


/**@brief  Function for initializing the LED state
 */
static void led_init(void)
{
    /* nrf_gpio_pin_set: LED Off
       nrf_gpio_pin_clear: LED On */
    nrf_gpio_cfg_output(LED_1);
    nrf_gpio_pin_set(LED_1);
    nrf_gpio_cfg_output(LED_2);
    nrf_gpio_pin_set(LED_2);
    nrf_gpio_cfg_output(LED_3);
    nrf_gpio_pin_set(LED_3);    
    nrf_gpio_cfg_output(LED_4);
    nrf_gpio_pin_set(LED_4);
}

/**@brief  Function for turning ON the led 1
 */
static void led_1_turn_ON(void)
{
    nrf_gpio_pin_clear(LED_1);
}

/**@brief  Function for turning OFF the led 1
 */
static void led_1_turn_OFF(void)
{
    nrf_gpio_pin_set(LED_1);
}

/**
 * @brief Sets the sensor mode to 1.3KHz data rate normal mode
 */

static void sensor_set_mode(void)
{
    ret_code_t err_code;
    
    uint8_t i2c_cmd[2] = {LIS3DH_CTRL_REG1,LIS3DH_1_3KHZ_NORMAL_MODE};
    err_code = nrf_drv_twi_tx(&m_twi, LIS3DH_ADDRESS, i2c_cmd, sizeof(i2c_cmd),false);
    APP_ERROR_CHECK(err_code);

}


/**
 * @brief Reads sensor data to linear buffer
 */

static void read_sensor_to_buff(void)
{
    uint8_t cmnd = 0;
    uint8_t sample_data = 0;
    uint16_t data_val = 0;
    ret_code_t err_code;
    
    cmnd = LIS3DH_OUT_Z_H;
    err_code = nrf_drv_twi_tx(&m_twi, LIS3DH_ADDRESS, &cmnd, sizeof(cmnd),false);
    APP_ERROR_CHECK(err_code);
     

    err_code = nrf_drv_twi_rx(&m_twi, LIS3DH_ADDRESS, &sample_data, 1);
    APP_ERROR_CHECK(err_code);    
    data_val =  sample_data<< 8;

    cmnd = LIS3DH_OUT_Z_L;
    err_code = nrf_drv_twi_tx(&m_twi, LIS3DH_ADDRESS, &cmnd, sizeof(cmnd),false);
    APP_ERROR_CHECK(err_code);
  
    err_code = nrf_drv_twi_rx(&m_twi, LIS3DH_ADDRESS, &sample_data, 1);
    APP_ERROR_CHECK(err_code);
    data_val =  data_val | sample_data;

    g_data_buff[g_data_buff_index] = data_val;
    g_data_buff_index++;
    
    if(g_data_buff_index >= DATA_BUFF_SIZE)
    {
        g_data_buff_index = 0;
        g_data_buff_full_flag = 1;
    }
}


/**
 * @brief Computation of std deviation of data
 */
static float calculate_stddev(uint16_t *p_data, uint8_t num_data_points)
{
    float sum = 0.0, mean, stddev = 0.0;
    int i;
    
    for (i = 0; i < num_data_points; i++) {
        sum += p_data[i];
    }
    mean = sum / 10;
    for (i = 0; i < 10; ++i) {
        stddev += pow(p_data[i] - mean, 2);
    }
    return sqrt(stddev / 10);
}


/**
 * @brief Function for main application entry.
 */
int main(void)
{
    ret_code_t err_code;
     
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    lfclk_init();
    twi_init();
    sensor_set_mode();
    led_init();
    
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    
    timer_start();

    while (true)
    {
        __WFE;
    }
}

/** @} */
