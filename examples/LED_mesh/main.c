/***********************************************************************************
Copyright (c) Nordic Semiconductor ASA
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of other
  contributors to this software may be used to endorse or promote products
  derived from this software without specific prior written permission.

  4. This software must only be used in a processor manufactured by Nordic
  Semiconductor ASA, or in a processor manufactured by a third party that
  is used in combination with a processor manufactured by Nordic Semiconductor.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************/

#include "rbc_mesh.h"

#include "led_config.h"

#include "nrf_soc.h"
#include "nrf_assert.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "ble_advdata.h"
#include "nrf_adv_conn.h"
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "timeslot_handler.h"
#include "bsp.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define BLE_ADV_INTERVAL_750MS (800)
#define APP_TIMER_PRESCALER                  0   

/* Debug macros for debugging with logic analyzer */
#define SET_PIN(x) NRF_GPIO->OUTSET = (1 << (x))
#define CLEAR_PIN(x) NRF_GPIO->OUTCLR = (1 << (x))
#define TICK_PIN(x) do { SET_PIN((x)); CLEAR_PIN((x)); }while(0)

#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */

#define DFU_REV_MAJOR                        0x00                                       /** DFU Major revision number to be exposed. */
#define DFU_REV_MINOR                        0x01                                       /** DFU Minor revision number to be exposed. */
#define DFU_REVISION                         ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)     /** DFU Revision number to be exposed. Combined of major and minor versions. */



static ble_dfu_t m_dfus;

/**
* @brief General error handler. Sets the LEDs to blink forever
*/
static void error_loop(void)
{
    SET_PIN(7);
    while (true)
    {
        nrf_delay_ms(500);
        SET_PIN(LED_0);
        CLEAR_PIN(LED_1);
        nrf_delay_ms(500);
        SET_PIN(LED_1);
        CLEAR_PIN(LED_0);
    }
}    

/**
* @brief Softdevice crash handler, never returns
* 
* @param[in] pc Program counter at which the assert failed
* @param[in] line_num Line where the error check failed 
* @param[in] p_file_name File where the error check failed
*/
void sd_assert_handler(uint32_t pc, uint16_t line_num, const uint8_t* p_file_name)
{
    error_loop();
}

/**
* @brief App error handle callback. Called whenever an APP_ERROR_CHECK() fails.
*   Never returns.
* 
* @param[in] error_code The error code sent to APP_ERROR_CHECK()
* @param[in] line_num Line where the error check failed 
* @param[in] p_file_name File where the error check failed
*/
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    error_loop();
}

void HardFault_Handler(void)
{
    NVIC_SystemReset();
}

/**
* @brief Softdevice event handler 
*/
/*void SD_EVT_IRQHandler(void)
{
    rbc_mesh_sd_irq_handler();
    
    ble_evt_t ble_evt;
    uint16_t len = sizeof(ble_evt);
    while (sd_ble_evt_get((uint8_t*) &ble_evt, &len) == NRF_SUCCESS)
    {
        nrf_adv_conn_evt_handler(&ble_evt);
    }
}*/

/**
* @brief RBC_MESH framework event handler. Defined in rbc_mesh.h. Handles
*   events coming from the mesh. Sets LEDs according to data
*
* @param[in] evt RBC event propagated from framework
*/
void rbc_mesh_event_handler(rbc_mesh_event_t* evt)
{
    //TICK_PIN(28);
    switch (evt->event_type)
    {
        case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:   
        case RBC_MESH_EVENT_TYPE_NEW_VAL:
        case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
        
            if (evt->value_handle > 2)
                break;
            
            led_config(evt->value_handle, evt->data[0]);
            break;
        /*case RBC_MESH_EVENT_TYPE_INITIALIZED:
             init BLE gateway softdevice application: 
            nrf_adv_conn_init();
            break;  */
    }
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
		//led_config(2, 1);
		rbc_mesh_sys_evt_handler(sys_evt);
}


#ifdef BOARD_PCA10001
/* configure button interrupt for evkits */
static void gpiote_init(void)
{
    NRF_GPIO->PIN_CNF[BUTTON_0] = (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos)
                                | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                | (BUTTON_PULL << GPIO_PIN_CNF_PULL_Pos)
                                | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
    
    NRF_GPIO->PIN_CNF[BUTTON_1] = (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos)
                                | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                | (BUTTON_PULL << GPIO_PIN_CNF_PULL_Pos)
                                | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
    

    /* GPIOTE interrupt handler normally runs in STACK_LOW priority, need to put it 
    in APP_LOW in order to use the mesh API */
    NVIC_SetPriority(GPIOTE_IRQn, 3);
    
    NVIC_EnableIRQ(GPIOTE_IRQn);
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
}
#endif

void test_app_init(void)
{   
    nrf_gpio_cfg_output(LED_0);
    nrf_gpio_cfg_output(LED_1);  
    
#ifdef BOARD_PCA10000
    nrf_gpio_pin_clear(LED_RGB_RED);
    nrf_gpio_pin_clear(LED_RGB_GREEN);
    nrf_gpio_pin_clear(LED_RGB_BLUE);
#endif

#ifdef BOARD_PCA10001 
    nrf_gpio_range_cfg_output(0, 32);
    nrf_gpio_cfg_input(BUTTON_0, BUTTON_PULL);
    nrf_gpio_cfg_input(BUTTON_1, BUTTON_PULL);  
    //gpiote_init();
#endif    

    led_config(1, 0);
    led_config(2, 0);
}

/*static void advertising_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);
}*/

static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
		//led_config(1, 1);
		nrf_adv_conn_evt_handler(p_ble_evt);
		ble_conn_params_on_ble_evt(p_ble_evt);
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);  
    //on_ble_evt(p_ble_evt);
}

static void reset_prepare(void)
{
    uint32_t err_code;
    
    /*if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        err_code = bsp_indication_set(BSP_INDICATE_IDLE);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // If not connected, then the device will be advertising. Hence stop the advertising.
        advertising_stop();
    }*/

    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

int main(void)
{
    uint32_t error_code;
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, 0);
		//APP_ERROR_CHECK(error_code);
    
    ble_enable_params_t ble_enable_params;
    ble_enable_params.gatts_enable_params.service_changed = 0;
    
    error_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(error_code);
	
		// Register with the SoftDevice handler module for BLE events.
    error_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(error_code);
		
    // Register with the SoftDevice handler module for BLE events.
    error_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(error_code);
	
    test_app_init();
		//scheduler_init();
	
    rbc_mesh_init_params_t init_params;

    init_params.access_addr = 0xA541A68F;
    init_params.adv_int_ms = 200;
    init_params.channel = 38;
    init_params.handle_count = 12;
    init_params.packet_format = RBC_MESH_PACKET_FORMAT_ORIGINAL;
    init_params.radio_mode = RBC_MESH_RADIO_MODE_BLE_1MBIT;
    
    error_code = rbc_mesh_init(init_params);
    APP_ERROR_CHECK(error_code);
    
    error_code = rbc_mesh_value_enable(1);
    APP_ERROR_CHECK(error_code);
    error_code = rbc_mesh_value_enable(2);
    APP_ERROR_CHECK(error_code);
    error_code = rbc_mesh_value_enable(3);
    APP_ERROR_CHECK(error_code);
    error_code = rbc_mesh_value_enable(4);
    APP_ERROR_CHECK(error_code);  

		uint8_t val = 1;
		rbc_mesh_value_set(1, &val, 0);
    
		/*ble_dfu_init_t   dfus_init;
    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));
    dfus_init.evt_handler    = dfu_app_on_dfu_evt;
    dfus_init.error_handler  = NULL; //service_error_handler - Not used as only the switch from app to DFU mode is required and not full dfu service.
    error_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(error_code);
    
    dfu_app_reset_prepare_set(reset_prepare);
		
    sd_nvic_EnableIRQ(SD_EVT_IRQn);*/
		
		nrf_adv_conn_init();
		
		//sd_nvic_EnableIRQ(SD_EVT_IRQn);
		
		led_config(1, 0);
    led_config(2, 0);
		led_config(3, 0);
    led_config(4, 0);
		
    /* sleep */
    for(;;)
    {
				//app_sched_execute();
        power_manage();
    }
    

}

