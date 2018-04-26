/*UVOS*/

/* This file is part of TelemetrySystem.
 *
 * TelemetrySystem is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License (LGPL) version 3 as published by
 * the Free Software Foundation.
 *
 * TelemetrySystem is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with TelemetrySystem.  If not, see <http://www.gnu.org/licenses/>.
*/

extern "C"
{
    #define __STATIC_INLINE static inline
    #define __STDC_CONSTANT_MACROS
    
    //Standard libraries
    #include <stdint.h>
    #include <stddef.h>
    #include <string.h>

    //SDK headers
    #include "nrf.h"
    #include "nrf_gpiote.h"
    #include "nrf_gpio.h"
    #include "nrf_delay.h"
    #include "nrf_drv_gpiote.h"
    #include "app_error.h"
    #include "softdevice_handler.h"
   
}

#include "board.h"

#ifdef SERIAL_ENABLE
#include "serial.h"
#endif

#include "adc.h"
#include "point3D.h"
#include "twiCshm.h"
#include "MPU9150.h"
#include "MCP4725.h"
#include "btle.h"
#include "cal.h"
#include "sampler.h"
#include "dispatch.h"

void debugBlink()
{
    nrf_drv_gpiote_out_toggle(LED_PIN);
    nrf_delay_ms(200);
    nrf_drv_gpiote_out_toggle(LED_PIN);
    nrf_delay_ms(200);
}

void initGpio()
{
    if(!nrf_drv_gpiote_is_init())nrf_drv_gpiote_init();
    nrf_drv_gpiote_out_config_t pinconf;
    pinconf.init_state = NRF_GPIOTE_INITIAL_VALUE_LOW;
    pinconf.task_pin = false;

    nrf_drv_gpiote_out_init(LED_PIN, &pinconf);
    
    #ifdef AMP_POWER_PIN
    nrf_drv_gpiote_out_init(AMP_POWER_PIN, &pinconf);
    nrf_drv_gpiote_out_clear(AMP_POWER_PIN);
    #endif
    
    #ifdef SG_PM_PIN
    nrf_drv_gpiote_out_init(SG_PM_PIN, &pinconf);
    nrf_drv_gpiote_out_clear(SG_PM_PIN);
    #endif
    
    #ifdef SLPSW_PIN
    nrf_gpio_cfg_sense_input(SG_PM_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIOTE_POLARITY_TOGGLE);
    #endif
}

int main()
{   
    APP_TIMER_INIT(0, APP_TIMER_OP_QUEUE_SIZE, NULL);
    
    initGpio();
    
    debugBlink();

    #ifdef SERIAL_ENABLE
    Serial serial;
    serial.write("\n -------------- Telemetrysystem Starting -------- \n");
    serial.write("0x6b65727374696e0a\n");
    #endif
    
    debugBlink();
    
    //BTLE
    #ifdef SERIAL_ENABLED
    serial.write("btleSeral init \n");
    #endif
    Btle btle;
    btle.start();
    
    debugBlink();
    
    //ADC
    #ifdef SERIAL_ENABLED
    serial.write("adc init \n");
    #endif
    adcInit();
    
    debugBlink();
    
    //i2c
    twiCshmInit(DEFAULT_SCL_PIN, DEFAULT_SDA_PIN);
    
    Mpu9150 mpu;
    
    Mcp4725 dac;
    
    debugBlink();
    
    //Cal
    Cal cal;
    //cal.load();
    
    debugBlink();
    
    Sampler sampler(&btle, &cal, &mpu);
    
    debugBlink();
    
    //timers
    APP_TIMER_DEF(sampleTimerAdc); 
    APP_TIMER_DEF(sampleTimerAux); 
    APP_TIMER_DEF(sleepTimer);
        
    debugBlink();
        
    //dispatch
    Dispatch dispatch(&dac, &mpu, &sampler, &cal, sampleTimerAdc, sampleTimerAux, sleepTimer);

    
    //connect callbacks
    btle.setRxCallback([&dispatch](uint8_t* buffer, uint32_t length){dispatch.rxDispatch(buffer, length);});
    btle.setDisconnectCallback([&dispatch](){dispatch.onBlteDisconnect();});
    btle.setConnectCallback([&dispatch](){dispatch.onBlteConnect();});
    

    nrf_drv_gpiote_out_set(LED_PIN);
    
    while(true) __WFE();
    
    return 0;
}
