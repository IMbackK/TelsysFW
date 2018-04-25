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

uint16_t meanAdcValue( uint16_t count )
{
    uint32_t accumulator = 0;
    for(unsigned i  = 0; i < count; i++) 
    {
        accumulator+=getAdcValue();
        nrf_delay_us(100);
    }
    return accumulator/count;
}

void offsetCallibration(Mcp4725* dac)
{
    uint16_t value = 0;
    dac->setValue(0);
    
    #ifdef SG_PM_PIN
    nrf_drv_gpiote_out_set(SG_PM_PIN);
    #endif
    
    nrf_delay_ms(500);
    
    while(meanAdcValue(5) > UINT16_MAX/2  && (value & 0xF000) == 0 )
    {
        dac->setValue(value+=8);
        nrf_delay_ms(1);
    }
    nrf_delay_ms(500);
    while(meanAdcValue(5) < UINT16_MAX/2 && value > 1)
    {
        dac->setValue(value--);
        nrf_delay_ms(1);
    }
    nrf_drv_gpiote_out_toggle(LED_PIN);
    
    #ifdef SG_PM_PIN
    nrf_drv_gpiote_out_clear(SG_PM_PIN);
    #endif
}

void debugBlink()
{
    nrf_drv_gpiote_out_toggle(LED_PIN);
    nrf_delay_ms(200);
    nrf_drv_gpiote_out_toggle(LED_PIN);
    nrf_delay_ms(200);
}

int main()
{   
    //APP_TIMER_INIT(0, APP_TIMER_OP_QUEUE_SIZE, NULL);
    
    if(!nrf_drv_gpiote_is_init())nrf_drv_gpiote_init();
    nrf_drv_gpiote_out_config_t pinconf;
    pinconf.init_state = NRF_GPIOTE_INITIAL_VALUE_LOW;
    pinconf.task_pin = false;

        nrf_drv_gpiote_out_init(LED_PIN, &pinconf);
    
    #ifdef AMP_POWER_PIN
    nrf_drv_gpiote_out_init(AMP_POWER_PIN, &pinconf);
    nrf_drv_gpiote_out_set(AMP_POWER_PIN);
    #endif
    
    #ifdef SG_PM_PIN
    nrf_drv_gpiote_out_init(SG_PM_PIN, &pinconf);
    nrf_drv_gpiote_out_clear(SG_PM_PIN);
    #endif
    
    debugBlink();

    #ifdef SERIAL_ENABLE
    Serial serial;
    serial.write("\n -------------- Telemetrysystem Starting -------- \n");
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
    mpu.start();
    
    Mcp4725 dac;
    
    debugBlink();
    
    //Cal
    offsetCallibration(&dac);
    Cal cal;
    //cal.load();
    
    debugBlink();
    
    Sampler sampler(&btle, &cal, &mpu);
    
    //Sample Timer
    APP_TIMER_DEF(sampleTimerAdc); 
    app_timer_create(&sampleTimerAdc, APP_TIMER_MODE_REPEATED, &Sampler::sampleStaticAdc);
    
    
    APP_TIMER_DEF(sampleTimerAux); 
    app_timer_create(&sampleTimerAux, APP_TIMER_MODE_REPEATED, &Sampler::sampleStaticAux);
    
    uint32_t desierdTicks = 164*2; //timmer runs at 32768Hz. this value corrsponds to 1/100 of a second.
    
    bool sampeling = false;
    debugBlink();
    
    uint count = 0;
    
    while(true)
    {
        uint8_t rxBuffer[64];
        int length = btle.read(rxBuffer, 64);
        if( length > 0 )
        {
            if( length > 2 && rxBuffer[0] == 'o' && rxBuffer[1] == 'n' ) 
            {
                sampler.resetTimes();
                app_timer_start(sampleTimerAdc, desierdTicks, reinterpret_cast<void*>(&sampler));
                app_timer_start(sampleTimerAux, 1638, reinterpret_cast<void*>(&sampler)); //20Hz
                sampeling = true;
                nrf_delay_us(100);
            }
            else if( length > 3 && rxBuffer[0] == 'o' && rxBuffer[1] == 'f' && rxBuffer[2] == 'f')
            {
                app_timer_stop(sampleTimerAdc);
                app_timer_stop(sampleTimerAux);
                sampeling = false;
                nrf_delay_us(100);
            }
            else if(length > 2 && rxBuffer[0] == 'o' && rxBuffer[1] == 's' && rxBuffer[2] == 't')
            {
                app_timer_stop(sampleTimerAux);
                offsetCallibration(&dac);
                app_timer_start(sampleTimerAux, 1092, reinterpret_cast<void*>(&sampler));
            }
            else if( length > 3 && rxBuffer[0] == 'r' && rxBuffer[1] == 's' && rxBuffer[2] == 't' ) sd_nvic_SystemReset();
            else if( !sampeling )
            {
                if(length > 12 && rxBuffer[0] == 'c' && rxBuffer[1] == 'a' && rxBuffer[2] == 'l')
                {
                    cal.setAmpValues(rxBuffer+3);
                    cal.setTempValues(rxBuffer+8);
                    cal.save();
                }
                
                else if(length > 4 && rxBuffer[0] == 'r' && rxBuffer[1] == 'a' && rxBuffer[2] == 't')
                {
                    uint16_t newSampleRate = rxBuffer[4];
                    newSampleRate += rxBuffer[3] << 8;
                    if(newSampleRate < 20) newSampleRate = 20;
                    else if(newSampleRate > 500) newSampleRate = 500;
                    desierdTicks = usToTicks(1000000/newSampleRate);
                    debugBlink();
                }
            }
        }
        
        nrf_drv_gpiote_out_toggle(LED_PIN);
        /*
        count+=2;
        dac.setValue(count);
        */
        nrf_delay_ms(100);
    }
    
    return 0;
}
