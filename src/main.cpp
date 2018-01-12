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

#define APP_TIMER_PRESCALER             0    //Value of the RTC1 PRESCALER register.
#define APP_TIMER_OP_QUEUE_SIZE         5    //Size of timer operation queues.

extern "C"
{
    #define __STATIC_INLINE static inline
    
    //Standard libraries
    #include <stdint.h>
    #include <stddef.h>
    #include <string.h>

    //SDK headers
    #include "nrf.h"
    #include "nrf_gpiote.h"
    #include "nrf_gpio.h"
    #include "boards.h"
    //#include "nrf_drv_ppi.h"
    #include "nrf_delay.h"
    #include "nrf_drv_gpiote.h"
    #include "app_error.h"
   
}

#include "serial.h"
#include "adc.h"
#include "point3D.h"
#include "twiCshm.h"
#include "MPU9150.h"
#include "btleserial.h"

extern "C" void noophandler(void* param)
{
}

inline uint16_t ticksToUs(uint32_t ticks)
{
    return ticks*((APP_TIMER_PRESCALER+1)*100000) / APP_TIMER_CLOCK_FREQ;
}

int main()
{
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    

    
    if(!nrf_drv_gpiote_is_init())nrf_drv_gpiote_init();
    nrf_drv_gpiote_out_config_t pinconf;
    pinconf.init_state = NRF_GPIOTE_INITIAL_VALUE_LOW;
    pinconf.task_pin = false;
    
    nrf_drv_gpiote_out_init(17, &pinconf);
    nrf_drv_gpiote_out_init(7, &pinconf);
    
     nrf_drv_gpiote_out_toggle(17);
     nrf_delay_ms(500);
     nrf_drv_gpiote_out_toggle(17);
     nrf_delay_ms(500);

    Serial serial;
    serial.write("\n -------------- Telemetrysystem Starting -------- \n");
    
    serial.write("btleSeral init \n");
    BtleSerial btle(&serial);
    serial.write("btleSeral start \n");
    btle.start();
    
     nrf_drv_gpiote_out_toggle(17);
     nrf_delay_ms(500);
     nrf_drv_gpiote_out_toggle(17);
     nrf_delay_ms(500);
    
    serial.write("adc init \n");
    bool adcSucsess = adcInit();
    
    serial.write("result: ");
    adcSucsess ? serial.write("true") : serial.write("false");
    serial.putChar('\n');
    
     nrf_drv_gpiote_out_toggle(17);
     nrf_delay_ms(500);
     nrf_drv_gpiote_out_toggle(17);
     nrf_delay_ms(500);
    
    serial.write("mpu init \n");
    Mpu9150 mpu;
    mpu.start();
    
    serial.write("going to loop \n");
    
    serial.putChar('\n');
    
    //needet to start RTC1
    APP_TIMER_DEF(dummyTimer); 
    app_timer_create(&dummyTimer, APP_TIMER_MODE_REPEATED, noophandler);
    serial.write("timer start return: ");
    serial.write(app_timer_start(dummyTimer, 1000, NULL));
    serial.putChar('\n');
    
    uint32_t previousTicks = app_timer_cnt_get();
    
    while (1)
    {
        uint32_t current = app_timer_cnt_get();
        uint32_t delta;
        app_timer_cnt_diff_compute(current, previousTicks, &delta);
        previousTicks = current;
        delta = ticksToUs(delta);
        
        volatile uint16_t adcValue = getAdcValue();
        volatile Point3D <int16_t> accel = mpu.getAccelData();
        volatile Point3D <int16_t> magn = mpu.getMagnData();
       
        volatile int16_t temperature = mpu.getTemperature();
        
        serial.write("Sample Time Delta: ");
        serial.write(delta);
        serial.write("us\n");
        
        serial.write("ADC value: ");
        
        serial.write(adcValue);
        serial.putChar('\n');
        
        serial.write("Accel vector: x = ");
        serial.write(accel.x);
        serial.write(" y = ");
        serial.write(accel.y);
        serial.write(" z = ");
        serial.write(accel.z);
        serial.putChar('\n');

        serial.write("Magnetic vector: x = ");
        serial.write(magn.x);
        serial.write(" y = ");
        serial.write(magn.y);
        serial.write(" z = ");
        serial.write(magn.z);
        serial.putChar('\n');
        
        
        serial.write("temperature: ");
        serial.write(temperature);
        serial.write("c\n");
        
        if(btle.isConnected()) 
        {
            
            btle.write("Sample Time Delta: ");
            btle.write(delta);
        
            btle.write("ADC value: ");
            btle.write(adcValue);
            btle.putChar('\n');
        
            btle.write("Accel vector: x = ");
            btle.write(accel.x);
            btle.write(" y = ");
            btle.write(accel.y);
            btle.write(" z = ");
            btle.write(accel.z);
            btle.putChar('\n');
            
            btle.write("Magnetic vector: x = ");
            btle.write(magn.x);
            btle.write(" y = ");
            btle.write(magn.y);
            btle.write(" z = ");
            btle.write(magn.z);
            btle.putChar('\n');
            
            
            btle.write("temperature: ");
            btle.write(temperature);
            btle.write("c\n");
            
        }
        else serial.write("not connected.\n");
        serial.putChar('\n');
        
        nrf_drv_gpiote_out_toggle(17);
        nrf_drv_gpiote_out_toggle(7);
        //nrf_delay_ms(1);
    }
    
    return 0;
}
