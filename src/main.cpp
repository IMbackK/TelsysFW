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


#include "board.h"
#include "serial.h"
#include "adc.h"
#include "point3D.h"
#include "twiCshm.h"
#include "MPU9150.h"
#include "MCP4725.h"
#include "btleserial.h"

#define ADC_PAKET_SIGNATURE 0x4587AD75

extern "C" void noophandler(void* param)
{
}

inline uint16_t ticksToUs(uint32_t ticks)
{
    return ticks*((APP_TIMER_PRESCALER+1)*1000000) / APP_TIMER_CLOCK_FREQ;
}

namespace adcSampler
{
    BtleSerial* btle;
    void adcSampleHandler(int16_t* buffer, uint16_t length)
    {
        uint16_t delta = 10;
        if(btle->isConnected()) 
        {
            uint8_t packet[1024];
            packet[0] =   ADC_PAKET_SIGNATURE >> 24;
            packet[1] = ( ADC_PAKET_SIGNATURE & 0x00FF0000) >> 16;
            packet[2] = ( ADC_PAKET_SIGNATURE & 0x0000FF00) >> 8;
            packet[3] = ( ADC_PAKET_SIGNATURE & 0x000000FF);
            
            for(uint16_t i = 0; i < length; i++)
            {
                    packet[i+4] = buffer[i] >> 8; 
                    packet[i+5] = buffer[i] & 0x00FF;
                    packet[i+6] = delta >> 8; 
                    packet[i+7] = delta & 0x00FF;
                   
            }
            packet[length+7] = 0xFF;
            packet[length+8] = 0xFF;
            btle->write(packet,length+9);
        }
    }
}

void offsetCallibration(Mcp4725* dac)
{
    uint16_t value = 0;
    dac->setValue(0);
    while(getAdcValue() > 120 && (value & 0xF000) == 0 )
    {
        dac->setValue(value+=8);
    }
    while(getAdcValue() < 125 && value > 1)
    {
        dac->setValue(value--);
    }
}

int main()
{
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    
    if(!nrf_drv_gpiote_is_init())nrf_drv_gpiote_init();
    nrf_drv_gpiote_out_config_t pinconf;
    pinconf.init_state = NRF_GPIOTE_INITIAL_VALUE_LOW;
    pinconf.task_pin = false;
    
    nrf_drv_gpiote_out_init(LED_PIN, &pinconf);
    nrf_drv_gpiote_out_init(7, &pinconf);
    
     nrf_drv_gpiote_out_toggle(LED_PIN);
     nrf_delay_ms(500);
     nrf_drv_gpiote_out_toggle(LED_PIN);
     nrf_delay_ms(500);

    Serial serial;
    serial.write("\n -------------- Telemetrysystem Starting -------- \n");
    
    nrf_drv_gpiote_out_toggle(LED_PIN);
    nrf_delay_ms(500);
    nrf_drv_gpiote_out_toggle(LED_PIN);
    nrf_delay_ms(500);
    
    
    //BTLE
    serial.write("btleSeral init \n");
    BtleSerial btle(&serial);
    serial.write("btleSeral start \n");
    btle.start();
    
    
    nrf_drv_gpiote_out_toggle(LED_PIN);
    nrf_delay_ms(500);
    nrf_drv_gpiote_out_toggle(LED_PIN);
    nrf_delay_ms(500);
    
    //ADC
    serial.write("adc init \n");
    bool adcSucsess = adcInit();
    /*adcSampler::btle = &btle;
    serial.write("adc async init return: ");
    serial.write(adcCreateAsyncTask(&adcSampler::adcSampleHandler));
    serial.putChar('\n');*/
    
    serial.write("result: ");
    adcSucsess ? serial.write("true") : serial.write("false");
    serial.putChar('\n');
    
    //i2c
    twiCshmInit(DEFAULT_SCL_PIN, DEFAULT_SDA_PIN);
    
    serial.write("mpu init \n");
    Mpu9150 mpu;
    mpu.start();
    
    serial.write("dac init \n");
    
    Mcp4725 dac;
    offsetCallibration(&dac);
    
    serial.write("going to loop\n\n");
    
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
       /* volatile Point3D <int16_t> accel = mpu.getAccelData();
        volatile Point3D <int16_t> magn = mpu.getMagnData();
       
        volatile int16_t temperature = mpu.getTemperature();*/
        
        serial.write("Sample Time Delta: ");
        serial.write(delta);
        serial.write("us\n");
        
        /*serial.write("ADC value: ");
        
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
        serial.write("c\n");*/
        
        if(btle.isConnected()) 
        {
            uint8_t packet[10];
            packet[0] = 0x4587AD75 >> 24;
            packet[1] = ( 0x4587AD75 & 0x00FF0000) >> 16;
            packet[2] = ( 0x4587AD75 & 0x0000FF00) >> 8;
            packet[3] = ( 0x4587AD75 & 0x000000FF);
            packet[4] = adcValue >> 8; 
            packet[5] = adcValue & 0x00FF;
            packet[6] = delta >> 8; 
            packet[7] = delta & 0x00FF;
            packet[8] = 0xFF;
            packet[9] = 0xFF;
            btle.write(packet,10);
        }
        //else serial.write("not connected.\n");
        //serial.putChar('\n');
        
        nrf_drv_gpiote_out_toggle(LED_PIN);
        nrf_drv_gpiote_out_toggle(SAMPLE_CLOCK_PIN);
        //nrf_delay_ms(1000);
    }
    
    return 0;
}
