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
#include "btleserial.h"
#include "cal.h"

#define ADC_PAKET_SIGNATURE 0x4587AD75

extern "C" void noophandler(void* param)
{
}

inline uint16_t ticksToUs(uint32_t ticks)
{
    return ticks*((APP_TIMER_PRESCALER+1)*1000000) / APP_TIMER_CLOCK_FREQ;
}

inline uint32_t usToTicks(uint16_t us)
{
    return us * APP_TIMER_CLOCK_FREQ /((APP_TIMER_PRESCALER+1)*1000000);
}

namespace adcSampler
{
    BtleSerial* btle; // under the rug you go
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
    nrf_delay_ms(50);
    nrf_drv_gpiote_out_toggle(LED_PIN);
    while(getAdcValue() > UINT16_MAX/2  && (value & 0xF000) == 0 )
    {
        dac->setValue(value+=8);
        nrf_delay_ms(1);
    }
    nrf_drv_gpiote_out_toggle(LED_PIN);
    nrf_delay_ms(100);
    nrf_drv_gpiote_out_toggle(LED_PIN);
    while(getAdcValue() < UINT16_MAX/2 && value > 1)
    {
        dac->setValue(value--);
    }
    nrf_drv_gpiote_out_toggle(LED_PIN);
}

int main()
{
    APP_TIMER_INIT(0, APP_TIMER_OP_QUEUE_SIZE, NULL);
    
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

    #ifdef SERIAL_ENABLE
    Serial serial;
    serial.write("\n -------------- Telemetrysystem Starting -------- \n");
    #endif
    
    nrf_drv_gpiote_out_toggle(LED_PIN);
    nrf_delay_ms(500);
    nrf_drv_gpiote_out_toggle(LED_PIN);
    nrf_delay_ms(500);
    
    
    //BTLE
    #ifdef SERIAL_ENABLED
    serial.write("btleSeral init \n");
    #endif
    BtleSerial btle;
    btle.start();
    
    
    nrf_drv_gpiote_out_toggle(LED_PIN);
    nrf_delay_ms(500);
    nrf_drv_gpiote_out_toggle(LED_PIN);
    nrf_delay_ms(500);
    
    //ADC
    #ifdef SERIAL_ENABLED
    serial.write("adc init \n");
    #endif
    bool adcSucsess = adcInit();
    /*adcSampler::btle = &btle;
    serial.write("adc async init return: ");
    serial.write(adcCreateAsyncTask(&adcSampler::adcSampleHandler));
    serial.putChar('\n');*/
    
    //i2c
    twiCshmInit(DEFAULT_SCL_PIN, DEFAULT_SDA_PIN);
    
    Mpu9150 mpu;
    mpu.start();
    
    Mcp4725 dac;
    nrf_delay_ms(500);
    
    //Cal
    offsetCallibration(&dac);
    Cal cal;
    //cal.load();
    
    //needet to start RTC1
    APP_TIMER_DEF(dummyTimer); 
    app_timer_create(&dummyTimer, APP_TIMER_MODE_REPEATED, noophandler);
    app_timer_start(dummyTimer, 1000, NULL);
    uint32_t previousTicks = app_timer_cnt_get();
    uint32_t ticksSincePacket = previousTicks;
    uint32_t targetDelta = 10000;
    
    bool sample = false;
    
#define SAMPLE_BUFFER_SIZE 5
    
    uint16_t sampleBuffer[SAMPLE_BUFFER_SIZE];
    uint8_t currentSampleBufferIndex = 0;
    
    int32_t smoothSample = 0;
    uint16_t prevSample = 0;
    
    while (1)
    {
        uint32_t current = app_timer_cnt_get();
        uint32_t delta;
        app_timer_cnt_diff_compute(current, previousTicks, &delta);
        previousTicks = current;
        ticksSincePacket += delta;
        

        /* volatile Point3D <int16_t> accel = mpu.getAccelData();
        volatile Point3D <int16_t> magn = mpu.getMagnData();
        volatile int16_t temperature = mpu.getTemperature();
        volatile uint16_t strainValue = cal.applyTemp(cal.applyAmp(getAdcValue()), temperature);*/
        volatile uint16_t strainValue = cal.applyAmp(getAdcValue());
        //volatile uint16_t strainValue = getAdcValue();
        
        smoothSample = smoothSample + (strainValue - smoothSample)/8;
        if(prevSample - 100  )
        
        if(smoothSample < 0) smoothSample = 0;
        else if(smoothSample > UINT16_MAX) smoothSample = UINT16_MAX;

        sampleBuffer[currentSampleBufferIndex] = (uint16_t)smoothSample;
        currentSampleBufferIndex++;
        
        uint8_t rxBuffer[256];
        int length = btle.read(rxBuffer, 256);
        if( length > 0 )
        {
            if( length > 2 && rxBuffer[0] == 'o' && rxBuffer[1] == 'n') 
            {
                sample = true;
                currentSampleBufferIndex = 0;
                ticksSincePacket = 0;
            }
            else if( length > 3 && rxBuffer[0] == 'o' && rxBuffer[1] == 'f' && rxBuffer[2] == 'f' )sample = false;
            else if( length > 3 && rxBuffer[0] == 'r' && rxBuffer[1] == 's' && rxBuffer[2] == 't' )sd_nvic_SystemReset();
            else if( sample == false)
            {
                if(length > 12 && rxBuffer[0] == 'c' && rxBuffer[1] == 'a' && rxBuffer[2] == 'l')
                {
                    cal.setAmpValues(rxBuffer+3);
                    cal.setTempValues(rxBuffer+8);
                    cal.save();
                }
                if(length > 6 && strncmp((char*)rxBuffer, "offset",  6))
                {
                    offsetCallibration(&dac);
                    sample = true;
                    currentSampleBufferIndex = 0;
                    ticksSincePacket = 0;
                }
            }
        }
        
        if(currentSampleBufferIndex == SAMPLE_BUFFER_SIZE)
        {
            currentSampleBufferIndex = 0;
            if(btle.isConnected() && sample) 
            {
                uint16_t usPacketDelta = ticksToUs(ticksSincePacket);
                uint8_t packet[SAMPLE_BUFFER_SIZE*2+9];
                packet[0] =   ADC_PAKET_SIGNATURE >> 24;
                packet[1] = ( ADC_PAKET_SIGNATURE & 0x00FF0000) >> 16;
                packet[2] = ( ADC_PAKET_SIGNATURE & 0x0000FF00) >> 8;
                packet[3] = ( ADC_PAKET_SIGNATURE & 0x000000FF);
                packet[4] = SAMPLE_BUFFER_SIZE;
                packet[5] = usPacketDelta >> 8; 
                packet[6] = usPacketDelta & 0x00FF;
                for(uint8_t i = 0; i < SAMPLE_BUFFER_SIZE; i++)
                {
                    packet[i*2+7] = sampleBuffer[i] >> 8; 
                    packet[i*2+8] = sampleBuffer[i] & 0x00FF;
                }
                packet[SAMPLE_BUFFER_SIZE*2+7] = 0xFF;
                packet[SAMPLE_BUFFER_SIZE*2+8] = 0xFF;
                btle.write(packet,SAMPLE_BUFFER_SIZE*2+9);
                ticksSincePacket = 0;
            }
        }
        
        nrf_drv_gpiote_out_toggle(LED_PIN);
        nrf_drv_gpiote_out_toggle(SAMPLE_CLOCK_PIN);
        
       nrf_delay_us(1000);
    }
    
    return 0;
}
