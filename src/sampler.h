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

#pragma once

#include <array>
#include <stdint.h>

#include "btle.h"
#include "adc.h"
#include "cal.h"
#include "MPU9150.h"

extern "C" 
{
    #include "app_timer.h"
}

#define SAMPLE_BUFFER_SIZE 8
#define SAMPELING_SMOOTHING_FACTOR 1 //no smoothing 

inline uint32_t ticksToUs(uint32_t ticks)
{
    return ticks*((APP_TIMER_PRESCALER+1)*1000000) / APP_TIMER_CLOCK_FREQ;
}

inline uint32_t usToTicks(uint32_t us)
{
    return us * APP_TIMER_CLOCK_FREQ /((APP_TIMER_PRESCALER+1)*1000000);
}

class Sampler
{
private:
    
    uint_fast8_t sampleNumber = 0;
    
    int16_t lastTemperature = 20;
    
    uint16_t smoothsample = UINT16_MAX/2;
    
    uint32_t previousAdcPaketTicks = 0;
    uint32_t previousAuxPaketTicks = 0;
    
    std::array<uint16_t, SAMPLE_BUFFER_SIZE> _sampleBuffer;
    Btle* _btle;
    Mpu9150* _mpu;
    Cal*    _cal;
    
    void sendMpuData();
    
public:
    Sampler(Btle* btle, Cal* cal = nullptr, Mpu9150* mpu = nullptr);
    
    void sampleAdc();
    void sampleAux();
    static void sampleStaticAdc(void* voidInstance);
    static void sampleStaticAux(void* voidInstance);
    
    void resetTimes();
    
};
