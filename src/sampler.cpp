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

#include "sampler.h"

extern "C"
{
   #include "nrf_drv_gpiote.h"
}

Sampler::Sampler(Btle* btle, Cal* cal, Mpu9150* mpu): _btle(btle), _mpu(mpu), _cal(cal)
{
}

void Sampler::sampleStaticAdc(void* voidInstance)
{
    Sampler* instance = reinterpret_cast<Sampler*>(voidInstance);
    instance->sampleAdc();
}

void Sampler::sampleStaticAux(void* voidInstance)
{
    Sampler* instance = reinterpret_cast<Sampler*>(voidInstance);
    instance->sampleAux();
}

void Sampler::sampleAux()
{
    uint32_t delta;
    uint32_t current = app_timer_cnt_get();
    app_timer_cnt_diff_compute(current, previousAuxPaketTicks, &delta);
    previousAuxPaketTicks = current;
    if(delta > UINT16_MAX) delta = UINT16_MAX;
    
    lastTemperature = _mpu->getTemperature();
    
    _btle->sendAuxPacket(_mpu->getAccelData(), _mpu->getMagnData(), lastTemperature, delta);
}

void Sampler::sampleAdc()
{    
    
    #ifdef SG_PM_PIN
    nrf_drv_gpiote_out_set(SG_PM_PIN);
    #endif
    
    smoothsample = smoothsample + ((getAdcValue() - smoothsample)/SAMPELING_SMOOTHING_FACTOR);
    
    #ifdef SG_PM_PIN
    #ifdef SG_PM
    nrf_drv_gpiote_out_clear(SG_PM_PIN);
    #endif
    #endif
    
    _sampleBuffer[sampleNumber] = smoothsample;//_cal->applyTemp(_cal->applyAmp(smoothsample), lastTemperature);
    
    sampleNumber++;
    
    if(sampleNumber == _sampleBuffer.size())
    {
        sampleNumber = 0;
        
        uint32_t delta;
        uint32_t current = app_timer_cnt_get();
        app_timer_cnt_diff_compute(current, previousAdcPaketTicks, &delta);
        previousAdcPaketTicks = current;
        if(delta/_sampleBuffer.size() > UINT16_MAX) delta = UINT16_MAX;
        
        _btle->sendAdcSamples(_sampleBuffer.data(), _sampleBuffer.size(), delta/_sampleBuffer.size());
    }
}


void Sampler::resetTimes()
{
    previousAuxPaketTicks = app_timer_cnt_get();
    previousAdcPaketTicks = app_timer_cnt_get();
}
