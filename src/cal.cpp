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

#include "cal.h"
#include "storage.h"

extern "C" 
{
    #include "nrf_drv_gpiote.h"
    #include "nrf_delay.h"
}

#include "adc.h"

void Cal::save()
{
    uint32_t pages[10];
    
    for(unsigned int pageNumber = 0; pageNumber < 10; pageNumber++)
    {
            float* tmpfloat;
            if(pageNumber < 5)
            {
                *tmpfloat = _ampCalValues[pageNumber];
            }
            else if( pageNumber < 10 )
            {
                *tmpfloat = _tempCalValues[pageNumber-5];
            }
            pages[pageNumber] = *(reinterpret_cast<uint32_t*>(tmpfloat));
    }
    for(unsigned int i = 0; i < 10; i++) flash_store(i, pages[i], false);
}

void Cal::load()
{
    uint32_t pages[10];
    for(unsigned int i = 0; i < 10; i++) pages[i] = flash_read(i);
    
    for(unsigned int pageNumber = 0; pageNumber < 10; pageNumber++)
    {
        if(pageNumber < 5)          _ampCalValues[pageNumber]    = *(reinterpret_cast<float*>(&pages[pageNumber]));
        else if( pageNumber < 10 )  _tempCalValues[pageNumber-5] = *(reinterpret_cast<float*>(&pages[pageNumber]));
    }
}

void Cal::setAmpValues(uint8_t* values)
{
    int8_t* signValues = reinterpret_cast<int8_t*>(values);
    for(unsigned int i = 0; i < 5; i++) _ampCalValues[i] =  1.0+(float)signValues[i]*0.001;
}
void Cal::setTempValues(uint8_t* values)
{
    int8_t* signValues = reinterpret_cast<int8_t*>(values);
    for(unsigned int i = 0; i < 5; i++) _tempCalValues[i] = 1.0+(float)signValues[i]*0.001;
}

uint16_t Cal::applyAmp(uint16_t in)
{
    float floatIn = (float)in/((float)(UINT16_MAX));
    float floatOut = interpolate(_ampCalValues, floatIn)*floatIn;
    return (uint16_t)(floatOut*UINT16_MAX);
}

uint16_t Cal::applyTemp(uint16_t in, const uint8_t temp)
{
    if(temp > 80) return in;
    float floatIn = (float)in/((float)(UINT16_MAX));
    float floatOut = interpolate(_tempCalValues, temp/((float)80.0))*floatIn;
    return (uint16_t)floatOut*UINT16_MAX;
}

float Cal::interpolate(const std::array<float, 5> values, float xValue)
{
    uint8_t index = (uint8_t)(xValue*4);
    return (values[index+1] - values[index])*xValue + values[index];
}

void offsetCallibration(Mcp4725* dac)
{
    uint16_t value = 0;
    dac->setValue(0);
    
    #ifdef AMP_POWER_PIN
    nrf_drv_gpiote_out_set(AMP_POWER_PIN);
    #endif
    
    #ifdef SG_PM_PIN
    nrf_drv_gpiote_out_set(SG_PM_PIN);
    #endif
    
    nrf_delay_ms(500);
    
    while(meanAdcValue(2) > UINT16_MAX/2  && (value & 0xF000) == 0 )
    {
        dac->setValue(value+=8);
        nrf_delay_ms(1);
    }
    nrf_delay_ms(500);
    while(meanAdcValue(2) < UINT16_MAX/2 && value > 1)
    {
        dac->setValue(value--);
        nrf_delay_ms(1);
    }
    
    //dac->setStartupValue(value);
    
    /*#ifdef SG_PM_PIN
    #ifdef SG_PM
    nrf_drv_gpiote_out_clear(SG_PM_PIN);
    #endif
    #endif
    
    #ifdef AMP_POWER_PIN
    nrf_drv_gpiote_out_clear(AMP_POWER_PIN);
    #endif*/
    
    //dac->setStartupValue(value);
}
