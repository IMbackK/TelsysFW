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

#include "dispatch.h"

extern "C" 
{
    #include "softdevice_handler.h"
    #include "nrf_delay.h"
    #include "nrf_drv_gpiote.h"
}


static void debugBlink(int length = 200)
{
    nrf_drv_gpiote_out_toggle(LED_PIN);
    nrf_delay_ms(length);
    nrf_drv_gpiote_out_toggle(LED_PIN);
    nrf_delay_ms(length);
}

Dispatch::Dispatch(Mcp4725* dac, Mpu9150* mpu, Sampler* sampler, Cal* cal, app_timer_t* sampleTimerAdc, app_timer_t* sampleTimerAux, app_timer_t* sleepTimer)
{
    _dac = dac;
    _mpu = mpu;
    _sampler = sampler;
    _cal = cal;
    
    _sleepTimer = sleepTimer;
    _sampleTimerAdc = sampleTimerAdc;
    _sampleTimerAux = sampleTimerAux;
    
    app_timer_create(&_sampleTimerAdc, APP_TIMER_MODE_REPEATED, &Sampler::sampleStaticAdc);
    app_timer_create(&_sampleTimerAux, APP_TIMER_MODE_REPEATED, &Sampler::sampleStaticAux);
    app_timer_create(&_sleepTimer, APP_TIMER_MODE_SINGLE_SHOT, &Dispatch::deepSleep);
    
    _dac->on();
    offsetCallibration(_dac);
    stop();
    onBlteDisconnect();
}

void Dispatch::stop()
{
    app_timer_stop(_sampleTimerAdc);
    app_timer_stop(_sampleTimerAux);
    
    //_dac->off();
    _mpu->stop();
    
    #ifdef AMP_POWER_PIN
    nrf_drv_gpiote_out_clear(AMP_POWER_PIN);
    #endif
    
    #ifdef SG_PM_PIN
    nrf_drv_gpiote_out_clear(SG_PM_PIN);
    #endif
    
    stopped = true;
}

void Dispatch::start()
{
    #ifdef AMP_POWER_PIN
    nrf_drv_gpiote_out_set(AMP_POWER_PIN);
    #endif
    
    _dac->on();
    _mpu->start();
    
    #ifdef SG_PM_PIN
    nrf_drv_gpiote_out_set(SG_PM_PIN);
    #endif
    
    nrf_delay_ms(50);
    
    _sampler->resetTimes();
    app_timer_start(_sampleTimerAdc, desierdTicks, reinterpret_cast<void*>(_sampler));
    app_timer_start(_sampleTimerAux, 1638, reinterpret_cast<void*>(_sampler)); //20Hz
    nrf_delay_us(100);
    
    stopped = false;
}

void Dispatch::rxDispatch(uint8_t* buffer, uint32_t length)
{
    if( length > 0 )
    {
        if( length > 2 && buffer[0] == 'o' && buffer[1] == 'n' ) start();
        else if( length > 3 && buffer[0] == 'o' && buffer[1] == 'f' && buffer[2] == 'f') stop();
        else if(length > 2 && buffer[0] == 'o' && buffer[1] == 's' && buffer[2] == 't')
        {
            debugBlink(500);
            bool wasStopped = stopped;
            if(!stopped) stop();
            else _dac->on();
            offsetCallibration(_dac);
            if(!wasStopped) start();
            else _dac->off();
        }
        else if( length > 3 && buffer[0] == 'r' && buffer[1] == 's' && buffer[2] == 't' ) sd_nvic_SystemReset();
        else if(length > 12 && buffer[0] == 'c' && buffer[1] == 'a' && buffer[2] == 'l')
        {
            debugBlink(500);
            stop();
            _cal->setAmpValues(buffer+3);
            _cal->setTempValues(buffer+8);
            _cal->save();
        }
        
        else if(length > 4 && buffer[0] == 'r' && buffer[1] == 'a' && buffer[2] == 't')
        {
            debugBlink(500);
            stop();
            uint16_t newSampleRate = buffer[4];
            newSampleRate += buffer[3] << 8;
            if(newSampleRate < 20) newSampleRate = 20;
            else if(newSampleRate > 500) newSampleRate = 500;
            desierdTicks = usToTicks(1000000/newSampleRate);
        }
    }
}

void Dispatch::onBlteDisconnect()
{
    debugBlink();
    debugBlink();
    stop();
#ifdef SLEEP_ENABLED
    app_timer_start(_sleepTimer, 3840000, reinterpret_cast<void*>(this)); //~120 seconds until deep sleep;
#endif
}

void Dispatch::onBlteConnect()
{
    debugBlink();
    app_timer_stop(_sleepTimer); //cancle deep sleep
}

void Dispatch::deepSleep( void* instance )
{
    debugBlink(500);
    debugBlink(500);
    debugBlink(500);
    Dispatch* dispatchInstance = reinterpret_cast<Dispatch*>(instance);
    dispatchInstance->stop();
        
    #ifdef SLPSW_PIN
    nrf_drv_gpiote_out_clear(LED_PIN);
    sd_power_system_off(); 
    #endif
}

