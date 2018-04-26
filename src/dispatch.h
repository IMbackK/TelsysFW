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

extern "C" 
{

}

#include "MPU9150.h"
#include "MCP4725.h"
#include "cal.h"
#include "sampler.h"

class Dispatch
{
private:
    
    Mcp4725* _dac;
    Mpu9150* _mpu;
    Sampler* _sampler;
    Cal* _cal;

    //timers
    app_timer_t* _sleepTimer;
    app_timer_t* _sampleTimerAdc;
    app_timer_t* _sampleTimerAux;
    
    uint32_t desierdTicks = 164*2; //Timmer runs at 32768Hz. this value corrsponds to 1/100 of a second.
    
    static void deepSleep( void* instance);
    void stop();
    void start();
    
public: 
    
    Dispatch(Mcp4725* dac, Mpu9150* mpu, Sampler* sampler, Cal* cal, app_timer_t* sampleTimerAdc, app_timer_t* sampleTimerAux, app_timer_t* sleepTimer);
    void rxDispatch(uint8_t* buffer, uint32_t length);
    void onBlteDisconnect();
    void onBlteConnect();
};
