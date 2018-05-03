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

#define __STDC_CONSTANT_MACROS
#include <stdint.h>
#include <array>

#include "MCP4725.h"

class Cal
{
private:
    
    std::array<float, 5> _ampCalValues  = {1,1,1,1,1}; //values specifyed at 0% 25% 50% 75% and 100% range of uint16_t
    std::array<float, 5> _tempCalValues = {1,1,1,1,1}; //values specifyed at 0C 20C 40C 60C and 80C
    
    inline float apply(std::array<float, 5> values, const float in);
    inline float interpolate(std::array<float, 5> values, const float xValue);
    
public:
    
    void save();
    void load();
    
    void setAmpValues(uint8_t* values);
    void setTempValues(uint8_t* values);
    
    uint16_t applyAmp(const uint16_t in);
    uint16_t applyTemp(const uint16_t in, const uint8_t temp);
};

void offsetCallibration(Mcp4725* dac);

