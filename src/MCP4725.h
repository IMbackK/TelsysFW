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

#include "i2c.h"

class Mcp4725 : private I2cDevice
{
private:
    uint16_t _value;
    
public:
    Mcp4725(const uint8_t  address = DEFAULT_MCP4725_ADRESS);
    void setValue(const uint16_t value);
    void setStartupValue(const uint16_t value, const bool poweroff = false);
    void off();
    void on();
};
