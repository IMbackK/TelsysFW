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


#include "MCP4725.h"
#include "nrf_delay.h"

Mcp4725::Mcp4725(const uint8_t  address): I2cDevice(address), _value(0)
{
    setValue(0);
}

void Mcp4725::setValue(const uint16_t value)
{
    uint16_t packet = ((value & 0x00FF) << 8) + ((value & 0b0000111100000000) >> 8); //change endieness and mask first 4 bits.
    write( (uint8_t*)&packet, 2, true );
    _value = value;
}

void Mcp4725::setStartupValue(const uint16_t value, bool powerOff)
{
    uint8_t packet[3];
    packet[0] = 0b01100000;
    packet[1] = (value & 0b0000111111110000) >> 4;
    packet[2] = (value & 0b0000000000001111) << 4;
    write( (uint8_t*)&packet, 3, true );
    nrf_delay_ms(500);
    setValue(_value);
}


void Mcp4725::off()
{
    //uint16_t packet = 0b0000000000110000;
    //write( (uint8_t*)&packet, 2, true );
}

void Mcp4725::on()
{
    setValue(_value);
    nrf_delay_ms(200);
    setValue(_value);
}
