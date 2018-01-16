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

#include <stdint.h>
#include "board.h"

#define BUFFER_SIZE 128

class I2cDevice
{
protected:
    
    uint8_t _devAdress; 
    
    uint32_t _rxIndex=0;
    
public:
    I2cDevice(const uint8_t devAdress);
    ~I2cDevice();
    
    void putChar(const uint8_t ch, const bool stop = false);
    void write(const uint8_t* in, const unsigned int length, const bool stop = true);
    void write(const uint8_t in[], const bool stop = true);
    
    bool dataIsWaiting();
    void read(uint8_t* buffer, const uint32_t length);
    void txRxSequence( uint8_t* txBuffer, const unsigned int txLength, uint8_t* rxBuffer, const uint32_t rxLength );
    uint8_t txRxSequence( const uint8_t tx);
    
};
