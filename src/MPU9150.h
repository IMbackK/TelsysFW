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
#include "point3D.h"

#define DEFAULT_MPU9150_I2C_ADDRESS 0x68
#define DEFAULT_COMPASS_SUBDEVICE_I2C_ADDRESS 0x0C

class Mpu9150 : private I2cDevice
{
private:
    void writeRegsiter( uint8_t address, uint8_t data );
    const uint8_t _compassAddress;
public:
    
    Mpu9150(const uint8_t  address = DEFAULT_MPU9150_I2C_ADDRESS, const uint8_t compassAddress = DEFAULT_COMPASS_SUBDEVICE_I2C_ADDRESS);
    
    Point3D <int16_t> getAccelData();
    Point3D <int16_t> getMagnData();
    
    int16_t getTemperature();
    
    void stop();
    void start();
    
};
