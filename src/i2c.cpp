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

#include "i2c.h"
#include "string.h"
#include "twiCshm.h"
#include "app_error.h"

I2cDevice::I2cDevice(const uint8_t  devAdress): _devAdress(devAdress)
{
  
}

I2cDevice::~I2cDevice()
{
    twiCshmDisable();
}

void I2cDevice::putChar( const uint8_t ch, bool stop)
{
    twiCshm_drv_twi_tx( _devAdress, &ch, 1, stop);
}

void I2cDevice::write(const uint8_t* in, const unsigned int length, bool stop)
{
    twiCshm_drv_twi_tx(_devAdress, in, length, stop);
}

void I2cDevice::write( const uint8_t in[], bool stop)
{
    twiCshm_drv_twi_tx( _devAdress, in, strlen((const char*)in), stop);
}

void I2cDevice::read(uint8_t* buffer, uint32_t length)
{
    twiCshm_drv_twi_rx(_devAdress, buffer, length);
}

void I2cDevice::txRxSequence( uint8_t* txBuffer, const unsigned int txLength, uint8_t* rxBuffer, const uint32_t rxLength )
{
    ret_code_t ret = twiCshm_drv_twi_tx( _devAdress, txBuffer, txLength, false);
    if( ret == NRF_SUCCESS )twiCshm_drv_twi_rx(_devAdress, rxBuffer, rxLength);
}

uint8_t I2cDevice::txRxSequence(uint8_t tx)
{
    uint8_t rx;
    txRxSequence(&tx, 1, &rx, 1);
    return rx;
}
