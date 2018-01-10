#include "i2c.h"
#include "string.h"
#include "twiCshm.h"
#include "app_error.h"

I2cDevice::I2cDevice(const uint8_t  devAdress, const uint32_t sclPin, const uint32_t sdaPin): _sclPin(sclPin), _sdaPin(sdaPin), _devAdress(devAdress)
{
    twiCshmInit(_sclPin, _sdaPin);
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
