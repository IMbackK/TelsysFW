#include "i2c.h"
#include "string.h"

I2cDevice::I2cDevice(const uint32_t sclPin, const uint32_t sdaPin):  _sclPin(sclPin), _sdaPin(sdaPin)
{
    twiCshmInit(_sclPin, _sdaPin);
}

I2cDevice::~I2cDevice()
{
    twiCshmDisable();
}

void I2cDevice::putChar(const uint8_t  address, const unsigned char ch, bool stop)
{
    while( !twiCshm_drv_twi_tx( address, &ch, 1, stop) );
}

void I2cDevice::write(const uint8_t  address, const char* in, const unsigned int length, bool stop)
{
    while( !twiCshm_drv_twi_tx(address, (uint8_t*)in, length, stop)  );
}

void I2cDevice::write(const uint8_t  address,const char in[], bool stop)
{
    while( !twiCshm_drv_twi_tx( address, (uint8_t*)in, strlen(in), stop) );
}

void I2cDevice::read(const uint8_t  address, uint8_t* buffer, uint32_t length)
{
    twiCshm_drv_twi_rx(address, buffer, length);
}
