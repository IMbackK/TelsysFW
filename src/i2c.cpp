#include "i2c.h"
#include "string.h"

I2cDevice::I2cDevice(const uint8_t address, const uint32_t sclPin, const uint32_t sdaPin):  _sclPin(sclPin), _sdaPin(sdaPin), _address(address)
{
    twiCshmInit(_sclPin, _sdaPin);
}

I2cDevice::~I2cDevice()
{
    twiCshmDisable();
}

void I2cDevice::putChar(const unsigned char ch, bool stop)
{
    while( !twiCshm_drv_twi_tx( _address, &ch, 1, stop) );
}

void I2cDevice::write(const char* in, const unsigned int length, bool stop)
{
    while( !twiCshm_drv_twi_tx(_address, (uint8_t*)in, length, stop)  );
}

void I2cDevice::write(const char in[], bool stop)
{
    while( !twiCshm_drv_twi_tx( _address, (uint8_t*)in, strlen(in), stop) );
}

void I2cDevice::read(uint8_t* buffer, uint32_t length)
{
    twiCshm_drv_twi_rx(_address, buffer, length);
}
