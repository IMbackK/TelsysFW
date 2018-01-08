#include "i2c.h"
#include "string.h"
#include "twiCshm.h"

I2cDevice::I2cDevice(const uint8_t  devAdress, const uint32_t sclPin, const uint32_t sdaPin): _sclPin(sclPin), _sdaPin(sdaPin), _devAdress(devAdress << 1)
{
    twiCshmInit(_sclPin, _sdaPin);
}

I2cDevice::~I2cDevice()
{
    twiCshmDisable();
}

void I2cDevice::putChar( const uint8_t ch, bool stop)
{
    while( !twiCshm_drv_twi_tx( _devAdress, &ch, 1, stop) );
}

void I2cDevice::write(const uint8_t* in, const unsigned int length, bool stop)
{
    while( !twiCshm_drv_twi_tx(_devAdress, in, length, stop)  );
}

void I2cDevice::write( const uint8_t in[], bool stop)
{
    while( !twiCshm_drv_twi_tx( _devAdress, in, strlen((const char*)in), stop) );
}

void I2cDevice::read(uint8_t* buffer, uint32_t length)
{
    twiCshm_drv_twi_rx(_devAdress, buffer, length);
}

void I2cDevice::txRxSequence( uint8_t* txBuffer, const unsigned int txLength, uint8_t* rxBuffer, const uint32_t rxLength )
{
    write(txBuffer, txLength);
    read(rxBuffer, rxLength);
}

uint8_t I2cDevice::txRxSequence(uint8_t tx)
{
    putChar(tx, true);
    uint8_t rx;
    read(&rx, 1);
    return rx;
}
