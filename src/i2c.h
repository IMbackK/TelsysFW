#pragma once

#include <stdint.h>


#define DEFAULT_SCL_PIN      26
#define DEFAULT_SDA_PIN      25
#define BUFFER_SIZE 128

class I2cDevice
{
protected:
    
    const uint32_t _sclPin;
    const uint32_t _sdaPin;
    
    uint8_t _devAdress; 
    
    uint32_t _rxIndex=0;
    
public:
    I2cDevice(const uint8_t _devAdress, const uint32_t sclPin = DEFAULT_SCL_PIN, const uint32_t sdaPin = DEFAULT_SDA_PIN);
    ~I2cDevice();
    
    void putChar(const uint8_t ch, const bool stop = false);
    void write(const uint8_t* in, const unsigned int length, const bool stop = true);
    void write(const uint8_t in[], const bool stop = true);
    
    bool dataIsWaiting();
    void read(uint8_t* buffer, const uint32_t length);
    void txRxSequence( uint8_t* txBuffer, const unsigned int txLength, uint8_t* rxBuffer, const uint32_t rxLength );
    uint8_t txRxSequence( const uint8_t tx);
    
};
