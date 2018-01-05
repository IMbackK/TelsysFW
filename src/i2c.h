#pragma once

#include <stdint.h>

#include "twiCshm.h"

#define DEFAULT_SCL_PIN      27
#define DEFAULT_SDA_PIN      26
#define BUFFER_SIZE 128

class I2cDevice
{
private:
    
    const uint32_t _sclPin;
    const uint32_t _sdaPin;
    
    
    uint32_t _rxIndex=0;
    
public:
    I2cDevice(const uint32_t sclPin = DEFAULT_SCL_PIN, const uint32_t sdaPin = DEFAULT_SDA_PIN);
    ~I2cDevice();
    
    void putChar(const unsigned char ch, const bool stop = false);
    void write(const uint8_t  address, const char* in, const unsigned int length, const bool stop = false);
    void write(const uint8_t  address, const char in[], const bool stop = false);
    
    bool dataIsWaiting();
    void read(const uint8_t  address, uint8_t* buffer, const uint32_t length);
    
};
