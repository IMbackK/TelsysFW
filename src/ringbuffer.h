//UVOS
#pragma once

#include <stdint>

template < int BUFFER_SIZE >
class RingBuffer
{
private:
    
    uint32_t _headIndex = 0;
    uint32_t _tailIndex = 0;
    uint8_t _buffer[BUFFER_SIZE];
    
public:
    
    RingBuffer(){}
    bool isEmpty();
    uint8_t read();
    void read( uint8_t* buffer, unsigned int length );
    void write( uint8_t in );
    void write( uint8_t* buffer, unsigned int length );
};
