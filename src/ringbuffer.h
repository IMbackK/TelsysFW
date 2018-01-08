//UVOS
#pragma once

#include <stdint.h>

template < int BUFFER_SIZE >
class RingBuffer
{
private:
    
    uint32_t _headIndex = 0;
    uint32_t _tailIndex = 0;
    uint8_t _buffer[BUFFER_SIZE];
    
public:
    
    RingBuffer(){}
    bool isEmpty()
    {
        return _tailIndex >= _headIndex;
    }
    uint8_t read()
    {
    if(!isEmpty())
    {
        _tailIndex++;
        return _buffer[(_tailIndex - 1) % BUFFER_SIZE];
    }
    else return '\0';
    }
    void read( uint8_t* buffer, unsigned int length )
    {
        for(uint8_t i = 0; i < length; i++)
        {
            buffer[i] = read();
        }
    }
    void write( uint8_t in )
    {
        if (_headIndex - BUFFER_SIZE > 0 && _tailIndex - BUFFER_SIZE > 0)
        {
            _headIndex -= BUFFER_SIZE;
            _tailIndex -= BUFFER_SIZE;
        }
        _buffer[_tailIndex] = in;
        _tailIndex++;
    }
    void write( uint8_t* buffer, unsigned int length )
    {
        for(uint8_t i = 0; i < length; i++) write(buffer[i]);
    }
};
