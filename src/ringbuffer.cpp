//UVOS
#include "ringbuffer.h"

bool RingBuffer::isEmpty()
{
    return _tailIndex >= _headIndex;
}

uint8_t RingBuffer::read()
{
    if(!isEmpty())
    {
        _tailIndex++;
        return _buffer[(_tailIndex - 1) % BUFFER_SIZE];
    }
    else return '\0';
}

void RingBuffer::read( uint8_t* buffer, unsigned int length )
{
    for(uint8_t i = 0; i < length; i++)
    {
        buffer[i] = read();
    }
}

void RingBuffer::write(uint8_t in)
{
    if (_headIndex - BUFFER_SIZE > 0 && _tailIndex - BUFFER_SIZE > 0)
    {
        _headIndex -= BUFFER_SIZE;
        _tailIndex -= BUFFER_SIZE;
    }
    buffer[_tailIndex] = in;
    _tailIndex++;
}

void RingBuffer::write( uint8_t* buffer, unsigned int length )
{
    for(uint8_t i = 0; i < length; i++) write(buffer[i]);
}
