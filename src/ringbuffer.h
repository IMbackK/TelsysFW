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
#pragma once

#include <stdint.h>

template < int BUFFER_SIZE >
class RingBuffer
{
private:
    
    uint_fast16_t _headIndex = 0;
    uint_fast16_t _tailIndex = 0;
    uint8_t _buffer[BUFFER_SIZE];
    
public:
    
    RingBuffer(){}
    
    uint_fast16_t remaining()
    {
        return (_headIndex-_tailIndex);
    }
    
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
    
    unsigned int read( uint8_t* buffer, unsigned int length )
    {
        unsigned int i = 0;
        for(; i < length && !isEmpty(); i++)
        {
            buffer[i] = read();
        }
        return i;
    }
    
    void write( uint8_t in )
    {
        if (_headIndex - BUFFER_SIZE > 0 && _tailIndex - BUFFER_SIZE > 0)
        {
            _headIndex -= BUFFER_SIZE;
            _tailIndex -= BUFFER_SIZE;
        }
        _buffer[_headIndex % BUFFER_SIZE] = in;
        _headIndex++;
    }
    
    void write( uint8_t* buffer, unsigned int length )
    {
        for(uint8_t i = 0; i < length; i++) write(buffer[i]);
    }
    
    void flush()
    {
        _headIndex = 0;
        _tailIndex = 0;
        for(int i = 0; i < BUFFER_SIZE; i++) _buffer[i] = ' ';
    }
    
    unsigned int getString(uint8_t terminator, char* buffer, const unsigned int bufferLength)
    {
        unsigned int i = 0;
        for(; i <= remaining() && i <= BUFFER_SIZE && _buffer[(_tailIndex+i) % BUFFER_SIZE] != terminator; i++);
        
        if( i < remaining() && i > 0)
        {
            if(i > bufferLength-1) i = bufferLength-1;
            read((uint8_t*)buffer, i);
            buffer[i+1]='\0';
            _tailIndex++;
        }
        else i = 0;

        if (_buffer[(_tailIndex+i) % BUFFER_SIZE] == terminator || _buffer[(_tailIndex+i) % BUFFER_SIZE] == '\0'  ) _tailIndex++;
        
        return i;
    }
};
