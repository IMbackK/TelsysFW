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

#include "serial.h"

#include "ringbuffer.h"

RingBuffer<UART_BUFFER_SIZE> rxBuffer;

void UARTE0_UART0_IRQHandler()
{
    if (NRF_UART0->EVENTS_RXDRDY)
        {
            rxBuffer.write(NRF_UART0->RXD);
            NRF_UART0->EVENTS_RXDRDY = 0x0UL;
        }
}


Serial::Serial(const uint32_t rxPin, const uint32_t txPin, NRF_UART_Type *uart_driver): _rxPin(rxPin), _txPin(txPin), _uart_driver(uart_driver)
{
    //setup uart device
    _uart_driver->PSELTXD = _txPin;
    _uart_driver->PSELRXD = _rxPin;
    _uart_driver->CONFIG = (UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos) | UART_CONFIG_HWFC_Disabled;
    _uart_driver->BAUDRATE = BAUD;
    _uart_driver->ENABLE = UART_ENABLE_ENABLE_Enabled;
    _uart_driver->EVENTS_RXDRDY = 0x0UL;
    _uart_driver->EVENTS_TXDRDY = 0x0UL;
    _uart_driver->TASKS_STARTRX = 0x1UL;
    _uart_driver->TASKS_STARTTX = 0x1UL;
    _uart_driver->INTENSET = UART_INTENSET_RXDRDY_Msk;
    
    //todo enable IRQ
    NVIC_ClearPendingIRQ(_irq);
    NVIC_SetPriority(_irq, 3);
    NVIC_EnableIRQ(_irq);
}

void Serial::putChar(const char c)
{
    _uart_driver->TXD = c;

    while(!_uart_driver->EVENTS_TXDRDY);

    _uart_driver->EVENTS_TXDRDY = 0x0UL;
}

void Serial::write(const char* in, const unsigned int length)
{
    for(unsigned int i = 0; i < length && in[i] != '\0'; i++)
    {
        putChar(in[i]);
    }
}

void Serial::write(const char in[])
{
    for(unsigned int i = 0; i < strlen(in); i++)
    {
        putChar(in[i]);
    }
}

void Serial::write(int32_t in)
{
    if(in == 0)
    {
        putChar('0');
    }
    else
    {
        bool sign = false;
        char str[64] = { 0 }; 
        int16_t i = 62;
        if (in < 0) 
        {
            sign = true;
            in = abs(in);
        }

        while (in != 0 && i > 0) 
        { 
            str[i--] = (in % 10) + '0';
            in /= 10;
        }

        if (sign) str[i--] = '-';
        write(str + i + 1, 64-(i+1));
    }
}


bool Serial::dataIsWaiting()
{
    return !rxBuffer.isEmpty();
}

char Serial::getChar()
{
    if(dataIsWaiting())
    {
        return rxBuffer.read();
    }
    else return '\0';
}

unsigned int Serial::getString(char* buffer, const int bufferLength)
{
    return rxBuffer.getString(_terminator, buffer, bufferLength);
}

void Serial::flush()
{
    rxBuffer.flush();
}

void Serial::setTerminator(char terminator){_terminator = terminator;}
