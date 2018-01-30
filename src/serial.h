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

#ifndef SERIAL_H
#define SERIAL_H

#include <string.h>
#include <stdlib.h>
#include "app_uart.h"
#include "nrf.h"
#include "board.h"

#define BAUD UART_BAUDRATE_BAUDRATE_Baud115200
#define UART_BUFFER_SIZE 128

class Serial
{
private:
    char _terminator = '\n';
    
    IRQn_Type _irq;
    
    const uint32_t _rxPin;
    const uint32_t _txPin;
    NRF_UART_Type *_uart_driver;
    
public:
    Serial(const uint32_t rxPin = DEFAULT_RX_PIN, const uint32_t txPin = DEFAULT_TX_PIN, NRF_UART_Type *uart_driver = NRF_UART0);
    void putChar(const char c);
    void write(const char* in, const unsigned int length);
    void write(const char in[]);
    void write(int32_t in);
    bool dataIsWaiting();
    char getChar();
    unsigned int getString(char* buffer, const int bufferLength);
    void flush();    
    void setTerminator(const char terminator);
};

#endif

