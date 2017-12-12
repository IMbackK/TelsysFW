#ifndef SERIAL_H
#define SERIAL_H

#define BAUD UART_BAUDRATE_BAUDRATE_Baud115200
#define BUFFER_SIZE 128
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 256
#define RX_PIN 8
#define TX_PIN 6


#include <string.h>
#include <stdlib.h>
#include "app_uart.h"
#include "nrf.h"



class Serial
{
private:
    char _terminator = '\n';
    uint16_t _rxIndex=0;
    
    IRQn_Type irq;
    
public:
    Serial();
    void putChar(const char c);
    void write(const char* in, const unsigned int length);
    void write(const char in[]);
    void write(const int32_t in);
    bool dataIsWaiting();
    char getChar();
    unsigned int getString(char* buffer, const int bufferLength);
    void flush();    
    void setTerminator(const char terminator);
};

#endif

