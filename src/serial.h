#ifndef SERIAL_H
#define SERIAL_H

#define BAUD UART_BAUDRATE_BAUDRATE_Baud115200
#define BUFFER_SIZE 128
#define DEFAULT_RX_PIN 8
#define DEFAULT_TX_PIN 6


#include <string.h>
#include <stdlib.h>
#include "app_uart.h"
#include "nrf.h"


class Serial
{
private:
    char _terminator = '\n';
    uint16_t _rxIndex=0;
    
    IRQn_Type _irq;
    
    const uint32_t _rxPin;
    const uint32_t _txPin;
    NRF_UART_Type *_uart_driver;
    
public:
    Serial(const uint32_t rxPin = DEFAULT_RX_PIN, const uint32_t txPin = DEFAULT_TX_PIN, NRF_UART_Type *uart_driver = NRF_UART0);
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

