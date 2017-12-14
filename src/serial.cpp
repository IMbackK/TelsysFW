#include "serial.h"

char rxBuffer[BUFFER_SIZE];
volatile uint32_t interruptIndex = 0;

extern "C"
{
    void UARTE0_UART0_IRQHandler()
    {
        if (NRF_UART0->EVENTS_RXDRDY)
            {
                rxBuffer[interruptIndex % BUFFER_SIZE] = NRF_UART0->RXD;
                interruptIndex++;
                NRF_UART0->EVENTS_RXDRDY = 0x0UL;
            }
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
    return (interruptIndex > _rxIndex);
}

char Serial::getChar()
{
    if( _rxIndex >= (32768) - 2*BUFFER_SIZE ) flush(); //may explode only occasionaly
    if(dataIsWaiting())
    {
        _rxIndex++;
        return rxBuffer[(_rxIndex -1) % BUFFER_SIZE];
    }
    else return '\0';
}

unsigned int Serial::getString(char* buffer, const int bufferLength)
{
    int i = 0;
    for(; i <= (interruptIndex-_rxIndex) && i <= BUFFER_SIZE && rxBuffer[(_rxIndex+i) % BUFFER_SIZE] != _terminator; i++);
    
    if( i < (interruptIndex-_rxIndex) && i > 0)
    {
       int j = 0;
       for(; j < i &&  j < bufferLength-1 ; j++)
       {
           buffer[j] = getChar();
       }
       buffer[j+1]='\0';
       _rxIndex++;
    }
    else
    {
         i = 0;
         if( _rxIndex >= (32768) - 2*BUFFER_SIZE ) flush();
    }

    if (rxBuffer[(_rxIndex+i) % BUFFER_SIZE] == _terminator) _rxIndex++;
    
    return i;
}

void Serial::flush()
{
    _rxIndex = 0;
    interruptIndex = 0;
    for(int i = 0; i < BUFFER_SIZE; i++) rxBuffer[i] = ' ';
}

void Serial::setTerminator(char terminator){_terminator = terminator;}
