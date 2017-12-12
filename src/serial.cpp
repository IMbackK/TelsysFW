#include "serial.h"

char rxBuffer[BUFFER_SIZE];
volatile uint32_t interruptIndex = 0;
NRF_UART_Type *uart_driver = NRF_UART0;

extern "C"
{
    void UARTE0_UART0_IRQHandler()
    {
        if (uart_driver->EVENTS_RXDRDY)
            {
                rxBuffer[interruptIndex % BUFFER_SIZE] = uart_driver->RXD;
                interruptIndex++;
                uart_driver->EVENTS_RXDRDY = 0x0UL;
            }
    }
}


Serial::Serial() 
{
    //setup uart device
    uart_driver->PSELTXD = TX_PIN;
    uart_driver->PSELRXD = RX_PIN;
    uart_driver->CONFIG = (UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos) | UART_CONFIG_HWFC_Disabled;
    uart_driver->BAUDRATE = BAUD;
    uart_driver->ENABLE = UART_ENABLE_ENABLE_Enabled;
    uart_driver->EVENTS_RXDRDY = 0x0UL;
    uart_driver->EVENTS_TXDRDY = 0x0UL;
    uart_driver->TASKS_STARTRX = 0x1UL;
    uart_driver->TASKS_STARTTX = 0x1UL;
    uart_driver->INTENSET = UART_INTENSET_RXDRDY_Msk;
    NVIC_ClearPendingIRQ(irq);
    NVIC_SetPriority(irq, 3);
    NVIC_EnableIRQ(irq);
}

void Serial::putChar(const char c)
{
    uart_driver->TXD = c;

    while(!uart_driver->EVENTS_TXDRDY);

    uart_driver->EVENTS_TXDRDY = 0x0UL;
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
        bool flag = false;
        char str[64] = { 0 }; 
        int16_t i = 62;
        if (in < 0) 
        {
            flag = true;
            in = abs(in);
        }

        while (in != 0 && i > 0) 
        { 
            str[i--] = (in % 10) + '0';
            in /= 10;
        }

        if (flag) str[i--] = '-';
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
