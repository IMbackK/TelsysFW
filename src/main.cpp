extern "C"
{
    #define __STATIC_INLINE static inline
    
    //Standard libraries
    #include <stdint.h>
    #include <stddef.h>


    //SDK headers
    #include "nrf.h"
    #include "nrf_gpiote.h"
    #include "nrf_gpio.h"
    #include "boards.h"
    //#include "nrf_drv_ppi.h"
    #include "nrf_delay.h"
    #include "nrf_drv_gpiote.h"
    #include "app_error.h"
   
}

#include "serial.h"
#include "adc.h"
#include "point3D.h"
#include "twiCshm.h"
#include "MPU9150.h"
#include <string.h>



int main()
{
    if(!nrf_drv_gpiote_is_init())nrf_drv_gpiote_init(); //initalize gpio driver
    nrf_drv_gpiote_out_config_t pin30conf;
    pin30conf.init_state = NRF_GPIOTE_INITIAL_VALUE_LOW;
    pin30conf.task_pin = false;
    
    nrf_drv_gpiote_out_init(17, &pin30conf);
    
    //APP_TIMER_INIT(0, APP_TIMER_OP_QUEUE_SIZE, false); //start timer with 0 preescaler and que size 4.
    
    
    Serial serial;
    
    
    serial.write("adc init \n");
    bool adcSucsess = adcInit();
    
    serial.write("result: ");
    adcSucsess ? serial.write("true") : serial.write("false");
    serial.putChar('\n');
    
    serial.write("mpu init \n");
    //twiCshmInit(27,30);
    /*uint8_t powerOnSeq[2] = {0x6B, 0};
    twiCshm_drv_twi_tx(0x68, powerOnSeq, 2, true );
    
    uint8_t tmpChar = 0x3B;
    twiCshm_drv_twi_tx(0x68, &tmpChar, 1, false );
    twiCshm_drv_twi_rx(0x68, &tmpChar, 1);*/
    //twiCshm_drv_twi_tx(0x68, NULL, 0, true );
    Mpu9150 mpu;
    mpu.start();
    
    serial.write("going to loop \n");
    
    while (1)
    {
        serial.write("ADC value: ");
        serial.write(getAdcValue());
        serial.putChar('\n');
        
        Point3D <int16_t> temp = mpu.getAccelData();
        serial.write("Accel vector: x = ");
        serial.write(temp.x);
        serial.write(" y = ");
        serial.write(temp.y);
        serial.write(" z = ");
        serial.write(temp.z);
        serial.putChar('\n');
        
        temp = mpu.getMagnData();
        serial.write("Magnetic vector: x = ");
        serial.write(temp.x);
        serial.write(" y = ");
        serial.write(temp.y);
        serial.write(" z = ");
        serial.write(temp.z);
        serial.putChar('\n');
        
        serial.write("temperature: ");
        serial.write(mpu.getTemperature());
        serial.write("c\n");
        
        serial.putChar('\n');
        
        nrf_drv_gpiote_out_toggle(17);
        nrf_delay_ms(500);
    }
    
    return 0;
}
