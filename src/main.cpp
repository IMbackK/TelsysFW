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

int main()
{
    if(!nrf_drv_gpiote_is_init())nrf_drv_gpiote_init(); //initalize gpio driver
    nrf_drv_gpiote_out_config_t pin30conf;
    pin30conf.init_state = NRF_GPIOTE_INITIAL_VALUE_HIGH;
    pin30conf.task_pin = false;
    
    //APP_TIMER_INIT(0, APP_TIMER_OP_QUEUE_SIZE, false); //start timer with 0 preescaler and que size 4.
    
    Serial serial;
    
    
    
    nrf_drv_gpiote_out_init(17, &pin30conf);
    
    while (1)
    {
        serial.write("adc val: ");
        serial.write(getAdcValue());
        serial.putChar('\n');
        nrf_drv_gpiote_out_toggle(17);
        nrf_delay_ms(100);
    }
    
    return 0;
}
