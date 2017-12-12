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

int main()
{
    if(!nrf_drv_gpiote_is_init())nrf_drv_gpiote_init(); //initalize gpio driver
    nrf_drv_gpiote_out_config_t pin30conf;
    pin30conf.init_state = NRF_GPIOTE_INITIAL_VALUE_HIGH;
    pin30conf.task_pin = false;
    
    nrf_drv_gpiote_out_init(30, &pin30conf);
    
    while (1)
    {
        nrf_drv_gpiote_out_toggle(30);
        nrf_delay_ms(10);
    }
    
    return 0;
}
