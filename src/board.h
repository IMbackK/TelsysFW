#pragma once

#ifdef BOARD_FEATHER

    //timer config
    #define APP_TIMER_PRESCALER             0    //Value of the RTC1 PRESCALER register.
    #define APP_TIMER_OP_QUEUE_SIZE         5    //Size of timer operation queues.

    //clock source for SoftDevice
    #define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                    .rc_ctiv       = 0,                                \
                                    .rc_temp_ctiv  = 0,                                \
                                    .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}
    //serial pins
    #define DEFAULT_RX_PIN 8
    #define DEFAULT_TX_PIN 6

    //i2c pins
    #define DEFAULT_SCL_PIN      26
    #define DEFAULT_SDA_PIN      25
    
    //gipo pins
    #define LED_PIN              17
    #define SAMPLE_CLOCK_PIN      7
    
#endif

#ifdef BOARD_REDBEAR

    //timer config
    #define APP_TIMER_PRESCALER             0    //Value of the RTC1 PRESCALER register.
    #define APP_TIMER_OP_QUEUE_SIZE         5    //Size of timer operation queues.

    //clock source for SoftDevice
    #define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_RC,            \
                                    .rc_ctiv       = 16,                                \
                                    .rc_temp_ctiv  = 2,                                \
                                    .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}
    //serial pins
    #define DEFAULT_RX_PIN      28
    #define DEFAULT_TX_PIN      29

    //i2c pins
    #define DEFAULT_SCL_PIN     5
    #define DEFAULT_SDA_PIN     4
    
    //gipo pins
    #define LED_PIN             11
    #define SAMPLE_CLOCK_PIN    2
#endif

#ifdef BOARD_PROTOONE

    //timer config
    #define APP_TIMER_PRESCALER             0    //Value of the RTC1 PRESCALER register.
    #define APP_TIMER_OP_QUEUE_SIZE         5    //Size of timer operation queues.

    //clock source for SoftDevice
    #define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_RC,            \
                                    .rc_ctiv       = 16,                                \
                                    .rc_temp_ctiv  = 2,                                \
                                    .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}
    //serial pins
    #define DEFAULT_RX_PIN      28
    #define DEFAULT_TX_PIN      30

    //i2c pins
    #define DEFAULT_SCL_PIN     5
    #define DEFAULT_SDA_PIN     4
    
    //gipo pins
    #define LED_PIN             11
    #define SAMPLE_CLOCK_PIN    2
#endif
