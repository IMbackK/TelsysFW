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
                                    
    //adc config
    #define ADC_CH_CONFIG \
    {                                                  \
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,     \
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,     \
        .gain       = NRF_SAADC_GAIN1_2,               \
        .reference  = NRF_SAADC_REFERENCE_VDD4,        \
        .acq_time   = NRF_SAADC_ACQTIME_40US,          \
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,     \
        .burst      = NRF_SAADC_BURST_ENABLED,         \
        .pin_p      = NRF_SAADC_INPUT_AIN0,            \
        .pin_n      = NRF_SAADC_INPUT_DISABLED         \
    }
    
    //serial pins
    #define SERIAL_ENABLED      1
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
                                    
    //adc config
    #define ADC_CH_CONFIG \
    {                                                  \
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,     \
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,     \
        .gain       = NRF_SAADC_GAIN1_2,               \
        .reference  = NRF_SAADC_REFERENCE_VDD4,         \
        .acq_time   = NRF_SAADC_ACQTIME_40US,          \
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,     \
        .burst      = NRF_SAADC_BURST_ENABLED,         \
        .pin_p      = NRF_SAADC_INPUT_AIN0,            \
        .pin_n      = NRF_SAADC_INPUT_DISABLED         \
    }
    
    //serial pins
    #define SERIAL_ENABLED      1
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
    #define NRF_CLOCK_LFCLKSRC \
    {                                                       \
        .source        = NRF_CLOCK_LF_SRC_RC,               \
        .rc_ctiv       = 16,                                \
        .rc_temp_ctiv  = 2,                                 \
        .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM  \
    }
                                    
    //adc config
    #define ADC_CH_CONFIG \
    {                                                  \
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,     \
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,     \
        .gain       = NRF_SAADC_GAIN1_2,               \
        .reference  = NRF_SAADC_REFERENCE_VDD4,        \
        .acq_time   = NRF_SAADC_ACQTIME_40US,          \
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,     \
        .burst      = NRF_SAADC_BURST_ENABLED,         \
        .pin_p      = NRF_SAADC_INPUT_AIN0,            \
        .pin_n      = NRF_SAADC_INPUT_DISABLED         \
    }
    
    //serial
    //#define SERIAL_ENABLED      1
    #define DEFAULT_RX_PIN      28
    #define DEFAULT_TX_PIN      30

    //i2c pins
    #define DEFAULT_SCL_PIN     5
    #define DEFAULT_SDA_PIN     4
    
    //gipo pins
    #define LED_PIN             11
    #define SAMPLE_CLOCK_PIN    2
#endif
