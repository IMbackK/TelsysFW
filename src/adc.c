#include "adc.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "app_error.h"

#include "app_util_platform.h"

#define SAMPLE_BUFFER_SIZE 128

static const nrf_drv_timer_t timer           = NRF_DRV_TIMER_INSTANCE(1);
static nrf_ppi_channel_t     ppi;
static int16_t     buffer[2][SAMPLE_BUFFER_SIZE]; //double buffer 

static void  (*_callbackFunction)(int16_t* buffer, uint16_t length );

static bool async = false;

void noopTimerHandler(nrf_timer_event_t event_type, void * p_context){}

void saadcEventHandler(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE) 
    {
        _callbackFunction(p_event->data.done.p_buffer, SAMPLE_BUFFER_SIZE);
        nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLE_BUFFER_SIZE); //continue sampleing on buffer
    }
}

bool adcInit(void)
{
    nrf_drv_saadc_config_t config = NRF_DRV_SAADC_DEFAULT_CONFIG;
    static nrf_saadc_channel_config_t channel = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    channel.acq_time = NRF_SAADC_ACQTIME_3US;
    nrf_drv_saadc_channel_init(0, &channel);
    return NRF_SUCCESS == nrf_drv_saadc_init(&config, saadcEventHandler);
}

int32_t adcCreateAsyncTask(void (*callbackFunction)(int16_t* buffer, uint16_t length ))
{
    nrf_drv_saadc_buffer_convert(buffer[0], SAMPLE_BUFFER_SIZE); //start sampleing on first buffer
    nrf_drv_saadc_buffer_convert(buffer[1], SAMPLE_BUFFER_SIZE); //start sampleing on second buffer
    
    int32_t errorCode = 0;
    _callbackFunction = callbackFunction;
    errorCode = nrf_drv_ppi_init(); //Init Programmable Peripheral Interconnect (this is safe if PPI is already inited elswhere)
    if(errorCode != NRF_SUCCESS) return errorCode;
    
    nrf_drv_timer_config_t timerConfig = NRF_DRV_TIMER_DEFAULT_CONFIG;
    errorCode = nrf_drv_timer_init(&timer, &timerConfig, noopTimerHandler);
    if(errorCode != NRF_SUCCESS) return errorCode;
    
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&timer, 1000); //8kHz
    nrf_drv_timer_extended_compare(&timer,NRF_TIMER_CC_CHANNEL0,ticks,NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,false);
    nrf_drv_timer_enable(&timer);

    errorCode = nrf_drv_ppi_channel_alloc(&ppi);
    if(errorCode != NRF_SUCCESS) return errorCode;
    errorCode = nrf_drv_ppi_channel_assign(ppi, nrf_drv_timer_compare_event_address_get(&timer, NRF_TIMER_CC_CHANNEL0), nrf_drv_saadc_sample_task_get());
    if(errorCode != NRF_SUCCESS) return errorCode;
    errorCode = nrf_drv_ppi_channel_enable(ppi);
    if(errorCode != NRF_SUCCESS) return errorCode;
    
    async = true;
    return 0;
}

int32_t getAdcValue(void)
{
    nrf_saadc_value_t  sample = -2000;
    if(!async)
    {
        for(int i = 0; i < SAADC_CONFIG_OVERSAMPLE*SAADC_CONFIG_OVERSAMPLE; i++) nrf_drv_saadc_sample_convert(0, &sample);
        if(sample == -2000) nrf_drv_saadc_sample_convert(0, &sample);
        return sample > 0 ? sample*2 : 0 ; //values below ground are noise in our case.
    }
    else return sample*2;
}
