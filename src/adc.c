#include "adc.h"
#include "nrf_drv_saadc.h"

#define SAMPLE_BUFFER 4

void saadcEventHandler(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE) nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLE_BUFFER);
}

bool adcInit(void)
{
    nrf_drv_saadc_config_t config = NRF_DRV_SAADC_DEFAULT_CONFIG;
    static nrf_saadc_channel_config_t channel = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    nrf_drv_saadc_channel_init(0, &channel);
    return NRF_SUCCESS == nrf_drv_saadc_init(&config, saadcEventHandler);
}

int32_t getAdcValue(void)
{
    nrf_saadc_value_t  sample = -2000;
    for(int i = 0; i < SAADC_CONFIG_OVERSAMPLE*SAADC_CONFIG_OVERSAMPLE; i++) nrf_drv_saadc_sample_convert(0, &sample);
    if(sample == -2000) nrf_drv_saadc_sample_convert(0, &sample);
    return sample > 0 ? sample : 0 ; //values below ground are noise in our case.
}
