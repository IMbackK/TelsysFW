#include "adc.h"
#include "nrf_drv_saadc.h"


bool adcInit()
{
    nrf_drv_saadc_config_t config = NRF_DRV_SAADC_DEFAULT_CONFIG;
    static nrf_saadc_channel_config_t channel = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    nrf_drv_saadc_channel_init(0, &channel);
    return NRF_SUCCESS == nrf_drv_saadc_init(&config, NULL);
}

int32_t getAdcValue()
{
    nrf_saadc_value_t  sample = -1;
    nrf_drv_saadc_sample_convert(0, &sample);
    return sample;
}
