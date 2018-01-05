#include "adc.h"
#include "nrf_drv_adc.h"

bool adcInit()
{
    nrf_drv_adc_config_t config = NRF_DRV_ADC_DEFAULT_CONFIG;
    nrf_drv_adc_init(&config, NULL);
}

int32_t getAdcValue()
{
    static nrf_drv_adc_channel_t channel = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_1);
    return nrf_adc_convert_single(&channel);
}
