#include "twiCshm.h"

#define TWI0_USE_EASY_DMA 0
#include "nrf_drv_twi.h"

const nrf_drv_twi_t twiNrfDrf = NRF_DRV_TWI_INSTANCE(0);

void twi_event_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    if(p_event->type == NRF_DRV_TWI_EVT_DONE && p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
    {
        twiCshm_rxFinished = true;
    }
}

void twiCshmDisable(void)
{
    nrf_drv_twi_disable(&twiNrfDrf);
}

void twiCshmInit(uint32_t sclPin, uint32_t sdaPin)
{
    const nrf_drv_twi_config_t conf = 
    {
        .scl                = sclPin,
        .sda                = sdaPin,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = 1,
        .clear_bus_init     = true,
        .hold_bus_uninit    = false
    };
    
    nrf_drv_twi_init(&twiNrfDrf, &conf, NULL, NULL);
    nrf_drv_twi_enable(&twiNrfDrf);
} 
int32_t twiCshm_drv_twi_tx(const uint8_t deviceAddress, const uint8_t* in, const uint8_t length, const bool stop)
{
    ret_code_t ret = nrf_drv_twi_tx(&twiNrfDrf, deviceAddress, in, length, !stop);
    return ret;
}

bool twiCshm_drv_twi_rx(const uint8_t deviceAddress, uint8_t* data, const uint8_t length)
{
    ret_code_t ret = nrf_drv_twi_rx(&twiNrfDrf, deviceAddress, data, length);
    return ret == NRF_SUCCESS ? true : false;
}
