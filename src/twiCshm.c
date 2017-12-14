#include "twiCshm.h"

#define TWI0_USE_EASY_DMA 0
#include "nrf_drv_twi.h"
#include "nrf_twi.h"

bool twiCshm_rxFinished;

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
    const nrf_drv_twi_config_t conf = NRF_DRV_TWI_DEFAULT_CONFIG;
    /*config.scl=sclPin;
    config.sda=sdaPin;
    config.frequency=NRF_TWI_FREQ_400K;*/
    
    nrf_drv_twi_init(&twiNrfDrf, &conf, twi_event_handler, NULL);
    nrf_drv_twi_enable(&twiNrfDrf);
}

bool twiCshm_drv_twi_tx(const uint8_t address, const uint8_t* in, const uint8_t length, const bool stop)
{
    ret_code_t ret = nrf_drv_twi_tx(&twiNrfDrf,address, in, length, stop);
    return ret == NRF_SUCCESS ? true : false;
}

bool twiCshm_drv_twi_rx(const uint8_t address, uint8_t* data,const uint8_t length)
{
    twiCshm_rxFinished = false;
    nrf_drv_twi_rx(&twiNrfDrf, address, data, length);
    while(!twiCshm_rxFinished);
    return true;
}
