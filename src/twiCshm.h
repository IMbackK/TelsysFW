/*UVOS This Shim ist needed because nrf_drv_twi cannot be drectly used from c++ due to operator overloading.*/
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

extern bool twiCshm_rxFinished;

void twiCshmInit(uint32_t sclPin, uint32_t sdaPin);

void twiCshmDisable(void);

int32_t twiCshm_drv_twi_tx(const uint8_t deviceAddress, const uint8_t* in, const uint8_t length, const bool stop);

bool twiCshm_drv_twi_rx(const uint8_t deviceAddress, uint8_t* data, const uint8_t length);
 
#ifdef __cplusplus
}
#endif
