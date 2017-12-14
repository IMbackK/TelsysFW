/*This Shim ist needed because nrf_drv_twi cannot be drectly used from c++ due to operator overloading.*/

#pragma once

#include <stdint.h>
#include <stdbool.h>

extern bool twiCshm_rxFinished;

void twiCshmInit(uint32_t sclPin, uint32_t sdaPin);

void twiCshmDisable(void);

bool twiCshm_drv_twi_tx(uint8_t address, const uint8_t* in, uint8_t length, bool stop);

bool twiCshm_drv_twi_rx(const uint8_t address, uint8_t* data, const uint8_t length);
