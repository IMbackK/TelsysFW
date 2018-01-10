/*UVOS This Shim ist needed because nrf_drv_twi cannot be drectly used from c++ due to operator overloading.*/

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
