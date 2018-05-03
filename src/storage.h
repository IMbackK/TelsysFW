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
#include <stdint.h>
#include <stdbool.h>

#define FLASH_STORE_PAGES 32

#ifdef __cplusplus
extern "C" {
#endif

bool flash_storage_init(uint32_t pages);

void flash_store(uint32_t addr, uint32_t data, bool async);

bool flash_erase(uint32_t addr, uint32_t count, bool async);

uint32_t flash_read(uint32_t addr);

bool flash_operation_has_finished();

#ifdef __cplusplus
}
#endif
