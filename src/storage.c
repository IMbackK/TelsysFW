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

#include "storage.h"
#include <fstorage.h>
#include <stddef.h>

static bool fs_operation_finished;

static void fs_evt_handler(fs_evt_t const * const evt, fs_ret_t result)
{
    fs_operation_finished = true;
}

FS_REGISTER_CFG(fs_config_t flash_config) =
{
                .callback  = fs_evt_handler,
                .num_pages = FLASH_STORE_PAGES,
                .priority  = 0xFE
};

bool flash_storage_init(uint32_t pages)
{
    fs_operation_finished = false;
    fs_ret_t error = fs_init();
    return error == FS_SUCCESS;
}

void flash_store(uint32_t addr, uint32_t data, bool async)
{
    fs_operation_finished = false;
    fs_store(&flash_config, flash_config.p_start_addr+addr, &data, 1, NULL);
    if(async)while(!fs_operation_finished);
}

bool flash_erase(uint32_t addr, uint32_t count, bool async)
{
    fs_operation_finished = false;
    fs_erase(&flash_config, flash_config.p_start_addr+addr, count, NULL);
    if(async)while(!fs_operation_finished);
    return true;
}

bool flash_operation_has_finished()
{
    return fs_operation_finished;
}

uint32_t flash_read(uint32_t addr)
{
    return *(flash_config.p_start_addr + addr);
}
