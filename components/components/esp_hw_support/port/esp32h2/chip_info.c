/*
 * SPDX-FileCopyrightText: 2013-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "esp_chip_info.h"
#include "hal/efuse_hal.h"

void esp_chip_info(esp_chip_info_t *out_info)
{
    memset(out_info, 0, sizeof(*out_info));
    out_info->model = CHIP_ESP32H2;
    out_info->full_revision = efuse_hal_chip_revision();
    out_info->revision = efuse_hal_get_major_chip_version();
    out_info->cores = 1;
    out_info->features = CHIP_FEATURE_IEEE802154 | CHIP_FEATURE_BLE;
}