/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include "template.h"

struct template_dev_t {
    int reserved;
};

template_handle_t template_create(void)
{
    struct template_dev_t *dev = calloc(1, sizeof(struct template_dev_t));
    if (dev == NULL) {
        return NULL;
    }

    return dev;
}

esp_err_t template_delete(template_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    free(handle);
    return ESP_OK;
}
