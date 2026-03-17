/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Generic ESP-IDF component template
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

/**
 * @brief Opaque template handle
 */
typedef struct template_dev_t *template_handle_t;

/**
 * @brief Create a template component instance.
 *
 * Replace this placeholder with the initialization entry point required by
 * your real component.
 *
 * @return template_handle_t Handle on success, or NULL on failure.
 */
template_handle_t template_create(void);

/**
 * @brief Delete a template component instance.
 *
 * Replace this placeholder with the teardown logic required by your real
 * component.
 *
 * @param handle Handle returned by template_create().
 * @return ESP_OK on success, or an error code.
 */
esp_err_t template_delete(template_handle_t handle);

#ifdef __cplusplus
}
#endif
