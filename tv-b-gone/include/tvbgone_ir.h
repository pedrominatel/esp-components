#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifndef CONFIG_TVBGONE_IR_TX_GPIO
#define CONFIG_TVBGONE_IR_TX_GPIO 4
#endif

typedef enum {
    TVBGONE_IR_MODE_NA = 0,
    TVBGONE_IR_MODE_EU,
    TVBGONE_IR_MODE_BOTH,
} tvbgone_ir_mode_t;

typedef struct {
    int gpio_num;
    uint32_t resolution_hz;
    uint32_t tx_queue_depth;
    uint32_t code_gap_ms;
    uint32_t sweep_gap_ms;
} tvbgone_ir_config_t;

#define TVBGONE_IR_DEFAULT_CONFIG()                 \
    {                                               \
        .gpio_num = CONFIG_TVBGONE_IR_TX_GPIO,      \
        .resolution_hz = 100000,                    \
        .tx_queue_depth = 2,                        \
        .code_gap_ms = 205,                         \
        .sweep_gap_ms = 5000,                       \
    }

/**
 * @brief Initialize the TV-B-Gone IR transmitter.
 *
 * Creates and enables the RMT TX channel and prepares the component for use.
 * If @p config is NULL, the default configuration is used.
 *
 * @param[in] config Optional runtime configuration.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if the configuration is invalid
 *      - ESP_ERR_INVALID_STATE if the component is already initialized
 *      - ESP_ERR_NO_MEM if required resources cannot be allocated
 *      - Other ESP-IDF error codes returned by the RMT driver
 */
esp_err_t tvbgone_ir_init(const tvbgone_ir_config_t *config);

/**
 * @brief Deinitialize the TV-B-Gone IR transmitter.
 *
 * The component must be initialized and not currently running.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the component is not initialized or is still running
 */
esp_err_t tvbgone_ir_deinit(void);

/**
 * @brief Select which regional code set is included in each sweep.
 *
 * The Xiaomi integrated code remains part of every sweep regardless of mode.
 *
 * @param[in] mode Regional sweep mode.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if @p mode is invalid
 *      - ESP_ERR_INVALID_STATE if the component is not initialized
 *      - ESP_ERR_TIMEOUT if the internal state lock cannot be acquired in time
 */
esp_err_t tvbgone_ir_set_mode(tvbgone_ir_mode_t mode);

/**
 * @brief Get the currently configured regional sweep mode.
 *
 * @return Current mode value.
 */
tvbgone_ir_mode_t tvbgone_ir_get_mode(void);

/**
 * @brief Send one complete TV-B-Gone sweep synchronously.
 *
 * Sends the integrated code set first, followed by the selected regional sets.
 * The component must be initialized and not already running in background mode.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the component is not initialized or is already running
 *      - ESP_ERR_TIMEOUT if the internal state lock cannot be acquired in time
 *      - Other ESP-IDF error codes returned by the RMT driver
 */
esp_err_t tvbgone_ir_send_once(void);

/**
 * @brief Start continuous TV-B-Gone sweeps in a background task.
 *
 * If the component is already running, this function returns ESP_OK.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the component is not initialized
 *      - ESP_ERR_NO_MEM if the background task cannot be created
 *      - ESP_ERR_TIMEOUT if the internal state lock cannot be acquired in time
 */
esp_err_t tvbgone_ir_start(void);

/**
 * @brief Request the background sweep task to stop and wait for it to exit.
 *
 * If the component is not running, this function returns ESP_OK.
 *
 * @param[in] timeout_ticks Maximum time to wait for the task to stop.
 *                          Use `portMAX_DELAY` to wait indefinitely.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the component is not initialized
 *      - ESP_ERR_TIMEOUT if stopping exceeds @p timeout_ticks or the internal state lock cannot be acquired in time
 */
esp_err_t tvbgone_ir_stop(TickType_t timeout_ticks);

/**
 * @brief Check whether the background sweep task is currently running.
 *
 * @return true if continuous mode is active, otherwise false.
 */
bool tvbgone_ir_is_running(void);
