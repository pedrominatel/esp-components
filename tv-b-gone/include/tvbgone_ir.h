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

esp_err_t tvbgone_ir_init(const tvbgone_ir_config_t *config);
esp_err_t tvbgone_ir_deinit(void);

esp_err_t tvbgone_ir_set_mode(tvbgone_ir_mode_t mode);
tvbgone_ir_mode_t tvbgone_ir_get_mode(void);

esp_err_t tvbgone_ir_send_once(void);
esp_err_t tvbgone_ir_start(void);
esp_err_t tvbgone_ir_stop(TickType_t timeout_ticks);

bool tvbgone_ir_is_running(void);
