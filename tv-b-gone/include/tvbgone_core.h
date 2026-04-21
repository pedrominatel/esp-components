#pragma once

#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TVBGONE_CORE_DEFAULT_IRLED_GPIO GPIO_NUM_2
#define TVBGONE_CORE_DEFAULT_TASK_STACK_SIZE 4096U
#define TVBGONE_CORE_DEFAULT_TASK_PRIORITY 5U

typedef enum {
    TVBGONE_CORE_REGION_EU = 0,
    TVBGONE_CORE_REGION_NA,
    TVBGONE_CORE_REGION_BOTH,
} tvbgone_core_region_t;

typedef enum {
    TVBGONE_CORE_SEND_MODE_SINGLE = 0,
    TVBGONE_CORE_SEND_MODE_CONTINUOUS,
} tvbgone_core_send_mode_t;

typedef struct {
    gpio_num_t ir_led_gpio;
    uint32_t task_stack_size;
    UBaseType_t task_priority;
} tvbgone_core_config_t;

void tvbgone_core_get_default_config(tvbgone_core_config_t *config);
esp_err_t tvbgone_core_init(const tvbgone_core_config_t *config);
esp_err_t tvbgone_core_send(tvbgone_core_region_t region, tvbgone_core_send_mode_t mode);
esp_err_t tvbgone_core_stop(void);
esp_err_t tvbgone_core_deinit(void);

#ifdef __cplusplus
}
#endif
