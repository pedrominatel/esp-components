#include "tvbgone_core.h"

#include <stdint.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TVBGONE_IR_LED_GPIO GPIO_NUM_2
#define TVBGONE_BUTTON_NA_GPIO GPIO_NUM_10
#define TVBGONE_BUTTON_EU_GPIO GPIO_NUM_9
#define TVBGONE_VISIBLE_LED_GPIO GPIO_NUM_8
#define TVBGONE_VISIBLE_LED_ACTIVE_LOW true

#define BUTTON_POLL_MS 25

static const char *TAG = "tvbgone_example";

static TaskHandle_t s_send_task;
static volatile bool s_restart_requested;
static volatile tvbgone_core_region_t s_requested_region;
static portMUX_TYPE s_send_lock = portMUX_INITIALIZER_UNLOCKED;

static void send_task(void *arg);

static void set_visible_led(bool on)
{
    int level = on ? 1 : 0;
    if (TVBGONE_VISIBLE_LED_ACTIVE_LOW) {
        level = !level;
    }
    gpio_set_level(TVBGONE_VISIBLE_LED_GPIO, level);
}

static void request_region_send(tvbgone_core_region_t region)
{
    bool start_task = false;

    taskENTER_CRITICAL(&s_send_lock);
    if (s_send_task == NULL) {
        s_requested_region = region;
        s_restart_requested = false;
        start_task = true;
    } else {
        s_requested_region = region;
        s_restart_requested = true;
    }
    taskEXIT_CRITICAL(&s_send_lock);

    if (start_task) {
        BaseType_t send_task_created = xTaskCreate(send_task, "tvbg_send", 4096,
                                                   (void *)(intptr_t)region, 5,
                                                   &s_send_task);
        ESP_RETURN_VOID_ON_FALSE(send_task_created == pdPASS, TAG,
                                 "failed to create send task");
        return;
    }

    ESP_ERROR_CHECK(tvbgone_core_stop());
}

static void send_task(void *arg)
{
    tvbgone_core_region_t region = (tvbgone_core_region_t)(intptr_t)arg;

    while (true) {
        set_visible_led(true);
        ESP_LOGI(TAG, "Starting %s send",
                 (region == TVBGONE_CORE_REGION_NA) ? "NA" : "EU");

        esp_err_t err = tvbgone_core_send(region, TVBGONE_CORE_SEND_MODE_SINGLE);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Send complete");
        } else if (err == ESP_ERR_INVALID_STATE) {
            ESP_LOGI(TAG, "Send interrupted");
        } else {
            ESP_LOGE(TAG, "Send failed: %s", esp_err_to_name(err));
        }

        set_visible_led(false);

        taskENTER_CRITICAL(&s_send_lock);
        if (s_restart_requested) {
            region = s_requested_region;
            s_restart_requested = false;
            taskEXIT_CRITICAL(&s_send_lock);
            continue;
        }

        s_send_task = NULL;
        taskEXIT_CRITICAL(&s_send_lock);
        break;
    }

    vTaskDelete(NULL);
}

static void button_task(void *arg)
{
    (void)arg;

    int last_na_level = gpio_get_level(TVBGONE_BUTTON_NA_GPIO);
    int last_eu_level = gpio_get_level(TVBGONE_BUTTON_EU_GPIO);

    while (true) {
        int na_level = gpio_get_level(TVBGONE_BUTTON_NA_GPIO);
        int eu_level = gpio_get_level(TVBGONE_BUTTON_EU_GPIO);

        if ((last_na_level == 1) && (na_level == 0)) {
            request_region_send(TVBGONE_CORE_REGION_NA);
        }

        if ((last_eu_level == 1) && (eu_level == 0)) {
            request_region_send(TVBGONE_CORE_REGION_EU);
        }

        last_na_level = na_level;
        last_eu_level = eu_level;
        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}

void app_main(void)
{
    tvbgone_core_config_t config;
    gpio_config_t output_gpio_cfg = {
        .pin_bit_mask = (1ULL << TVBGONE_VISIBLE_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config_t input_gpio_cfg = {
        .pin_bit_mask = (1ULL << TVBGONE_BUTTON_NA_GPIO) | (1ULL << TVBGONE_BUTTON_EU_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&output_gpio_cfg));
    ESP_ERROR_CHECK(gpio_config(&input_gpio_cfg));
    set_visible_led(false);

    tvbgone_core_get_default_config(&config);
    config.ir_led_gpio = TVBGONE_IR_LED_GPIO;
    config.rmt_channel_mode = TVBGONE_CORE_RMT_CHANNEL_MODE_INTERNAL;
    ESP_ERROR_CHECK(tvbgone_core_init(&config));

    BaseType_t button_task_created = xTaskCreate(button_task, "tvbg_btn", 3072, NULL, 5, NULL);
    ESP_RETURN_VOID_ON_FALSE(button_task_created == pdPASS, TAG,
                             "failed to create button task");
}
