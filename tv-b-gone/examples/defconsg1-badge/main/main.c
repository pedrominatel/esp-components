#include "tvbgone_core.h"

#include <stdint.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TVBGONE_IR_LED_GPIO GPIO_NUM_19
#define TVBGONE_BUTTON_GPIO GPIO_NUM_10

#define BUTTON_POLL_MS 25

static const char *TAG = "defconsg1_badge";

static TaskHandle_t s_send_task;
static portMUX_TYPE s_send_lock = portMUX_INITIALIZER_UNLOCKED;

static void dcsg1b_send_task(void *arg);

static void dcsg1b_toggle_both_regions_send(void)
{
    bool start_task = false;

    taskENTER_CRITICAL(&s_send_lock);
    if (s_send_task == NULL) {
        start_task = true;
    }
    taskEXIT_CRITICAL(&s_send_lock);

    if (start_task) {
        BaseType_t send_task_created = xTaskCreate(dcsg1b_send_task, "tvbg_send", 4096,
                                                   (void *)(intptr_t)TVBGONE_CORE_REGION_BOTH,
                                                   5, &s_send_task);
        ESP_RETURN_VOID_ON_FALSE(send_task_created == pdPASS, TAG,
                                 "failed to create send task");
        return;
    }

    ESP_ERROR_CHECK(tvbgone_core_stop());
}

static void dcsg1b_send_task(void *arg)
{
    tvbgone_core_region_t region = (tvbgone_core_region_t)(intptr_t)arg;

    ESP_LOGI(TAG, "Starting BOTH-region send");

    esp_err_t err = tvbgone_core_send(region, TVBGONE_CORE_SEND_MODE_SINGLE);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Send complete");
    } else if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGI(TAG, "Send interrupted");
    } else {
        ESP_LOGE(TAG, "Send failed: %s", esp_err_to_name(err));
    }

    taskENTER_CRITICAL(&s_send_lock);
    s_send_task = NULL;
    taskEXIT_CRITICAL(&s_send_lock);

    vTaskDelete(NULL);
}

static void dcsg1b_button_task(void *arg)
{
    (void)arg;

    int last_level = gpio_get_level(TVBGONE_BUTTON_GPIO);

    while (true) {
        int level = gpio_get_level(TVBGONE_BUTTON_GPIO);

        if ((last_level == 1) && (level == 0)) {
            dcsg1b_toggle_both_regions_send();
        }

        last_level = level;
        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}

void app_main(void)
{
    tvbgone_core_config_t config;
    gpio_config_t input_gpio_cfg = {
        .pin_bit_mask = (1ULL << TVBGONE_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&input_gpio_cfg));

    tvbgone_core_get_default_config(&config);
    config.ir_led_gpio = TVBGONE_IR_LED_GPIO;
    ESP_ERROR_CHECK(tvbgone_core_init(&config));

    BaseType_t button_task_created = xTaskCreate(dcsg1b_button_task, "tvbg_btn", 3072, NULL, 5, NULL);
    ESP_RETURN_VOID_ON_FALSE(button_task_created == pdPASS, TAG,
                             "failed to create button task");
}
