#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "tvbgone_ir.h"

static const char *TAG = "TVBGONE_EXAMPLE";
static const gpio_num_t BUTTON_GPIO = GPIO_NUM_9;

static void button_task(void *arg)
{
    (void)arg;

    int last_level = gpio_get_level(BUTTON_GPIO);

    while (1) {
        int level = gpio_get_level(BUTTON_GPIO);
        if (last_level == 1 && level == 0) {
            // Simple debounce for mechanical button
            vTaskDelay(pdMS_TO_TICKS(25));
            if (gpio_get_level(BUTTON_GPIO) == 0) {
                if (tvbgone_ir_is_running()) {
                    esp_err_t err = tvbgone_ir_stop(pdMS_TO_TICKS(5000));
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "tvbgone_ir_stop failed: %s", esp_err_to_name(err));
                    } else {
                        ESP_LOGI(TAG, "TV-B-Gone stopped");
                    }
                } else {
                    esp_err_t err = tvbgone_ir_start();
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "tvbgone_ir_start failed: %s", esp_err_to_name(err));
                    } else {
                        ESP_LOGI(TAG, "TV-B-Gone started");
                    }
                }
                while (gpio_get_level(BUTTON_GPIO) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
        }
        last_level = level;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    tvbgone_ir_config_t config = TVBGONE_IR_DEFAULT_CONFIG();
    config.code_gap_ms = 205;
    config.sweep_gap_ms = 5000;

    ESP_LOGI(TAG, "Initializing TV-B-Gone IR on GPIO %d", config.gpio_num);
    esp_err_t err = tvbgone_ir_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "tvbgone_ir_init failed: %s", esp_err_to_name(err));
        return;
    }

    err = tvbgone_ir_set_mode(TVBGONE_IR_MODE_BOTH);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "tvbgone_ir_set_mode failed: %s", esp_err_to_name(err));
        tvbgone_ir_deinit();
        return;
    }

    gpio_config_t button_cfg = {
        .pin_bit_mask = 1ULL << BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&button_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "button gpio config failed: %s", esp_err_to_name(err));
        tvbgone_ir_deinit();
        return;
    }

    ESP_LOGI(TAG, "Press button on GPIO%d to start/stop TV-B-Gone", (int)BUTTON_GPIO);

    BaseType_t ok = xTaskCreate(button_task, "tvbgone_button", 3072, NULL, 5, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create button task");
        tvbgone_ir_deinit();
        return;
    }
}
