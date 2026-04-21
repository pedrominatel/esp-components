#include "tvbgone_core.h"

#include <stdint.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TVBGONE_IR_LED_GPIO GPIO_NUM_19
#define TVBGONE_BUTTON_GPIO GPIO_NUM_10

#define BUTTON_POLL_MS 25
#define BSP_RMT_RESOLUTION_HZ 1000000U
#define BSP_RMT_MEM_BLOCK_SYMBOLS 128U

static const char *TAG = "defconsg1_badge";

static TaskHandle_t s_send_task;
static rmt_channel_handle_t s_bsp_ir_rmt_channel;
static portMUX_TYPE s_send_lock = portMUX_INITIALIZER_UNLOCKED;

static void dcsg1b_send_task(void *arg);
static esp_err_t dcsg1b_init_bsp_ir_channel(void);

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

static esp_err_t dcsg1b_init_bsp_ir_channel(void)
{
    rmt_tx_channel_config_t tx_chan_cfg = {
        .gpio_num = TVBGONE_IR_LED_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = BSP_RMT_RESOLUTION_HZ,
        .mem_block_symbols = BSP_RMT_MEM_BLOCK_SYMBOLS,
        .trans_queue_depth = 1,
        .intr_priority = 0,
        .flags = {
            .invert_out = 0,
            .with_dma = 0,
            .io_loop_back = 0,
            .io_od_mode = 0,
            .allow_pd = 0,
        },
    };

    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_chan_cfg, &s_bsp_ir_rmt_channel), TAG,
                        "failed to create BSP-owned RMT TX channel");
    ESP_RETURN_ON_ERROR(rmt_enable(s_bsp_ir_rmt_channel), TAG,
                        "failed to enable BSP-owned RMT TX channel");
    return ESP_OK;
}

void app_main(void)
{
    tvbgone_core_config_t config;

    // Init from the BSP side
    gpio_config_t input_gpio_cfg = {
        .pin_bit_mask = (1ULL << TVBGONE_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&input_gpio_cfg));
    // Still on the BSP side, we need to initialize the RMT channel that will be used by the component
    ESP_ERROR_CHECK(dcsg1b_init_bsp_ir_channel());

    // On the component side, we need to set the RMT channel mode to BORROWED and provide the externally initialized channel
    tvbgone_core_get_default_config(&config);
    config.rmt_channel_mode = TVBGONE_CORE_RMT_CHANNEL_MODE_BORROWED;
    config.external_rmt_channel = s_bsp_ir_rmt_channel;
    // Now with all the peripheral initialization done, we can initialize the component with the config
    ESP_ERROR_CHECK(tvbgone_core_init(&config));
    // Here we will create a task to pool the button state and trigger sends when it's pressed
    // Will be also managed by the BSP, so some changes might be needed on the BSP side to fit this in the main application task
    BaseType_t button_task_created = xTaskCreate(dcsg1b_button_task, "tvbg_btn", 3072, NULL, 5, NULL);
    ESP_RETURN_VOID_ON_FALSE(button_task_created == pdPASS, TAG,
                             "failed to create button task");
}
