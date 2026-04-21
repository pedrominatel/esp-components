/*
  TV-B-Gone reusable component for ESP-IDF.

  The core owns only IR transmission and lifecycle. Board-specific button and
  visible-LED behavior lives in examples or application code.
*/

#include "tvbgone_core.h"

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "driver/rmt_common.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "tvbgone_core_internal.h"

static const char *TAG = "tvbgone_core";

#define LOW 0

#define TIME_BETWEEN_CODES_MS 205

#define RMT_RESOLUTION_HZ 1000000U
#define RMT_MEM_BLOCK_SYMBOLS 128
#define RMT_MAX_DURATION_TICKS 32767U
#define RMT_MAX_SYMBOLS 1280U

typedef struct {
    tvbgone_core_config_t config;
    rmt_channel_handle_t ir_rmt_channel;
    rmt_encoder_handle_t ir_copy_encoder;
    rmt_symbol_word_t ir_symbols[RMT_MAX_SYMBOLS];
    TaskHandle_t task_handle;
    volatile bool initialized;
    volatile bool sending;
    volatile bool stop_requested;
    tvbgone_core_region_t active_region;
    tvbgone_core_send_mode_t active_mode;
    uint16_t current_code_number;
    uint16_t total_codes;
} tvbgone_core_ctx_t;

static tvbgone_core_ctx_t s_ctx = {
    .config = {
        .ir_led_gpio = TVBGONE_CORE_DEFAULT_IRLED_GPIO,
        .task_stack_size = TVBGONE_CORE_DEFAULT_TASK_STACK_SIZE,
        .task_priority = TVBGONE_CORE_DEFAULT_TASK_PRIORITY,
        .rmt_channel_mode = TVBGONE_CORE_RMT_CHANNEL_MODE_INTERNAL,
        .external_rmt_channel = NULL,
    },
    .active_region = TVBGONE_CORE_REGION_NA,
    .active_mode = TVBGONE_CORE_SEND_MODE_SINGLE,
};

static portMUX_TYPE s_ctx_lock = portMUX_INITIALIZER_UNLOCKED;

static inline bool tvbgone_stop_requested(void)
{
    return s_ctx.stop_requested;
}

static inline bool tvbgone_owns_rmt_channel(void)
{
    return s_ctx.config.rmt_channel_mode == TVBGONE_CORE_RMT_CHANNEL_MODE_INTERNAL;
}

static uint16_t get_total_codes_for_region(tvbgone_core_region_t region)
{
    if (region == TVBGONE_CORE_REGION_BOTH) {
        return (uint16_t)num_NAcodes + (uint16_t)num_EUcodes;
    }

    return (region == TVBGONE_CORE_REGION_NA) ? (uint16_t)num_NAcodes : (uint16_t)num_EUcodes;
}

static void set_current_code_number(uint16_t code_number)
{
    taskENTER_CRITICAL(&s_ctx_lock);
    s_ctx.current_code_number = code_number;
    taskEXIT_CRITICAL(&s_ctx_lock);
}

static esp_err_t begin_send(tvbgone_core_region_t region, tvbgone_core_send_mode_t mode)
{
    esp_err_t err = ESP_OK;

    taskENTER_CRITICAL(&s_ctx_lock);
    if (!s_ctx.initialized) {
        err = ESP_ERR_INVALID_STATE;
    } else if (s_ctx.sending) {
        err = ESP_ERR_INVALID_STATE;
    } else {
        s_ctx.sending = true;
        s_ctx.stop_requested = false;
        s_ctx.active_region = region;
        s_ctx.active_mode = mode;
        s_ctx.current_code_number = 0;
        s_ctx.total_codes = get_total_codes_for_region(region);
    }
    taskEXIT_CRITICAL(&s_ctx_lock);

    return err;
}

static void finish_send(void)
{
    taskENTER_CRITICAL(&s_ctx_lock);
    s_ctx.sending = false;
    s_ctx.stop_requested = false;
    s_ctx.current_code_number = 0;
    s_ctx.task_handle = NULL;
    taskEXIT_CRITICAL(&s_ctx_lock);
}

static size_t append_half_level(rmt_symbol_word_t *symbols, size_t symbol_index,
                                bool *has_pending_half, bool *pending_level,
                                uint16_t *pending_duration, uint32_t duration_us,
                                bool level)
{
    while (duration_us > 0) {
        uint16_t chunk_ticks = (duration_us > RMT_MAX_DURATION_TICKS)
                                   ? (uint16_t)RMT_MAX_DURATION_TICKS
                                   : (uint16_t)duration_us;

        if (!*has_pending_half) {
            *pending_level = level;
            *pending_duration = chunk_ticks;
            *has_pending_half = true;
        } else {
            symbols[symbol_index].level0 = *pending_level;
            symbols[symbol_index].duration0 = *pending_duration;
            symbols[symbol_index].level1 = level;
            symbols[symbol_index].duration1 = chunk_ticks;
            symbol_index++;
            *has_pending_half = false;
        }

        duration_us -= chunk_ticks;
    }

    return symbol_index;
}

static size_t build_power_code_symbols(rmt_symbol_word_t *symbols, size_t max_symbols,
                                       int num_pairs, uint32_t *pairs_tab_ptr,
                                       uint8_t *sequence_tab_ptr)
{
    size_t symbol_index = 0;
    bool has_pending_half = false;
    bool pending_level = false;
    uint16_t pending_duration = 0;

    for (int i = 0; i < num_pairs; i++) {
        uint8_t pairs_index = sequence_tab_ptr[i] * 2;
        uint32_t on_time = pairs_tab_ptr[pairs_index];
        uint32_t off_time = pairs_tab_ptr[pairs_index + 1];

        symbol_index = append_half_level(symbols, symbol_index, &has_pending_half,
                                         &pending_level, &pending_duration,
                                         on_time, true);
        symbol_index = append_half_level(symbols, symbol_index, &has_pending_half,
                                         &pending_level, &pending_duration,
                                         off_time, false);

        if (symbol_index >= max_symbols) {
            ESP_LOGE(TAG, "RMT symbol buffer overflow while encoding POWER-Code");
            return 0;
        }
    }

    if (has_pending_half) {
        if (symbol_index >= max_symbols) {
            ESP_LOGE(TAG, "RMT symbol buffer overflow while finalizing POWER-Code");
            return 0;
        }
        symbols[symbol_index].level0 = pending_level;
        symbols[symbol_index].duration0 = pending_duration;
        symbols[symbol_index].level1 = LOW;
        symbols[symbol_index].duration1 = 0;
        symbol_index++;
    }

    return symbol_index;
}

static void recover_rmt_channel(void)
{
    if (s_ctx.ir_rmt_channel == NULL) {
        return;
    }

    esp_err_t err = rmt_disable(s_ctx.ir_rmt_channel);
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        ESP_LOGW(TAG, "rmt_disable recovery failed: %s", esp_err_to_name(err));
    }

    err = rmt_enable(s_ctx.ir_rmt_channel);
    if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
        ESP_LOGW(TAG, "rmt_enable recovery failed: %s", esp_err_to_name(err));
    }
}

static esp_err_t wait_tx_done_interruptible(void)
{
    esp_err_t err = rmt_tx_wait_all_done(s_ctx.ir_rmt_channel, -1);
    if (err != ESP_OK) {
        return err;
    }

    if (tvbgone_stop_requested()) {
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

static esp_err_t delay_interruptible(uint32_t delay_ms)
{
    const TickType_t step_ticks = pdMS_TO_TICKS(10);
    TickType_t remaining_ticks = pdMS_TO_TICKS(delay_ms);

    while (remaining_ticks > 0) {
        if (tvbgone_stop_requested()) {
            return ESP_ERR_INVALID_STATE;
        }

        TickType_t sleep_ticks = (remaining_ticks > step_ticks) ? step_ticks : remaining_ticks;
        vTaskDelay(sleep_ticks);
        remaining_ticks -= sleep_ticks;
    }

    return ESP_OK;
}

static esp_err_t xmit_code_interruptible(uint32_t carrier_freq, int num_pairs,
                                         uint32_t *pairs_tab_ptr, uint8_t *sequence_tab_ptr)
{
    size_t symbol_count = build_power_code_symbols(s_ctx.ir_symbols, RMT_MAX_SYMBOLS,
                                                   num_pairs, pairs_tab_ptr,
                                                   sequence_tab_ptr);
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags = {
            .eot_level = LOW,
            .queue_nonblocking = 0,
        },
    };

    if (symbol_count == 0) {
        ESP_LOGE(TAG, "Skipping POWER-Code transmit because symbol build failed");
        return ESP_FAIL;
    }

    if (carrier_freq != 0) {
        rmt_carrier_config_t carrier_cfg = {
            .frequency_hz = carrier_freq,
            .duty_cycle = 0.5f,
            .flags = {
                .polarity_active_low = 0,
                .always_on = 0,
            },
        };
        ESP_RETURN_ON_ERROR(rmt_apply_carrier(s_ctx.ir_rmt_channel, &carrier_cfg), TAG,
                            "failed to apply carrier");
    } else {
        ESP_RETURN_ON_ERROR(rmt_apply_carrier(s_ctx.ir_rmt_channel, NULL), TAG,
                            "failed to clear carrier");
    }

    ESP_RETURN_ON_ERROR(rmt_transmit(s_ctx.ir_rmt_channel, s_ctx.ir_copy_encoder,
                                     s_ctx.ir_symbols,
                                     symbol_count * sizeof(s_ctx.ir_symbols[0]),
                                     &tx_config),
                        TAG, "failed to transmit POWER-Code");

    return wait_tx_done_interruptible();
}

static esp_err_t send_region_once(tvbgone_core_region_t region, uint16_t base_code_number)
{
    uint8_t num_codes = (region == TVBGONE_CORE_REGION_NA) ? num_NAcodes : num_EUcodes;
    struct IrCode **power_codes = (region == TVBGONE_CORE_REGION_NA) ? NApowerCodes : EUpowerCodes;
    const char *region_name = (region == TVBGONE_CORE_REGION_NA) ? "NA" : "EU";

    for (int power_code_count = 0; power_code_count < num_codes; power_code_count++) {
        struct IrCode *pwr_code_ptr;

        if (tvbgone_stop_requested()) {
            ESP_LOGI(TAG, "Stop requested before %s POWER-Code %d", region_name, power_code_count);
            return ESP_ERR_INVALID_STATE;
        }

        pwr_code_ptr = power_codes[power_code_count];
        set_current_code_number((uint16_t)(base_code_number + power_code_count + 1));

        ESP_LOGI(TAG, "Transmitting %s POWER-Code %d...", region_name, power_code_count);
        ESP_RETURN_ON_ERROR(xmit_code_interruptible(pwr_code_ptr->carrier_freq,
                                                    pwr_code_ptr->num_pairs,
                                                    pwr_code_ptr->pairs,
                                                    pwr_code_ptr->sequence),
                            TAG, "failed to transmit %s POWER-Code %d",
                            region_name, power_code_count);
        ESP_LOGI(TAG, "Done");

        if (power_code_count != (num_codes - 1)) {
            ESP_RETURN_ON_ERROR(delay_interruptible(TIME_BETWEEN_CODES_MS), TAG,
                                "stopped during transmit gap");
        }
    }

    return ESP_OK;
}

static esp_err_t send_selected_region(tvbgone_core_region_t region)
{
    ESP_LOGI(TAG, "Starting %s send",
             (region == TVBGONE_CORE_REGION_NA) ? "NA"
             : (region == TVBGONE_CORE_REGION_EU) ? "EU"
             : "BOTH");

    if (region == TVBGONE_CORE_REGION_BOTH) {
        ESP_RETURN_ON_ERROR(send_region_once(TVBGONE_CORE_REGION_NA, 0), TAG,
                            "BOTH send interrupted during NA region");
        ESP_RETURN_ON_ERROR(delay_interruptible(TIME_BETWEEN_CODES_MS), TAG,
                            "BOTH send interrupted between regions");
        ESP_RETURN_ON_ERROR(send_region_once(TVBGONE_CORE_REGION_EU, num_NAcodes), TAG,
                            "BOTH send interrupted during EU region");
        return ESP_OK;
    }

    return send_region_once(region, 0);
}

static void tvbgone_continuous_task(void *pv_parameters)
{
    tvbgone_core_region_t region = (tvbgone_core_region_t)(intptr_t)pv_parameters;

    while (!tvbgone_stop_requested()) {
        esp_err_t err = send_selected_region(region);
        if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
            ESP_LOGE(TAG, "continuous send failed: %s", esp_err_to_name(err));
            break;
        }
    }

    ESP_LOGI(TAG, "Continuous send stopped");
    finish_send();
    vTaskDelete(NULL);
}

static esp_err_t init_hardware(void)
{
    rmt_tx_channel_config_t tx_chan_cfg = {
        .gpio_num = s_ctx.config.ir_led_gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = RMT_MEM_BLOCK_SYMBOLS,
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
    rmt_copy_encoder_config_t copy_encoder_cfg = {};
    esp_err_t err;

    if (s_ctx.config.rmt_channel_mode == TVBGONE_CORE_RMT_CHANNEL_MODE_BORROWED) {
        s_ctx.ir_rmt_channel = s_ctx.config.external_rmt_channel;
        ESP_RETURN_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_cfg, &s_ctx.ir_copy_encoder), TAG,
                            "failed to create RMT encoder");
        ESP_LOGI(TAG, "Using externally initialized RMT TX channel");
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_chan_cfg, &s_ctx.ir_rmt_channel), TAG,
                        "failed to create RMT TX channel");

    err = rmt_new_copy_encoder(&copy_encoder_cfg, &s_ctx.ir_copy_encoder);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to create RMT encoder: %s", esp_err_to_name(err));
        rmt_del_channel(s_ctx.ir_rmt_channel);
        s_ctx.ir_rmt_channel = NULL;
        return err;
    }

    err = rmt_enable(s_ctx.ir_rmt_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to enable RMT channel: %s", esp_err_to_name(err));
        rmt_del_encoder(s_ctx.ir_copy_encoder);
        rmt_del_channel(s_ctx.ir_rmt_channel);
        s_ctx.ir_copy_encoder = NULL;
        s_ctx.ir_rmt_channel = NULL;
        return err;
    }

    ESP_LOGI(TAG, "TV-B-Gone IR hardware configuration done");
    return ESP_OK;
}

void tvbgone_core_get_default_config(tvbgone_core_config_t *config)
{
    if (config == NULL) {
        return;
    }

    *config = (tvbgone_core_config_t) {
        .ir_led_gpio = TVBGONE_CORE_DEFAULT_IRLED_GPIO,
        .task_stack_size = TVBGONE_CORE_DEFAULT_TASK_STACK_SIZE,
        .task_priority = TVBGONE_CORE_DEFAULT_TASK_PRIORITY,
        .rmt_channel_mode = TVBGONE_CORE_RMT_CHANNEL_MODE_INTERNAL,
        .external_rmt_channel = NULL,
    };
}

esp_err_t tvbgone_core_get_status(tvbgone_core_status_t *status)
{
    ESP_RETURN_ON_FALSE(status != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "status must not be null");

    taskENTER_CRITICAL(&s_ctx_lock);
    status->run_state = s_ctx.stop_requested ? TVBGONE_CORE_RUN_STATE_STOPPING
                        : s_ctx.sending   ? TVBGONE_CORE_RUN_STATE_RUNNING
                                          : TVBGONE_CORE_RUN_STATE_IDLE;
    status->region = s_ctx.active_region;
    status->current_code_number = s_ctx.current_code_number;
    status->total_codes = s_ctx.total_codes;
    taskEXIT_CRITICAL(&s_ctx_lock);

    return ESP_OK;
}

esp_err_t tvbgone_core_init(const tvbgone_core_config_t *config)
{
    tvbgone_core_config_t effective_config;

    taskENTER_CRITICAL(&s_ctx_lock);
    bool already_initialized = s_ctx.initialized;
    taskEXIT_CRITICAL(&s_ctx_lock);

    ESP_RETURN_ON_FALSE(!already_initialized, ESP_ERR_INVALID_STATE, TAG,
                        "component already initialized");

    if (config == NULL) {
        tvbgone_core_get_default_config(&effective_config);
    } else {
        effective_config = *config;
    }

    ESP_RETURN_ON_FALSE(effective_config.task_stack_size > 0, ESP_ERR_INVALID_ARG, TAG,
                        "task stack size must be greater than zero");
    ESP_RETURN_ON_FALSE(effective_config.rmt_channel_mode <= TVBGONE_CORE_RMT_CHANNEL_MODE_BORROWED,
                        ESP_ERR_INVALID_ARG, TAG, "invalid RMT channel mode");

    if (effective_config.rmt_channel_mode == TVBGONE_CORE_RMT_CHANNEL_MODE_INTERNAL) {
        ESP_RETURN_ON_FALSE(GPIO_IS_VALID_OUTPUT_GPIO(effective_config.ir_led_gpio),
                            ESP_ERR_INVALID_ARG, TAG, "invalid IR LED GPIO");
    } else {
        ESP_RETURN_ON_FALSE(effective_config.external_rmt_channel != NULL,
                            ESP_ERR_INVALID_ARG, TAG, "external RMT channel must not be null");
    }

    s_ctx.config = effective_config;
    ESP_RETURN_ON_ERROR(init_hardware(), TAG, "failed to initialize hardware");

    taskENTER_CRITICAL(&s_ctx_lock);
    s_ctx.initialized = true;
    taskEXIT_CRITICAL(&s_ctx_lock);

    return ESP_OK;
}

esp_err_t tvbgone_core_send(tvbgone_core_region_t region, tvbgone_core_send_mode_t mode)
{
    ESP_RETURN_ON_FALSE(region <= TVBGONE_CORE_REGION_BOTH, ESP_ERR_INVALID_ARG, TAG,
                        "invalid region");
    ESP_RETURN_ON_FALSE(mode <= TVBGONE_CORE_SEND_MODE_CONTINUOUS, ESP_ERR_INVALID_ARG, TAG,
                        "invalid send mode");
    ESP_RETURN_ON_ERROR(begin_send(region, mode), TAG, "send cannot start");

    if (mode == TVBGONE_CORE_SEND_MODE_SINGLE) {
        esp_err_t err = send_selected_region(region);
        if (err == ESP_ERR_INVALID_STATE && tvbgone_stop_requested()) {
            ESP_LOGI(TAG, "Single send stopped");
        } else if (err == ESP_OK) {
            ESP_LOGI(TAG, "Single send complete");
        }
        finish_send();
        return err;
    }

    BaseType_t task_created = xTaskCreate(tvbgone_continuous_task, "tvbgone_cont",
                                          s_ctx.config.task_stack_size,
                                          (void *)(intptr_t)region,
                                          s_ctx.config.task_priority,
                                          &s_ctx.task_handle);
    if (task_created != pdPASS) {
        finish_send();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Continuous send started");
    return ESP_OK;
}

esp_err_t tvbgone_core_stop(void)
{
    bool was_sending;

    taskENTER_CRITICAL(&s_ctx_lock);
    was_sending = s_ctx.sending;
    s_ctx.stop_requested = true;
    taskEXIT_CRITICAL(&s_ctx_lock);

    if (!was_sending) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stop requested");
    recover_rmt_channel();
    return ESP_OK;
}

esp_err_t tvbgone_core_deinit(void)
{
    ESP_RETURN_ON_FALSE(s_ctx.initialized, ESP_ERR_INVALID_STATE, TAG,
                        "component not initialized");
    ESP_RETURN_ON_FALSE(!s_ctx.sending, ESP_ERR_INVALID_STATE, TAG,
                        "cannot deinit while sending");

    if (s_ctx.ir_copy_encoder != NULL) {
        ESP_RETURN_ON_ERROR(rmt_del_encoder(s_ctx.ir_copy_encoder), TAG,
                            "failed to delete RMT encoder");
        s_ctx.ir_copy_encoder = NULL;
    }

    if (s_ctx.ir_rmt_channel != NULL) {
        if (tvbgone_owns_rmt_channel()) {
            esp_err_t err = rmt_disable(s_ctx.ir_rmt_channel);
            if ((err != ESP_OK) && (err != ESP_ERR_INVALID_STATE)) {
                return err;
            }

            ESP_RETURN_ON_ERROR(rmt_del_channel(s_ctx.ir_rmt_channel), TAG,
                                "failed to delete RMT channel");
        }
        s_ctx.ir_rmt_channel = NULL;
    }

    taskENTER_CRITICAL(&s_ctx_lock);
    tvbgone_core_get_default_config(&s_ctx.config);
    s_ctx.initialized = false;
    s_ctx.stop_requested = false;
    s_ctx.active_mode = TVBGONE_CORE_SEND_MODE_SINGLE;
    s_ctx.current_code_number = 0;
    s_ctx.total_codes = 0;
    taskEXIT_CRITICAL(&s_ctx_lock);

    return ESP_OK;
}
