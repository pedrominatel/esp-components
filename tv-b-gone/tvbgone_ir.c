#include "tvbgone_ir.h"

#include <stddef.h>
#include <stdlib.h>

#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

typedef struct IrCode IrCode;
#include "WORLD_IR_CODES.h"

#define TAG "tvbgone_ir"

#define IR_TX_WAIT_TIMEOUT_MIN_MS 1500
#define IR_TX_WAIT_TIMEOUT_MAX_MS 8000
#define XIAOMI_POWER_FREQ_HZ      38000

#define NUM_ELEM(x) (sizeof(x) / sizeof((x)[0]))

typedef struct {
    SemaphoreHandle_t lock;
    rmt_channel_handle_t tx_channel;
    rmt_encoder_handle_t copy_encoder;
    TaskHandle_t task_handle;
    tvbgone_ir_config_t config;
    volatile bool initialized;
    volatile bool running;
    volatile bool stop_requested;
    tvbgone_ir_mode_t mode;
} tvbgone_ir_ctx_t;

static tvbgone_ir_ctx_t s_ctx;

static inline bool should_stop(void)
{
    return s_ctx.stop_requested;
}

static esp_err_t lock_ctx(TickType_t timeout_ticks)
{
    if (s_ctx.lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return xSemaphoreTake(s_ctx.lock, timeout_ticks) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

static void unlock_ctx(void)
{
    if (s_ctx.lock) {
        xSemaphoreGive(s_ctx.lock);
    }
}

static uint8_t read_bits(uint8_t count, const struct IrCode *code, uint8_t *bits_left, uint8_t *bits_reg, uint8_t *code_ptr)
{
    uint8_t tmp = 0;
    while (count--) {
        if (*bits_left == 0) {
            *bits_reg = code->codes[(*code_ptr)++];
            *bits_left = 8;
        }
        tmp = (tmp << 1) | ((*bits_reg >> --(*bits_left)) & 0x01);
    }
    return tmp;
}

static uint32_t build_rmt_symbols_from_code(const struct IrCode *code, rmt_symbol_word_t *symbols)
{
    uint32_t total_ticks = 0;
    uint8_t bits_left = 0;
    uint8_t bits_reg = 0;
    uint8_t code_ptr = 0;

    for (uint16_t i = 0; i < code->numpairs; i++) {
        uint16_t t_index = read_bits(code->bitcompression, code, &bits_left, &bits_reg, &code_ptr) * 2;
        uint32_t d0 = code->times[t_index];
        uint32_t d1 = code->times[t_index + 1];
        if (d0 == 0) {
            d0 = 1;
        }
        if (d1 == 0) {
            d1 = 1;
        }
        symbols[i].level0 = 1;
        symbols[i].duration0 = d0;
        symbols[i].level1 = 0;
        symbols[i].duration1 = d1;
        total_ticks += d0 + d1;
    }

    return total_ticks;
}

static void recover_channel_if_needed(void)
{
    esp_err_t dis_err = rmt_disable(s_ctx.tx_channel);
    if (dis_err != ESP_OK) {
        ESP_LOGW(TAG, "rmt_disable failed during recovery: %s", esp_err_to_name(dis_err));
    }
    esp_err_t en_err = rmt_enable(s_ctx.tx_channel);
    if (en_err != ESP_OK) {
        ESP_LOGW(TAG, "rmt_enable failed during recovery: %s", esp_err_to_name(en_err));
    }
}

static esp_err_t wait_tx_done(uint32_t total_ticks, const char *log_prefix)
{
    uint32_t expected_ms = total_ticks / (s_ctx.config.resolution_hz / 1000);
    uint32_t wait_timeout_ms = expected_ms + 500;
    if (wait_timeout_ms < IR_TX_WAIT_TIMEOUT_MIN_MS) {
        wait_timeout_ms = IR_TX_WAIT_TIMEOUT_MIN_MS;
    } else if (wait_timeout_ms > IR_TX_WAIT_TIMEOUT_MAX_MS) {
        wait_timeout_ms = IR_TX_WAIT_TIMEOUT_MAX_MS;
    }

    esp_err_t err = rmt_tx_wait_all_done(s_ctx.tx_channel, pdMS_TO_TICKS(wait_timeout_ms));
    if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "%s timeout (expected=%u ms, waited=%u ms)",
                 log_prefix, (unsigned)expected_ms, (unsigned)wait_timeout_ms);
        recover_channel_if_needed();
        return ESP_ERR_TIMEOUT;
    }
    return err;
}

static esp_err_t transmit_ir_code(const struct IrCode *code, size_t index, size_t total)
{
    rmt_carrier_config_t carrier_cfg = {
        .frequency_hz = code->timer_val * 1000,
        .duty_cycle = 0.33,
    };
    ESP_RETURN_ON_ERROR(rmt_apply_carrier(s_ctx.tx_channel, &carrier_cfg), TAG, "apply carrier failed");

    rmt_symbol_word_t *symbols = calloc(code->numpairs, sizeof(rmt_symbol_word_t));
    if (!symbols) {
        return ESP_ERR_NO_MEM;
    }

    uint32_t total_ticks = build_rmt_symbols_from_code(code, symbols);

    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
    };

    esp_err_t err = rmt_transmit(s_ctx.tx_channel, s_ctx.copy_encoder, symbols,
                                 sizeof(rmt_symbol_word_t) * code->numpairs, &tx_cfg);
    free(symbols);
    ESP_RETURN_ON_ERROR(err, TAG, "transmit failed");

    err = wait_tx_done(total_ticks, "TV-B-Gone code TX wait");
    if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "timeout on code %u/%u (freq=%u kHz)",
                 (unsigned)(index + 1), (unsigned)total, (unsigned)code->timer_val);
        return ESP_OK;
    }
    return err;
}

static esp_err_t send_xiaomi_power_code(void)
{
    const size_t pulse_count = NUM_ELEM(TVBGONE_IR_XIAOMI_POWER_RAW_US);
    const size_t symbol_count = pulse_count / 2;
    rmt_symbol_word_t *symbols = calloc(symbol_count, sizeof(rmt_symbol_word_t));
    if (!symbols) {
        return ESP_ERR_NO_MEM;
    }

    uint32_t total_ticks = 0;
    for (size_t i = 0; i < symbol_count; i++) {
        uint32_t d0 = (TVBGONE_IR_XIAOMI_POWER_RAW_US[i * 2] + 5) / 10;
        uint32_t d1 = (TVBGONE_IR_XIAOMI_POWER_RAW_US[(i * 2) + 1] + 5) / 10;
        if (d0 == 0) {
            d0 = 1;
        }
        if (d1 == 0) {
            d1 = 1;
        }
        symbols[i].level0 = 1;
        symbols[i].duration0 = d0;
        symbols[i].level1 = 0;
        symbols[i].duration1 = d1;
        total_ticks += d0 + d1;
    }

    rmt_carrier_config_t carrier_cfg = {
        .frequency_hz = XIAOMI_POWER_FREQ_HZ,
        .duty_cycle = 0.33,
    };

    esp_err_t err = rmt_apply_carrier(s_ctx.tx_channel, &carrier_cfg);
    if (err != ESP_OK) {
        free(symbols);
        return err;
    }

    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
    };

    err = rmt_transmit(s_ctx.tx_channel, s_ctx.copy_encoder, symbols,
                       symbol_count * sizeof(rmt_symbol_word_t), &tx_cfg);
    free(symbols);
    ESP_RETURN_ON_ERROR(err, TAG, "Xiaomi transmit failed");

    err = wait_tx_done(total_ticks, "Xiaomi power TX wait");
    return (err == ESP_ERR_TIMEOUT) ? ESP_OK : err;
}

static esp_err_t send_region_codes(const struct IrCode *const *codes, size_t num_codes, const char *region_name)
{
    ESP_LOGI(TAG, "Sending %s region set (%u codes)", region_name, (unsigned)num_codes);

    for (size_t i = 0; i < num_codes; i++) {
        if (should_stop()) {
            return ESP_OK;
        }
        if (!codes[i]) {
            continue;
        }
        if ((i % 10) == 0) {
            ESP_LOGI(TAG, "%s progress: %u/%u", region_name, (unsigned)(i + 1), (unsigned)num_codes);
        }

        esp_err_t err = transmit_ir_code(codes[i], i, num_codes);
        ESP_RETURN_ON_ERROR(err, TAG, "region transmit failed");
        vTaskDelay(pdMS_TO_TICKS(s_ctx.config.code_gap_ms));
    }

    return ESP_OK;
}

static esp_err_t create_tx_channel_with_fallback(void)
{
    const size_t mem_block_candidates[] = {256, 128, 96, 64, 48};

    for (size_t i = 0; i < NUM_ELEM(mem_block_candidates); i++) {
        rmt_tx_channel_config_t tx_chan_cfg = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = s_ctx.config.resolution_hz,
            .mem_block_symbols = mem_block_candidates[i],
            .trans_queue_depth = s_ctx.config.tx_queue_depth,
            .gpio_num = s_ctx.config.gpio_num,
        };

        esp_err_t err = rmt_new_tx_channel(&tx_chan_cfg, &s_ctx.tx_channel);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Allocated TX channel with mem_block_symbols=%u", (unsigned)mem_block_candidates[i]);
            return ESP_OK;
        }
        ESP_LOGW(TAG, "TX alloc failed with mem_block_symbols=%u: %s",
                 (unsigned)mem_block_candidates[i], esp_err_to_name(err));
    }

    return ESP_ERR_NOT_FOUND;
}

static esp_err_t send_one_sweep(void)
{
    const size_t num_na_codes = NUM_ELEM(NApowerCodes);
    const size_t num_eu_codes = NUM_ELEM(EUpowerCodes);

    tvbgone_ir_mode_t mode = s_ctx.mode;

    ESP_RETURN_ON_ERROR(send_xiaomi_power_code(), TAG, "Xiaomi send failed");
    vTaskDelay(pdMS_TO_TICKS(s_ctx.config.code_gap_ms));

    if (mode == TVBGONE_IR_MODE_NA || mode == TVBGONE_IR_MODE_BOTH) {
        ESP_RETURN_ON_ERROR(send_region_codes(NApowerCodes, num_na_codes, "NA"), TAG, "NA send failed");
    }
    if (mode == TVBGONE_IR_MODE_EU || mode == TVBGONE_IR_MODE_BOTH) {
        ESP_RETURN_ON_ERROR(send_region_codes(EUpowerCodes, num_eu_codes, "EU"), TAG, "EU send failed");
    }

    return ESP_OK;
}

static void tvbgone_task(void *arg)
{
    (void)arg;

    while (!should_stop()) {
        esp_err_t err = send_one_sweep();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "send sweep failed: %s", esp_err_to_name(err));
        }

        if (should_stop()) {
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(s_ctx.config.sweep_gap_ms));
    }

    if (lock_ctx(pdMS_TO_TICKS(100)) == ESP_OK) {
        s_ctx.running = false;
        s_ctx.stop_requested = false;
        s_ctx.task_handle = NULL;
        unlock_ctx();
    }

    vTaskDelete(NULL);
}

esp_err_t tvbgone_ir_init(const tvbgone_ir_config_t *config)
{
    if (s_ctx.lock == NULL) {
        s_ctx.lock = xSemaphoreCreateMutex();
        if (s_ctx.lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    ESP_RETURN_ON_ERROR(lock_ctx(pdMS_TO_TICKS(100)), TAG, "lock failed");

    if (s_ctx.initialized) {
        unlock_ctx();
        return ESP_ERR_INVALID_STATE;
    }

    s_ctx.config = config ? *config : (tvbgone_ir_config_t)TVBGONE_IR_DEFAULT_CONFIG();
    s_ctx.mode = TVBGONE_IR_MODE_BOTH;

    if (s_ctx.config.gpio_num < 0 || s_ctx.config.resolution_hz == 0 || s_ctx.config.tx_queue_depth == 0) {
        unlock_ctx();
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = create_tx_channel_with_fallback();
    if (err != ESP_OK) {
        unlock_ctx();
        return err;
    }

    rmt_copy_encoder_config_t copy_cfg = {};
    err = rmt_new_copy_encoder(&copy_cfg, &s_ctx.copy_encoder);
    if (err != ESP_OK) {
        rmt_del_channel(s_ctx.tx_channel);
        s_ctx.tx_channel = NULL;
        unlock_ctx();
        return err;
    }

    err = rmt_enable(s_ctx.tx_channel);
    if (err != ESP_OK) {
        rmt_del_encoder(s_ctx.copy_encoder);
        rmt_del_channel(s_ctx.tx_channel);
        s_ctx.copy_encoder = NULL;
        s_ctx.tx_channel = NULL;
        unlock_ctx();
        return err;
    }

    s_ctx.initialized = true;
    s_ctx.running = false;
    s_ctx.stop_requested = false;

    unlock_ctx();
    return ESP_OK;
}

esp_err_t tvbgone_ir_deinit(void)
{
    ESP_RETURN_ON_ERROR(lock_ctx(pdMS_TO_TICKS(100)), TAG, "lock failed");

    if (!s_ctx.initialized) {
        unlock_ctx();
        return ESP_ERR_INVALID_STATE;
    }
    if (s_ctx.running) {
        unlock_ctx();
        return ESP_ERR_INVALID_STATE;
    }

    if (s_ctx.tx_channel) {
        rmt_disable(s_ctx.tx_channel);
        rmt_del_channel(s_ctx.tx_channel);
    }
    if (s_ctx.copy_encoder) {
        rmt_del_encoder(s_ctx.copy_encoder);
    }

    s_ctx.tx_channel = NULL;
    s_ctx.copy_encoder = NULL;
    s_ctx.initialized = false;

    unlock_ctx();
    return ESP_OK;
}

esp_err_t tvbgone_ir_set_mode(tvbgone_ir_mode_t mode)
{
    if (mode != TVBGONE_IR_MODE_NA && mode != TVBGONE_IR_MODE_EU && mode != TVBGONE_IR_MODE_BOTH) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(lock_ctx(pdMS_TO_TICKS(100)), TAG, "lock failed");
    if (!s_ctx.initialized) {
        unlock_ctx();
        return ESP_ERR_INVALID_STATE;
    }

    s_ctx.mode = mode;
    unlock_ctx();
    return ESP_OK;
}

tvbgone_ir_mode_t tvbgone_ir_get_mode(void)
{
    return s_ctx.mode;
}

esp_err_t tvbgone_ir_send_once(void)
{
    ESP_RETURN_ON_ERROR(lock_ctx(pdMS_TO_TICKS(100)), TAG, "lock failed");
    if (!s_ctx.initialized || s_ctx.running) {
        unlock_ctx();
        return ESP_ERR_INVALID_STATE;
    }
    unlock_ctx();

    return send_one_sweep();
}

esp_err_t tvbgone_ir_start(void)
{
    ESP_RETURN_ON_ERROR(lock_ctx(pdMS_TO_TICKS(100)), TAG, "lock failed");

    if (!s_ctx.initialized) {
        unlock_ctx();
        return ESP_ERR_INVALID_STATE;
    }
    if (s_ctx.running) {
        unlock_ctx();
        return ESP_OK;
    }

    s_ctx.stop_requested = false;
    s_ctx.running = true;

    BaseType_t ok = xTaskCreate(tvbgone_task, "tvbgone_ir", 6144, NULL, 5, &s_ctx.task_handle);
    if (ok != pdPASS) {
        s_ctx.running = false;
        s_ctx.task_handle = NULL;
        unlock_ctx();
        return ESP_ERR_NO_MEM;
    }

    unlock_ctx();
    return ESP_OK;
}

esp_err_t tvbgone_ir_stop(TickType_t timeout_ticks)
{
    ESP_RETURN_ON_ERROR(lock_ctx(pdMS_TO_TICKS(100)), TAG, "lock failed");

    if (!s_ctx.initialized) {
        unlock_ctx();
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_ctx.running) {
        unlock_ctx();
        return ESP_OK;
    }

    s_ctx.stop_requested = true;
    unlock_ctx();

    TickType_t start = xTaskGetTickCount();
    while (tvbgone_ir_is_running()) {
        if (timeout_ticks != portMAX_DELAY) {
            TickType_t elapsed = xTaskGetTickCount() - start;
            if (elapsed >= timeout_ticks) {
                return ESP_ERR_TIMEOUT;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return ESP_OK;
}

bool tvbgone_ir_is_running(void)
{
    return s_ctx.running;
}
