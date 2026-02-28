/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * ST25DV NFC/I2C EEPROM Example — EEPROM read/write, NDEF, and field detection
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "st25dv.h"

static const char *TAG = "ST25DV-Example";

/* I2C configuration — adjust for your hardware */
#define I2C_MASTER_SCL_IO       CONFIG_ST25DV_I2C_SCL
#define I2C_MASTER_SDA_IO       CONFIG_ST25DV_I2C_SDA
#define I2C_MASTER_NUM          0
#define I2C_MASTER_FREQ_HZ      CONFIG_ST25DV_I2C_CLK_SPEED_HZ
#define GPO_GPIO                CONFIG_ST25DV_GPO_PIN

/* Demo URI written to NDEF (without the "https://" scheme) */
#define DEMO_URI                "espressif.com/components"

/* Global handle accessible from ISR context */
static st25dv_handle_t g_st25dv = NULL;
static volatile bool   g_gpo_triggered = false;

static void IRAM_ATTR gpo_isr_handler(void *arg)
{
    g_gpo_triggered = true;
}

static void print_uid(const uint8_t uid[8])
{
    printf("  UID: ");
    for (int i = 7; i >= 0; i--) {   /* ISO 15693 UID is transmitted LSB first; display MSB first */
        printf("%02X", uid[i]);
        if (i > 0) printf(":");
    }
    printf("\n");
}

void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "ST25DV NFC/I2C EEPROM Example");
    ESP_LOGI(TAG, "I2C SDA: GPIO%d, SCL: GPIO%d, Speed: %d Hz",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);

    /* ------------------------------------------------------------------ */
    /* 1. Initialize I2C master bus                                        */
    /* ------------------------------------------------------------------ */
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source             = I2C_CLK_SRC_DEFAULT,
        .i2c_port               = I2C_MASTER_NUM,
        .scl_io_num             = I2C_MASTER_SCL_IO,
        .sda_io_num             = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt      = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ret = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return;
    }

    /* ------------------------------------------------------------------ */
    /* 2. Create ST25DV device (registers both 0x53 and 0x57)             */
    /* ------------------------------------------------------------------ */
    st25dv_handle_t st25dv = st25dv_device_create(bus_handle, I2C_MASTER_FREQ_HZ);
    if (st25dv == NULL) {
        ESP_LOGE(TAG, "Failed to create ST25DV device — check wiring and I2C address");
        i2c_del_master_bus(bus_handle);
        return;
    }
    g_st25dv = st25dv;

    /* ------------------------------------------------------------------ */
    /* 3. Read chip info                                                   */
    /* ------------------------------------------------------------------ */
    uint8_t uid[8];
    if (st25dv_get_uid(st25dv, uid) == ESP_OK) {
        print_uid(uid);
    }

    uint8_t ic_ref;
    if (st25dv_get_ic_ref(st25dv, &ic_ref) == ESP_OK) {
        ESP_LOGI(TAG, "IC Reference: 0x%02X", ic_ref);
    }

    uint32_t mem_bytes;
    if (st25dv_get_mem_size(st25dv, &mem_bytes) == ESP_OK) {
        ESP_LOGI(TAG, "User memory: %lu bytes", (unsigned long)mem_bytes);
    }

    /* ------------------------------------------------------------------ */
    /* 4. Initialize GPO interrupt pin (optional)                         */
    /* ------------------------------------------------------------------ */
    if (GPO_GPIO >= 0) {
        /* Enable field-rising and field-falling GPO interrupt sources */
        ret = st25dv_gpo_configure(st25dv,
                                   ST25DV_GPO_FIELDRISING | ST25DV_GPO_FIELDFALLING);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "GPO configure failed (config area may be locked): %s",
                     esp_err_to_name(ret));
        }

        ret = st25dv_init_gpo_gpio((gpio_num_t)GPO_GPIO, gpo_isr_handler, NULL);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "GPO GPIO init failed, will use I2C polling");
        } else {
            ESP_LOGI(TAG, "GPO interrupt initialized on GPIO%d", GPO_GPIO);
        }
    }

    /* ------------------------------------------------------------------ */
    /* 5. Write NDEF Capability Container + URI record                    */
    /* ------------------------------------------------------------------ */
    ESP_LOGI(TAG, "Writing NDEF CC...");
    ret = st25dv_ndef_write_cc(st25dv);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to write CC: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Writing NDEF URI: https://" DEMO_URI);
        ret = st25dv_ndef_write_uri(st25dv, DEMO_URI);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to write NDEF URI: %s", esp_err_to_name(ret));
        }
    }

    /* ------------------------------------------------------------------ */
    /* 6. Read back NDEF payload and display                              */
    /* ------------------------------------------------------------------ */
    uint8_t ndef_buf[128];
    size_t  ndef_len = 0;
    ret = st25dv_ndef_read_raw(st25dv, ndef_buf, sizeof(ndef_buf), &ndef_len);
    if (ret == ESP_OK && ndef_len > 0) {
        ESP_LOGI(TAG, "NDEF payload (%zu bytes):", ndef_len);
        printf("  ");
        for (size_t j = 0; j < ndef_len; j++) {
            printf("%02X ", ndef_buf[j]);
            if ((j + 1) % 16 == 0) printf("\n  ");
        }
        printf("\n");
    } else if (ret != ESP_OK) {
        ESP_LOGW(TAG, "NDEF read failed: %s", esp_err_to_name(ret));
    }

    /* ------------------------------------------------------------------ */
    /* 7. Main loop: poll field + energy harvesting status                */
    /* ------------------------------------------------------------------ */
    ESP_LOGI(TAG, "Entering monitoring loop — bring an NFC reader near the tag...");

    while (1) {
        /* Handle GPO interrupt if triggered */
        if (g_gpo_triggered) {
            g_gpo_triggered = false;
            uint8_t it_sts = 0;
            if (st25dv_read_it_status(st25dv, &it_sts) == ESP_OK) {
                ESP_LOGI(TAG, "GPO event: IT_STS=0x%02X%s%s%s",
                         it_sts,
                         (it_sts & ST25DV_GPO_FIELDRISING)  ? " [FIELD_RISING]"  : "",
                         (it_sts & ST25DV_GPO_FIELDFALLING) ? " [FIELD_FALLING]" : "",
                         (it_sts & ST25DV_GPO_RFWRITE)      ? " [RF_WRITE]"       : "");
            }
        }

        /* I2C field polling */
        bool field_present = false;
        if (st25dv_is_field_present(st25dv, &field_present) == ESP_OK) {
            ESP_LOGI(TAG, "RF field: %s", field_present ? "PRESENT" : "absent");
        }

        /* Energy harvesting status */
        st25dv_eh_status_t eh_status;
        if (st25dv_eh_get_status(st25dv, &eh_status) == ESP_OK) {
            ESP_LOGI(TAG, "EH: eh_on=%d  field_on=%d  vcc_on=%d",
                     eh_status.eh_on, eh_status.field_on, eh_status.vcc_on);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    /* Unreachable in this example; shown for completeness */
    st25dv_device_delete(st25dv);
    i2c_del_master_bus(bus_handle);
}
