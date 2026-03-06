/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "st25dv.h"

static const char *TAG = "ST25DV";

/* =========================================================================
 * Private device struct
 * ========================================================================= */

struct st25dv_dev_t {
    i2c_master_bus_handle_t  bus_handle;   /*!< Master bus handle for ACK polling */
    i2c_master_dev_handle_t  data_dev;     /*!< Device at ST25DV_ADDR_DATA (0x53) */
    i2c_master_dev_handle_t  syst_dev;     /*!< Device at ST25DV_ADDR_SYST (0x57) */
    uint32_t                 mem_size_bytes; /*!< Total user EEPROM bytes */
#if __has_include("i2c_bus.h")
    i2c_bus_device_handle_t  bus_data_dev; /*!< i2c_bus device at 0x53 */
    i2c_bus_device_handle_t  bus_syst_dev; /*!< i2c_bus device at 0x57 */
    bool                     use_bus;
#endif
};

/* =========================================================================
 * Private helpers — 16-bit register I/O
 * ========================================================================= */

/**
 * @brief Read `len` bytes from a 16-bit register address over I2C.
 *
 * @param dev      Device struct pointer
 * @param use_syst true  → use system config address (0x57)
 *                 false → use data address (0x53)
 * @param reg_addr 16-bit register address (MSB sent first)
 * @param data     Output buffer
 * @param len      Number of bytes to read
 */
static esp_err_t st25dv_read_reg(struct st25dv_dev_t *dev,
                                  bool use_syst,
                                  uint16_t reg_addr,
                                  uint8_t *data,
                                  size_t len)
{
    uint8_t addr_buf[2] = { (uint8_t)(reg_addr >> 8), (uint8_t)(reg_addr & 0xFF) };
    esp_err_t ret;

#if __has_include("i2c_bus.h")
    if (dev->use_bus) {
        i2c_bus_device_handle_t bdev = use_syst ? dev->bus_syst_dev : dev->bus_data_dev;
        /* Send 2-byte register address as plain data, then read without an address prefix */
        ret = i2c_bus_write_bytes(bdev, NULL_I2C_MEM_ADDR, sizeof(addr_buf), addr_buf);
        ESP_RETURN_ON_ERROR(ret, TAG, "i2c_bus write reg addr failed (syst=%d, reg=0x%04X)", use_syst, reg_addr);
        ret = i2c_bus_read_bytes(bdev, NULL_I2C_MEM_ADDR, len, data);
        ESP_RETURN_ON_ERROR(ret, TAG, "i2c_bus read failed (syst=%d, reg=0x%04X)", use_syst, reg_addr);
        return ESP_OK;
    }
#endif

    i2c_master_dev_handle_t dev_hdl = use_syst ? dev->syst_dev : dev->data_dev;

    ret = i2c_master_transmit(dev_hdl, addr_buf, sizeof(addr_buf), -1);
    ESP_RETURN_ON_ERROR(ret, TAG, "I2C transmit reg addr failed (syst=%d, reg=0x%04X)", use_syst, reg_addr);

    ret = i2c_master_receive(dev_hdl, data, len, -1);
    ESP_RETURN_ON_ERROR(ret, TAG, "I2C receive failed (syst=%d, reg=0x%04X)", use_syst, reg_addr);

    return ESP_OK;
}

/**
 * @brief Write `len` bytes to a 16-bit register address over I2C.
 *
 * Constructs a local write buffer: [reg_MSB, reg_LSB, data...].
 * For the native path uses i2c_master_transmit with the combined buffer.
 * For the i2c_bus path uses i2c_bus_write_bytes with NULL_I2C_MEM_ADDR and
 * the same combined buffer.
 *
 * @param dev      Device struct pointer
 * @param use_syst true  → system config address (0x57)
 *                 false → data address (0x53)
 * @param reg_addr 16-bit register address (MSB sent first)
 * @param data     Data to write (may be NULL if len == 0, e.g., address-only write)
 * @param len      Number of data bytes to write (not including the 2-byte address)
 */
static esp_err_t st25dv_write_reg(struct st25dv_dev_t *dev,
                                   bool use_syst,
                                   uint16_t reg_addr,
                                   const uint8_t *data,
                                   size_t len)
{
    /* Allocate combined buffer on the stack for small writes, heap for large */
    size_t total = 2 + len;
    uint8_t stack_buf[68];   /* covers 2-byte addr + up to 64 bytes (safe stack bound) */
    uint8_t *buf = (total <= sizeof(stack_buf)) ? stack_buf : (uint8_t *)malloc(total);
    if (!buf) {
        return ESP_ERR_NO_MEM;
    }

    buf[0] = (uint8_t)(reg_addr >> 8);
    buf[1] = (uint8_t)(reg_addr & 0xFF);
    if (len > 0 && data) {
        memcpy(&buf[2], data, len);
    }

    esp_err_t ret;
#if __has_include("i2c_bus.h")
    if (dev->use_bus) {
        i2c_bus_device_handle_t bdev = use_syst ? dev->bus_syst_dev : dev->bus_data_dev;
        ret = i2c_bus_write_bytes(bdev, NULL_I2C_MEM_ADDR, total, buf);
    } else {
#endif
        i2c_master_dev_handle_t dev_hdl = use_syst ? dev->syst_dev : dev->data_dev;
        ret = i2c_master_transmit(dev_hdl, buf, total, -1);
#if __has_include("i2c_bus.h")
    }
#endif

    if (buf != stack_buf) {
        free(buf);
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed (syst=%d, reg=0x%04X): %s", use_syst, reg_addr, esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Poll the data I2C address until the device ACKs (EEPROM write complete).
 *
 * After an EEPROM write the ST25DV NACKs all I2C access until the write cycle
 * finishes (~5 ms typical). This function polls using i2c_master_probe.
 * Not required for the i2c_bus path (caller responsibility).
 *
 * @param dev        Device struct pointer
 * @param timeout_ms Maximum wait in milliseconds
 */
static esp_err_t st25dv_wait_ready(struct st25dv_dev_t *dev, uint32_t timeout_ms)
{
    uint32_t elapsed_ms = 0;
    while (elapsed_ms < timeout_ms) {
        if (i2c_master_probe(dev->bus_handle, ST25DV_ADDR_DATA, 20) == ESP_OK) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
        elapsed_ms++;
    }
    ESP_LOGE(TAG, "Device not ready after %lu ms", (unsigned long)timeout_ms);
    return ESP_ERR_TIMEOUT;
}

/* =========================================================================
 * Device Lifecycle
 * ========================================================================= */

st25dv_handle_t st25dv_device_create(i2c_master_bus_handle_t bus_handle, const uint32_t dev_speed)
{
    /* Probe both addresses before allocating resources */
    esp_err_t ret = i2c_master_probe(bus_handle, ST25DV_ADDR_DATA, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ST25DV data device not found at 0x%02X: %s",
                 ST25DV_ADDR_DATA, esp_err_to_name(ret));
        return NULL;
    }
    ret = i2c_master_probe(bus_handle, ST25DV_ADDR_SYST, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ST25DV system device not found at 0x%02X: %s",
                 ST25DV_ADDR_SYST, esp_err_to_name(ret));
        return NULL;
    }

    struct st25dv_dev_t *dev = calloc(1, sizeof(struct st25dv_dev_t));
    if (!dev) {
        ESP_LOGE(TAG, "Failed to allocate ST25DV device");
        return NULL;
    }
    dev->bus_handle = bus_handle;

    const i2c_device_config_t data_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = ST25DV_ADDR_DATA,
        .scl_speed_hz    = dev_speed,
    };
    ret = i2c_master_bus_add_device(bus_handle, &data_cfg, &dev->data_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add data device: %s", esp_err_to_name(ret));
        free(dev);
        return NULL;
    }

    const i2c_device_config_t syst_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = ST25DV_ADDR_SYST,
        .scl_speed_hz    = dev_speed,
    };
    ret = i2c_master_bus_add_device(bus_handle, &syst_cfg, &dev->syst_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add system device: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(dev->data_dev);
        free(dev);
        return NULL;
    }

    /* Read memory size during init */
    uint32_t mem_bytes = 0;
    if (st25dv_get_mem_size((st25dv_handle_t)dev, &mem_bytes) == ESP_OK) {
        dev->mem_size_bytes = mem_bytes;
    } else {
        ESP_LOGW(TAG, "Could not read memory size; defaulting to ST25DV04K (512 B)");
        dev->mem_size_bytes = 512;
    }

    ESP_LOGI(TAG, "ST25DV initialized: data=0x%02X syst=0x%02X mem=%lu B",
             ST25DV_ADDR_DATA, ST25DV_ADDR_SYST, (unsigned long)dev->mem_size_bytes);
    return (st25dv_handle_t)dev;
}

#if __has_include("i2c_bus.h")
st25dv_handle_t st25dv_device_create_i2cbus(i2c_bus_device_handle_t data_bus_dev,
                                              i2c_bus_device_handle_t syst_bus_dev)
{
    if (!data_bus_dev || !syst_bus_dev) {
        ESP_LOGE(TAG, "Invalid i2c_bus device handle(s)");
        return NULL;
    }

    struct st25dv_dev_t *dev = calloc(1, sizeof(struct st25dv_dev_t));
    if (!dev) {
        ESP_LOGE(TAG, "Failed to allocate ST25DV device");
        return NULL;
    }

    dev->bus_data_dev = data_bus_dev;
    dev->bus_syst_dev = syst_bus_dev;
    dev->use_bus = true;

    /* Read memory size during init */
    uint32_t mem_bytes = 0;
    if (st25dv_get_mem_size((st25dv_handle_t)dev, &mem_bytes) == ESP_OK) {
        dev->mem_size_bytes = mem_bytes;
    } else {
        ESP_LOGW(TAG, "Could not read memory size; defaulting to ST25DV04K (512 B)");
        dev->mem_size_bytes = 512;
    }

    ESP_LOGI(TAG, "ST25DV initialized via i2c_bus, mem=%lu B", (unsigned long)dev->mem_size_bytes);
    return (st25dv_handle_t)dev;
}
#endif

esp_err_t st25dv_device_delete(st25dv_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "NULL handle");
    struct st25dv_dev_t *dev = (struct st25dv_dev_t *)handle;
    esp_err_t ret = ESP_OK;

#if __has_include("i2c_bus.h")
    if (dev->use_bus) {
        if (dev->bus_data_dev) {
            esp_err_t r = i2c_bus_device_delete(&dev->bus_data_dev);
            if (r != ESP_OK) ret = r;
        }
        if (dev->bus_syst_dev) {
            esp_err_t r = i2c_bus_device_delete(&dev->bus_syst_dev);
            if (r != ESP_OK) ret = r;
        }
        free(dev);
        return ret;
    }
#endif

    if (dev->data_dev) {
        esp_err_t r = i2c_master_bus_rm_device(dev->data_dev);
        if (r != ESP_OK) ret = r;
    }
    if (dev->syst_dev) {
        esp_err_t r = i2c_master_bus_rm_device(dev->syst_dev);
        if (r != ESP_OK) ret = r;
    }
    free(dev);
    return ret;
}

/* =========================================================================
 * Chip Information
 * ========================================================================= */

esp_err_t st25dv_get_uid(st25dv_handle_t handle, uint8_t uid_buf[8])
{
    ESP_RETURN_ON_FALSE(handle && uid_buf, ESP_ERR_INVALID_ARG, TAG, "NULL argument");
    return st25dv_read_reg((struct st25dv_dev_t *)handle, true, ST25DV_UID_REG, uid_buf, 8);
}

esp_err_t st25dv_get_ic_ref(st25dv_handle_t handle, uint8_t *ic_ref)
{
    ESP_RETURN_ON_FALSE(handle && ic_ref, ESP_ERR_INVALID_ARG, TAG, "NULL argument");
    return st25dv_read_reg((struct st25dv_dev_t *)handle, true, ST25DV_IC_REF_REG, ic_ref, 1);
}

esp_err_t st25dv_get_mem_size(st25dv_handle_t handle, uint32_t *mem_bytes)
{
    ESP_RETURN_ON_FALSE(handle && mem_bytes, ESP_ERR_INVALID_ARG, TAG, "NULL argument");
    struct st25dv_dev_t *dev = (struct st25dv_dev_t *)handle;

    /* MEM_SIZE_REG holds 2 bytes (Mem_Size[15:0]), BLK_SIZE_REG holds 1 byte */
    uint8_t raw[3];
    esp_err_t ret = st25dv_read_reg(dev, true, ST25DV_MEM_SIZE_REG, raw, 2);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read MEM_SIZE_REG");

    uint8_t blk_size_byte;
    ret = st25dv_read_reg(dev, true, ST25DV_BLK_SIZE_REG, &blk_size_byte, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read BLK_SIZE_REG");

    uint16_t mem_size_field = ((uint16_t)raw[0] << 8) | raw[1];
    *mem_bytes = ((uint32_t)mem_size_field + 1) * ((uint32_t)blk_size_byte + 1);
    return ESP_OK;
}

/* =========================================================================
 * EEPROM Read / Write
 * ========================================================================= */

esp_err_t st25dv_read_bytes(st25dv_handle_t handle, uint16_t reg_addr, uint8_t *data, size_t len)
{
    ESP_RETURN_ON_FALSE(handle && data && len, ESP_ERR_INVALID_ARG, TAG, "Invalid argument");
    return st25dv_read_reg((struct st25dv_dev_t *)handle, false, reg_addr, data, len);
}

esp_err_t st25dv_write_bytes(st25dv_handle_t handle, uint16_t reg_addr, const uint8_t *data, size_t len)
{
    ESP_RETURN_ON_FALSE(handle && data && len, ESP_ERR_INVALID_ARG, TAG, "Invalid argument");
    struct st25dv_dev_t *dev = (struct st25dv_dev_t *)handle;

    esp_err_t ret = st25dv_write_reg(dev, false, reg_addr, data, len);
    ESP_RETURN_ON_ERROR(ret, TAG, "Write failed at reg 0x%04X", reg_addr);

#if __has_include("i2c_bus.h")
    if (dev->use_bus) {
        /* On i2c_bus path there is no bus handle for probing; use a fixed delay */
        vTaskDelay(pdMS_TO_TICKS(ST25DV_WRITE_TIMEOUT_MS));
        return ESP_OK;
    }
#endif

    return st25dv_wait_ready(dev, ST25DV_WRITE_TIMEOUT_MS * 10);
}

/* =========================================================================
 * Field Detection & GPO
 * ========================================================================= */

esp_err_t st25dv_is_field_present(st25dv_handle_t handle, bool *field_present)
{
    ESP_RETURN_ON_FALSE(handle && field_present, ESP_ERR_INVALID_ARG, TAG, "NULL argument");

    uint8_t eh_ctrl;
    esp_err_t ret = st25dv_read_reg((struct st25dv_dev_t *)handle, false,
                                     ST25DV_EH_CTRL_DYN_REG, &eh_ctrl, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read EH_CTRL_DYN_REG");

    *field_present = (eh_ctrl & ST25DV_EH_CTRL_DYN_FIELD) != 0;
    return ESP_OK;
}

esp_err_t st25dv_read_it_status(st25dv_handle_t handle, uint8_t *it_status)
{
    ESP_RETURN_ON_FALSE(handle && it_status, ESP_ERR_INVALID_ARG, TAG, "NULL argument");
    return st25dv_read_reg((struct st25dv_dev_t *)handle, false,
                            ST25DV_IT_STS_DYN_REG, it_status, 1);
}

esp_err_t st25dv_gpo_configure(st25dv_handle_t handle, uint8_t gpo_config)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "NULL handle");
    return st25dv_write_reg((struct st25dv_dev_t *)handle, true,
                             ST25DV_GPO1_REG, &gpo_config, 1);
}

esp_err_t st25dv_init_gpo_gpio(gpio_num_t gpo_gpio, st25dv_gpo_callback_t callback, void *callback_arg)
{
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(gpo_gpio), ESP_ERR_INVALID_ARG, TAG, "Invalid GPO GPIO");

    /* GPO is open-drain active LOW — configure as input with pull-up */
    gpio_config_t io_conf = {
        .pin_bit_mask  = (1ULL << gpo_gpio),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_ENABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_ANYEDGE,
    };

    esp_err_t ret = gpio_config(&io_conf);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to configure GPO GPIO");

    if (callback != NULL) {
        ret = gpio_install_isr_service(0);
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_RETURN_ON_ERROR(ret, TAG, "Failed to install GPIO ISR service");
        }
        ret = gpio_isr_handler_add(gpo_gpio, (gpio_isr_t)callback, callback_arg);
        ESP_RETURN_ON_ERROR(ret, TAG, "Failed to add GPO ISR handler");
        ESP_LOGI(TAG, "GPO GPIO%d initialized with interrupt", gpo_gpio);
    } else {
        ESP_LOGI(TAG, "GPO GPIO%d initialized (polling mode)", gpo_gpio);
    }

    return ESP_OK;
}

esp_err_t st25dv_read_gpo_pin(gpio_num_t gpo_gpio, bool *gpo_asserted)
{
    ESP_RETURN_ON_FALSE(gpo_asserted, ESP_ERR_INVALID_ARG, TAG, "NULL argument");
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(gpo_gpio), ESP_ERR_INVALID_ARG, TAG, "Invalid GPIO");

    /* GPO is active LOW — pin LOW means event is asserted */
    *gpo_asserted = (gpio_get_level(gpo_gpio) == 0);
    return ESP_OK;
}

/* =========================================================================
 * Energy Harvesting
 * ========================================================================= */

esp_err_t st25dv_eh_set_mode(st25dv_handle_t handle, st25dv_eh_mode_t mode)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "NULL handle");
    uint8_t val = (mode == ST25DV_EH_MODE_AFTER_BOOT) ? 0x01U : 0x00U;
    return st25dv_write_reg((struct st25dv_dev_t *)handle, true, ST25DV_EH_MODE_REG, &val, 1);
}

esp_err_t st25dv_eh_enable(st25dv_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "NULL handle");
    struct st25dv_dev_t *dev = (struct st25dv_dev_t *)handle;

    uint8_t reg;
    esp_err_t ret = st25dv_read_reg(dev, false, ST25DV_EH_CTRL_DYN_REG, &reg, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read EH_CTRL_DYN_REG");

    reg |= ST25DV_EH_CTRL_DYN_EH_EN;
    return st25dv_write_reg(dev, false, ST25DV_EH_CTRL_DYN_REG, &reg, 1);
}

esp_err_t st25dv_eh_disable(st25dv_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "NULL handle");
    struct st25dv_dev_t *dev = (struct st25dv_dev_t *)handle;

    uint8_t reg;
    esp_err_t ret = st25dv_read_reg(dev, false, ST25DV_EH_CTRL_DYN_REG, &reg, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read EH_CTRL_DYN_REG");

    reg &= ~ST25DV_EH_CTRL_DYN_EH_EN;
    return st25dv_write_reg(dev, false, ST25DV_EH_CTRL_DYN_REG, &reg, 1);
}

esp_err_t st25dv_eh_get_status(st25dv_handle_t handle, st25dv_eh_status_t *status)
{
    ESP_RETURN_ON_FALSE(handle && status, ESP_ERR_INVALID_ARG, TAG, "NULL argument");

    uint8_t reg;
    esp_err_t ret = st25dv_read_reg((struct st25dv_dev_t *)handle, false,
                                     ST25DV_EH_CTRL_DYN_REG, &reg, 1);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read EH_CTRL_DYN_REG");

    status->eh_enabled = (reg & ST25DV_EH_CTRL_DYN_EH_EN) != 0;
    status->eh_on      = (reg & ST25DV_EH_CTRL_DYN_EH_ON) != 0;
    status->field_on   = (reg & ST25DV_EH_CTRL_DYN_FIELD) != 0;
    status->vcc_on     = (reg & ST25DV_EH_CTRL_DYN_VCC)   != 0;
    return ESP_OK;
}

/* =========================================================================
 * NDEF
 * ========================================================================= */

esp_err_t st25dv_ndef_write_cc(st25dv_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "NULL handle");
    struct st25dv_dev_t *dev = (struct st25dv_dev_t *)handle;

    /*
     * CC file format (4 bytes):
     *   [0] 0xE1  — Magic number (NDEF Type 5 Tag)
     *   [1] 0x40  — Version 1.0, read/write access
     *   [2] mlen  — (mem_size - 4) / 8, capped at 0xFF
     *   [3] 0x00  — Additional feature flags (none)
     */
    uint32_t mem = dev->mem_size_bytes > 0 ? dev->mem_size_bytes : 512;
    uint8_t mlen = (uint8_t)(((mem - 4) / 8) & 0xFF);

    uint8_t cc[4] = { ST25DV_NDEF_CC_MAGIC, ST25DV_NDEF_CC_VERSION, mlen, 0x00 };
    return st25dv_write_bytes(handle, ST25DV_NDEF_CC_ADDR, cc, sizeof(cc));
}

esp_err_t st25dv_ndef_write_uri(st25dv_handle_t handle, const char *uri)
{
    ESP_RETURN_ON_FALSE(handle && uri, ESP_ERR_INVALID_ARG, TAG, "NULL argument");

    size_t uri_len   = strlen(uri);
    /* Payload = identifier byte (0x04 = "https://") + URI string */
    size_t payload   = 1 + uri_len;
    /* NDEF record: header(1) + type_len(1) + payload_len(1) + type(1) + payload */
    size_t rec_len   = 1 + 1 + 1 + 1 + payload;
    /* TLV: type(1) + length(1) + record + terminator(1) */
    size_t total     = 1 + 1 + rec_len + 1;

    if (total > 256) {
        ESP_LOGE(TAG, "URI too long for single NDEF write buffer (%zu bytes)", total);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t buf[256];
    size_t i = 0;

    /* TLV header */
    buf[i++] = ST25DV_NDEF_TLV_NDEF_TYPE;
    buf[i++] = (uint8_t)rec_len;

    /* NDEF record header: MB=1, ME=1, CF=0, SR=1, IL=0, TNF=001 (Well Known) */
    buf[i++] = 0xD1;
    buf[i++] = 0x01;                  /* type length = 1 ('U') */
    buf[i++] = (uint8_t)payload;      /* payload length */
    buf[i++] = 'U';                   /* record type */
    buf[i++] = 0x04;                  /* URI identifier: "https://" */
    memcpy(&buf[i], uri, uri_len);
    i += uri_len;

    /* TLV terminator */
    buf[i++] = ST25DV_NDEF_TLV_TERMINATOR;

    return st25dv_write_bytes(handle, ST25DV_NDEF_MSG_ADDR, buf, i);
}

esp_err_t st25dv_ndef_write_text(st25dv_handle_t handle, const char *text, const char *lang)
{
    ESP_RETURN_ON_FALSE(handle && text && lang, ESP_ERR_INVALID_ARG, TAG, "NULL argument");

    size_t text_len  = strlen(text);
    size_t lang_len  = strlen(lang);
    if (lang_len > 63) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Text record payload:
     *   status byte: 0x00–0x3F where bits[5:0] = lang code length, bit6=0 (UTF-8), bit7=0
     *   lang code bytes
     *   text bytes
     */
    size_t payload  = 1 + lang_len + text_len;
    size_t rec_len  = 1 + 1 + 1 + 1 + payload;  /* header + type_len + payload_len + type + payload */
    size_t total    = 1 + 1 + rec_len + 1;        /* TLV type + len + record + terminator */

    if (total > 256) {
        ESP_LOGE(TAG, "Text too long for single NDEF write buffer (%zu bytes)", total);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t buf[256];
    size_t i = 0;

    buf[i++] = ST25DV_NDEF_TLV_NDEF_TYPE;
    buf[i++] = (uint8_t)rec_len;

    /* NDEF record: MB=1, ME=1, CF=0, SR=1, IL=0, TNF=001 */
    buf[i++] = 0xD1;
    buf[i++] = 0x01;                           /* type length = 1 ('T') */
    buf[i++] = (uint8_t)payload;               /* payload length */
    buf[i++] = 'T';                            /* record type */
    buf[i++] = (uint8_t)(lang_len & 0x3F);     /* status: UTF-8, lang code length */
    memcpy(&buf[i], lang, lang_len);
    i += lang_len;
    memcpy(&buf[i], text, text_len);
    i += text_len;

    buf[i++] = ST25DV_NDEF_TLV_TERMINATOR;

    return st25dv_write_bytes(handle, ST25DV_NDEF_MSG_ADDR, buf, i);
}

esp_err_t st25dv_ndef_read_raw(st25dv_handle_t handle, uint8_t *buf, size_t buf_len, size_t *out_len)
{
    ESP_RETURN_ON_FALSE(handle && buf && buf_len && out_len, ESP_ERR_INVALID_ARG, TAG, "NULL argument");

    /* Read CC (4 bytes) first to validate */
    uint8_t cc[4];
    esp_err_t ret = st25dv_read_bytes(handle, ST25DV_NDEF_CC_ADDR, cc, sizeof(cc));
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read CC");

    if (cc[0] != ST25DV_NDEF_CC_MAGIC) {
        ESP_LOGW(TAG, "CC magic mismatch: expected 0xE1 got 0x%02X (NDEF not initialized?)", cc[0]);
        return ESP_ERR_INVALID_STATE;
    }

    /* Read TLV header from NDEF message area */
    uint8_t tlv_hdr[2];
    ret = st25dv_read_bytes(handle, ST25DV_NDEF_MSG_ADDR, tlv_hdr, sizeof(tlv_hdr));
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read TLV header");

    if (tlv_hdr[0] != ST25DV_NDEF_TLV_NDEF_TYPE) {
        ESP_LOGW(TAG, "TLV type byte is 0x%02X, expected 0x03 (NDEF TLV)", tlv_hdr[0]);
        return ESP_ERR_INVALID_STATE;
    }

    size_t ndef_len = tlv_hdr[1];
    if (ndef_len == 0) {
        *out_len = 0;
        return ESP_OK;
    }
    if (ndef_len > buf_len) {
        ESP_LOGE(TAG, "NDEF payload (%zu bytes) exceeds buffer (%zu bytes)", ndef_len, buf_len);
        return ESP_ERR_NO_MEM;
    }

    ret = st25dv_read_bytes(handle, ST25DV_NDEF_MSG_ADDR + 2, buf, ndef_len);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read NDEF payload");

    *out_len = ndef_len;
    return ESP_OK;
}
