#include <string.h>
#include "max17043.h"
#include "esp_log.h"

static const char *TAG = "MAX17043";

typedef struct {
    i2c_master_bus_handle_t i2c_bus;
    i2c_master_dev_handle_t i2c_dev;
    uint8_t i2c_addr;
} max17043_dev_t;

max17043_handle_t max17043_create(i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr)
{
    if (!bus_handle) {
        ESP_LOGE(TAG, "Invalid I2C bus handle");
        return NULL;
    }

    max17043_dev_t *dev = (max17043_dev_t *)malloc(sizeof(max17043_dev_t));
    if (!dev) {
        ESP_LOGE(TAG, "Failed to allocate memory for device handle");
        return NULL;
    }

    dev->i2c_bus = bus_handle;
    dev->i2c_addr = i2c_addr;

    /* Create I2C device */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = 400000,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        free(dev);
        return NULL;
    }

    ESP_LOGI(TAG, "MAX17043 device created successfully");
    return (max17043_handle_t)dev;
}

esp_err_t max17043_delete(max17043_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    max17043_dev_t *dev = (max17043_dev_t *)handle;

    esp_err_t ret = i2c_master_bus_rm_device(dev->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove I2C device: %s", esp_err_to_name(ret));
        return ret;
    }

    free(dev);
    return ESP_OK;
}

esp_err_t max17043_read_voltage(max17043_handle_t handle, uint16_t *voltage)
{
    if (!handle || !voltage) {
        return ESP_ERR_INVALID_ARG;
    }

    max17043_dev_t *dev = (max17043_dev_t *)handle;

    uint8_t reg = MAX17043_REG_VCELL;
    uint8_t data[2];

    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, data, 2, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read VCELL register: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Convert to mV: VCELL = (data[0] << 4) | (data[1] >> 4), in 1.25mV steps */
    *voltage = ((data[0] << 4) | (data[1] >> 4)) * 1.25;

    return ESP_OK;
}

esp_err_t max17043_read_soc(max17043_handle_t handle, uint8_t *soc)
{
    if (!handle || !soc) {
        return ESP_ERR_INVALID_ARG;
    }

    max17043_dev_t *dev = (max17043_dev_t *)handle;

    uint8_t reg = MAX17043_REG_SOC;
    uint8_t data[2];

    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, data, 2, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read SOC register: %s", esp_err_to_name(ret));
        return ret;
    }

    *soc = data[0];

    return ESP_OK;
}

esp_err_t max17043_read_version(max17043_handle_t handle, uint16_t *version)
{
    if (!handle || !version) {
        return ESP_ERR_INVALID_ARG;
    }

    max17043_dev_t *dev = (max17043_dev_t *)handle;

    uint8_t reg = MAX17043_REG_VERSION;
    uint8_t data[2];

    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, data, 2, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read VERSION register: %s", esp_err_to_name(ret));
        return ret;
    }

    *version = (data[0] << 8) | data[1];

    return ESP_OK;
}

esp_err_t max17043_set_alert_threshold(max17043_handle_t handle, uint8_t threshold)
{
    if (!handle || threshold > 32) {
        return ESP_ERR_INVALID_ARG;
    }

    max17043_dev_t *dev = (max17043_dev_t *)handle;

    uint8_t reg = MAX17043_REG_CONFIG;
    uint8_t data[2];

    /* Read current CONFIG register */
    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, &data[1], 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CONFIG register: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Modify alert threshold bits (bits 0-4) */
    data[1] = (data[1] & 0xE0) | (threshold & 0x1F);
    data[0] = reg;

    /* Write modified value */
    ret = i2c_master_transmit(dev->i2c_dev, data, 2, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write CONFIG register: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t max17043_get_alert_status(max17043_handle_t handle, uint8_t *alert)
{
    if (!handle || !alert) {
        return ESP_ERR_INVALID_ARG;
    }

    max17043_dev_t *dev = (max17043_dev_t *)handle;

    uint8_t reg = MAX17043_REG_CONFIG;
    uint8_t data[1];

    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, data, 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CONFIG register: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Alert bit is bit 5 */
    *alert = (data[0] >> 5) & 0x01;

    return ESP_OK;
}

esp_err_t max17043_clear_alert(max17043_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    max17043_dev_t *dev = (max17043_dev_t *)handle;

    uint8_t reg = MAX17043_REG_CONFIG;
    uint8_t data[2];

    /* Read current CONFIG register */
    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, &data[1], 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CONFIG register: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Clear alert bit (bit 5) */
    data[1] &= ~(0x01 << 5);
    data[0] = reg;

    /* Write modified value */
    ret = i2c_master_transmit(dev->i2c_dev, data, 2, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write CONFIG register: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t max17043_sleep(max17043_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    max17043_dev_t *dev = (max17043_dev_t *)handle;

    uint8_t reg = MAX17043_REG_CONFIG;
    uint8_t data[2];

    /* Read current CONFIG register */
    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, &data[1], 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CONFIG register: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Set sleep bit (bit 7) */
    data[1] |= (0x01 << 7);
    data[0] = reg;

    /* Write modified value */
    ret = i2c_master_transmit(dev->i2c_dev, data, 2, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write CONFIG register: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "MAX17043 entering sleep mode");
    return ESP_OK;
}

esp_err_t max17043_wake(max17043_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    max17043_dev_t *dev = (max17043_dev_t *)handle;

    uint8_t reg = MAX17043_REG_CONFIG;
    uint8_t data[2];

    /* Read current CONFIG register */
    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, &data[1], 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CONFIG register: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Clear sleep bit (bit 7) */
    data[1] &= ~(0x01 << 7);
    data[0] = reg;

    /* Write modified value */
    ret = i2c_master_transmit(dev->i2c_dev, data, 2, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write CONFIG register: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "MAX17043 waking up");
    return ESP_OK;
}

esp_err_t max17043_quick_start(max17043_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    max17043_dev_t *dev = (max17043_dev_t *)handle;

    uint8_t reg = MAX17043_REG_MODE;
    uint8_t data[2];

    /* Write 0x4000 to MODE register for quick start */
    data[0] = reg;
    data[1] = 0x40;

    esp_err_t ret = i2c_master_transmit(dev->i2c_dev, data, 2, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to quick start: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "MAX17043 quick started");
    return ESP_OK;
}
