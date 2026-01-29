#include <string.h>
#include "lis3dh.h"
#include "esp_log.h"

static const char *TAG = "LIS3DH";

typedef struct {
    i2c_master_bus_handle_t i2c_bus;
    i2c_master_dev_handle_t i2c_dev;
    uint8_t i2c_addr;
    lis3dh_fs_t fs;
} lis3dh_dev_t;

esp_err_t lis3dh_init(const lis3dh_config_t *config, lis3dh_handle_t *out_handle)
{
    if (!config || !out_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    lis3dh_dev_t *dev = (lis3dh_dev_t *)malloc(sizeof(lis3dh_dev_t));
    if (!dev) {
        ESP_LOGE(TAG, "Failed to allocate memory for device handle");
        return ESP_ERR_NO_MEM;
    }

    dev->i2c_bus = config->i2c_bus;
    dev->i2c_addr = config->i2c_addr;
    dev->fs = config->fs;

    /* Create I2C device */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = config->i2c_addr,
        .scl_speed_hz = 400000,
    };

    esp_err_t ret = i2c_master_bus_add_device(config->i2c_bus, &dev_cfg, &dev->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        free(dev);
        return ret;
    }

    /* Verify device by reading WHO_AM_I */
    uint8_t id;
    ret = lis3dh_whoami((lis3dh_handle_t)dev, &id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        i2c_master_bus_rm_device(dev->i2c_dev);
        free(dev);
        return ret;
    }

    if (id != LIS3DH_ID) {
        ESP_LOGE(TAG, "Invalid device ID: 0x%02X (expected 0x%02X)", id, LIS3DH_ID);
        i2c_master_bus_rm_device(dev->i2c_dev);
        free(dev);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "LIS3DH initialized successfully");

    /* Set output data rate */
    ret = lis3dh_set_odr((lis3dh_handle_t)dev, config->odr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ODR");
        i2c_master_bus_rm_device(dev->i2c_dev);
        free(dev);
        return ret;
    }

    /* Set full scale range */
    ret = lis3dh_set_fs((lis3dh_handle_t)dev, config->fs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set full scale range");
        i2c_master_bus_rm_device(dev->i2c_dev);
        free(dev);
        return ret;
    }

    *out_handle = (lis3dh_handle_t)dev;

    return ESP_OK;
}

esp_err_t lis3dh_deinit(lis3dh_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    lis3dh_dev_t *dev = (lis3dh_dev_t *)handle;

    esp_err_t ret = i2c_master_bus_rm_device(dev->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove I2C device: %s", esp_err_to_name(ret));
        return ret;
    }

    free(dev);

    return ESP_OK;
}

esp_err_t lis3dh_whoami(lis3dh_handle_t handle, uint8_t *id)
{
    if (!handle || !id) {
        return ESP_ERR_INVALID_ARG;
    }

    lis3dh_dev_t *dev = (lis3dh_dev_t *)handle;

    uint8_t reg = LIS3DH_REG_WHO_AM_I;
    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, id, 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t lis3dh_read_accel(lis3dh_handle_t handle, lis3dh_accel_t *accel)
{
    if (!handle || !accel) {
        return ESP_ERR_INVALID_ARG;
    }

    lis3dh_dev_t *dev = (lis3dh_dev_t *)handle;

    /* Read 6 bytes of acceleration data */
    uint8_t reg = LIS3DH_REG_OUT_X_L | 0x80;  /* Set multi-byte read bit */
    uint8_t data[6];
    
    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, data, 6, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read acceleration data: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Convert to 16-bit values */
    accel->x = (int16_t)((data[1] << 8) | data[0]);
    accel->y = (int16_t)((data[3] << 8) | data[2]);
    accel->z = (int16_t)((data[5] << 8) | data[4]);

    return ESP_OK;
}

esp_err_t lis3dh_set_odr(lis3dh_handle_t handle, lis3dh_odr_t odr)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    lis3dh_dev_t *dev = (lis3dh_dev_t *)handle;

    uint8_t reg = LIS3DH_REG_CTRL_REG1;
    uint8_t data[2];

    /* Read current register value */
    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, &data[1], 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CTRL_REG1: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Modify ODR bits (bits 4-7) */
    data[1] = (data[1] & 0x0F) | ((odr & 0x0F) << 4);
    data[1] |= 0x07;  /* Enable X, Y, Z axes */

    data[0] = reg;

    /* Write modified value */
    ret = i2c_master_transmit(dev->i2c_dev, data, 2, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write CTRL_REG1: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t lis3dh_set_fs(lis3dh_handle_t handle, lis3dh_fs_t fs)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    lis3dh_dev_t *dev = (lis3dh_dev_t *)handle;
    dev->fs = fs;

    uint8_t reg = LIS3DH_REG_CTRL_REG4;
    uint8_t data[2];

    /* Read current register value */
    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, &data[1], 1, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read CTRL_REG4: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Modify full scale range bits (bits 4-5) */
    data[1] = (data[1] & 0xCF) | ((fs & 0x03) << 4);
    data[1] |= 0x08;  /* Enable high resolution mode */

    data[0] = reg;

    /* Write modified value */
    ret = i2c_master_transmit(dev->i2c_dev, data, 2, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write CTRL_REG4: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}
