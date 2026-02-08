#include <string.h>
#include "lc709203.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LC709203";

/* CRC-8 calculation for LC709203F (polynomial: 0x07) */
static uint8_t crc8(uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

typedef struct {
    i2c_master_bus_handle_t i2c_bus;
    i2c_master_dev_handle_t i2c_dev;
    uint8_t i2c_addr;
} lc709203_dev_t;

lc709203_handle_t lc709203_create(i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr)
{
    if (!bus_handle) {
        ESP_LOGE(TAG, "Invalid I2C bus handle");
        return NULL;
    }

    lc709203_dev_t *dev = (lc709203_dev_t *)malloc(sizeof(lc709203_dev_t));
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
        .scl_speed_hz = 100000,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        free(dev);
        return NULL;
    }

    ESP_LOGI(TAG, "LC709203 device created successfully");
    return (lc709203_handle_t)dev;
}

esp_err_t lc709203_delete(lc709203_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    lc709203_dev_t *dev = (lc709203_dev_t *)handle;

    esp_err_t ret = i2c_master_bus_rm_device(dev->i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove I2C device: %s", esp_err_to_name(ret));
        return ret;
    }

    free(dev);
    return ESP_OK;
}

esp_err_t lc709203_read_cell_voltage(lc709203_handle_t handle, uint16_t *voltage)
{
    if (!handle || !voltage) {
        return ESP_ERR_INVALID_ARG;
    }

    lc709203_dev_t *dev = (lc709203_dev_t *)handle;

    uint8_t cmd = LC709203_CMD_CELL_VOLTAGE;
    uint8_t data[3];  /* 2 data bytes + 1 CRC byte */

    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &cmd, 1, data, 3, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read cell voltage: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Verify CRC - calculate over complete SMBus transaction */
    uint8_t crc_data[5];
    crc_data[0] = (dev->i2c_addr << 1);       /* Write address */
    crc_data[1] = cmd;                         /* Command */
    crc_data[2] = (dev->i2c_addr << 1) | 0x01; /* Read address */
    crc_data[3] = data[0];                     /* Data low */
    crc_data[4] = data[1];                     /* Data high */
    
    uint8_t crc_calc = crc8(crc_data, 5);
    
    if (crc_calc != data[2]) {
        ESP_LOGW(TAG, "CRC mismatch for cell voltage (expected 0x%02X, got 0x%02X)", crc_calc, data[2]);
        /* Continue anyway, some implementations are lenient */
    }

    /* Voltage in mV */
    *voltage = (data[1] << 8) | data[0];

    return ESP_OK;
}

esp_err_t lc709203_read_rsoc(lc709203_handle_t handle, uint16_t *rsoc)
{
    if (!handle || !rsoc) {
        return ESP_ERR_INVALID_ARG;
    }

    lc709203_dev_t *dev = (lc709203_dev_t *)handle;

    uint8_t cmd = LC709203_CMD_RSOC;
    uint8_t data[3];  /* 2 data bytes + 1 CRC byte */

    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &cmd, 1, data, 3, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read RSOC: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Verify CRC - calculate over complete SMBus transaction */
    uint8_t crc_data[5];
    crc_data[0] = (dev->i2c_addr << 1);       /* Write address */
    crc_data[1] = cmd;                         /* Command */
    crc_data[2] = (dev->i2c_addr << 1) | 0x01; /* Read address */
    crc_data[3] = data[0];                     /* Data low */
    crc_data[4] = data[1];                     /* Data high */
    
    uint8_t crc_calc = crc8(crc_data, 5);
    
    if (crc_calc != data[2]) {
        ESP_LOGW(TAG, "CRC mismatch for RSOC (expected 0x%02X, got 0x%02X)", crc_calc, data[2]);
    }

    /* RSOC in 0.1% units */
    *rsoc = (data[1] << 8) | data[0];

    return ESP_OK;
}

esp_err_t lc709203_read_ite(lc709203_handle_t handle, uint16_t *ite)
{
    if (!handle || !ite) {
        return ESP_ERR_INVALID_ARG;
    }

    lc709203_dev_t *dev = (lc709203_dev_t *)handle;

    uint8_t cmd = LC709203_CMD_ITE;
    uint8_t data[3];  /* 2 data bytes + 1 CRC byte */

    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &cmd, 1, data, 3, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ITE: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Verify CRC - calculate over complete SMBus transaction */
    uint8_t crc_data[5];
    crc_data[0] = (dev->i2c_addr << 1);       /* Write address */
    crc_data[1] = cmd;                         /* Command */
    crc_data[2] = (dev->i2c_addr << 1) | 0x01; /* Read address */
    crc_data[3] = data[0];                     /* Data low */
    crc_data[4] = data[1];                     /* Data high */
    
    uint8_t crc_calc = crc8(crc_data, 5);
    
    if (crc_calc != data[2]) {
        ESP_LOGW(TAG, "CRC mismatch for ITE (expected 0x%02X, got 0x%02X)", crc_calc, data[2]);
    }

    /* ITE in 0.1% units */
    *ite = (data[1] << 8) | data[0];

    return ESP_OK;
}

esp_err_t lc709203_read_ic_version(lc709203_handle_t handle, uint16_t *version)
{
    if (!handle || !version) {
        return ESP_ERR_INVALID_ARG;
    }

    lc709203_dev_t *dev = (lc709203_dev_t *)handle;

    uint8_t cmd = LC709203_CMD_IC_VERSION;
    uint8_t data[3];  /* 2 data bytes + 1 CRC byte */

    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &cmd, 1, data, 3, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read IC version: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Verify CRC - calculate over complete SMBus transaction */
    uint8_t crc_data[5];
    crc_data[0] = (dev->i2c_addr << 1);       /* Write address */
    crc_data[1] = cmd;                         /* Command */
    crc_data[2] = (dev->i2c_addr << 1) | 0x01; /* Read address */
    crc_data[3] = data[0];                     /* Data low */
    crc_data[4] = data[1];                     /* Data high */
    
    uint8_t crc_calc = crc8(crc_data, 5);
    
    if (crc_calc != data[2]) {
        ESP_LOGW(TAG, "CRC mismatch for IC version (expected 0x%02X, got 0x%02X)", crc_calc, data[2]);
    }

    *version = (data[1] << 8) | data[0];

    return ESP_OK;
}

esp_err_t lc709203_set_apa(lc709203_handle_t handle, uint8_t apa)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    lc709203_dev_t *dev = (lc709203_dev_t *)handle;

    uint8_t buffer[4];
    buffer[0] = (dev->i2c_addr << 1);
    buffer[1] = LC709203_CMD_APA;
    buffer[2] = apa;
    buffer[3] = 0x00;
    
    uint8_t crc = crc8(buffer, 4);
    
    uint8_t data[4];
    data[0] = LC709203_CMD_APA;
    data[1] = apa;
    data[2] = 0x00;
    data[3] = crc;

    esp_err_t ret = i2c_master_transmit(dev->i2c_dev, data, 4, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set APA: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t lc709203_set_cell_temp(lc709203_handle_t handle, uint16_t temp_celsius)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    lc709203_dev_t *dev = (lc709203_dev_t *)handle;

    /* Convert Celsius to 0.1K units (Kelvin * 10) */
    uint16_t temp_kelvin = (temp_celsius + 273) * 10;

    uint8_t buffer[4];
    buffer[0] = (dev->i2c_addr << 1);
    buffer[1] = LC709203_CMD_CELL_TEMP;
    buffer[2] = temp_kelvin & 0xFF;
    buffer[3] = (temp_kelvin >> 8) & 0xFF;
    
    uint8_t crc = crc8(buffer, 4);
    
    uint8_t data[4];
    data[0] = LC709203_CMD_CELL_TEMP;
    data[1] = temp_kelvin & 0xFF;
    data[2] = (temp_kelvin >> 8) & 0xFF;
    data[3] = crc;

    esp_err_t ret = i2c_master_transmit(dev->i2c_dev, data, 4, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set cell temperature: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t lc709203_set_battery_profile(lc709203_handle_t handle, uint8_t profile)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    lc709203_dev_t *dev = (lc709203_dev_t *)handle;

    uint8_t buffer[4];
    buffer[0] = (dev->i2c_addr << 1);
    buffer[1] = LC709203_CMD_CHANGE_PARAM;
    buffer[2] = profile;
    buffer[3] = 0x00;
    
    uint8_t crc = crc8(buffer, 4);
    
    uint8_t data[4];
    data[0] = LC709203_CMD_CHANGE_PARAM;
    data[1] = profile;
    data[2] = 0x00;
    data[3] = crc;

    esp_err_t ret = i2c_master_transmit(dev->i2c_dev, data, 4, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set battery profile: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t lc709203_set_rsoc_alarm(lc709203_handle_t handle, uint8_t alarm_percent)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    lc709203_dev_t *dev = (lc709203_dev_t *)handle;

    uint8_t buffer[4];
    buffer[0] = (dev->i2c_addr << 1);
    buffer[1] = LC709203_CMD_ALARM_LOW_RSOC;
    buffer[2] = alarm_percent;
    buffer[3] = 0x00;
    
    uint8_t crc = crc8(buffer, 4);
    
    uint8_t data[4];
    data[0] = LC709203_CMD_ALARM_LOW_RSOC;
    data[1] = alarm_percent;
    data[2] = 0x00;
    data[3] = crc;

    esp_err_t ret = i2c_master_transmit(dev->i2c_dev, data, 4, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set RSOC alarm: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t lc709203_set_power_mode(lc709203_handle_t handle, lc709203_power_mode_t mode)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    lc709203_dev_t *dev = (lc709203_dev_t *)handle;

    uint8_t buffer[4];
    buffer[0] = (dev->i2c_addr << 1);
    buffer[1] = LC709203_CMD_POWER_MODE;
    buffer[2] = mode;
    buffer[3] = 0x00;
    
    uint8_t crc = crc8(buffer, 4);
    
    uint8_t data[4];
    data[0] = LC709203_CMD_POWER_MODE;
    data[1] = mode;
    data[2] = 0x00;
    data[3] = crc;

    esp_err_t ret = i2c_master_transmit(dev->i2c_dev, data, 4, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set power mode: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t lc709203_init_rsoc(lc709203_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    lc709203_dev_t *dev = (lc709203_dev_t *)handle;

    uint8_t buffer[4];
    buffer[0] = (dev->i2c_addr << 1);
    buffer[1] = LC709203_CMD_INIT_RSOC;
    buffer[2] = 0xAA;
    buffer[3] = 0x55;
    
    uint8_t crc = crc8(buffer, 4);
    
    uint8_t data[4];
    data[0] = LC709203_CMD_INIT_RSOC;
    data[1] = 0xAA;
    data[2] = 0x55;
    data[3] = crc;

    esp_err_t ret = i2c_master_transmit(dev->i2c_dev, data, 4, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize RSOC: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}
esp_err_t lc709203_read_status(lc709203_handle_t handle, uint16_t *status)
{
    if (!handle || !status) {
        return ESP_ERR_INVALID_ARG;
    }

    lc709203_dev_t *dev = (lc709203_dev_t *)handle;

    uint8_t cmd = LC709203_CMD_STATUS_BIT;
    uint8_t data[3];  /* 2 data bytes + 1 CRC byte */

    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &cmd, 1, data, 3, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read status: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Verify CRC */
    uint8_t crc_data[5];
    crc_data[0] = (dev->i2c_addr << 1);
    crc_data[1] = cmd;
    crc_data[2] = (dev->i2c_addr << 1) | 0x01;
    crc_data[3] = data[0];
    crc_data[4] = data[1];
    
    uint8_t crc_calc = crc8(crc_data, 5);
    
    if (crc_calc != data[2]) {
        ESP_LOGW(TAG, "CRC mismatch for status (expected 0x%02X, got 0x%02X)", crc_calc, data[2]);
    }

    *status = (data[1] << 8) | data[0];

    return ESP_OK;
}

esp_err_t lc709203_set_temperature_mode(lc709203_handle_t handle, uint8_t mode)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    lc709203_dev_t *dev = (lc709203_dev_t *)handle;

    uint8_t buffer[4];
    buffer[0] = (dev->i2c_addr << 1);
    buffer[1] = LC709203_CMD_STATUS_BIT;
    buffer[2] = mode;
    buffer[3] = 0x00;
    
    uint8_t crc = crc8(buffer, 4);
    
    uint8_t data[4];
    data[0] = LC709203_CMD_STATUS_BIT;
    data[1] = mode;
    data[2] = 0x00;
    data[3] = crc;

    esp_err_t ret = i2c_master_transmit(dev->i2c_dev, data, 4, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set temperature mode: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t lc709203_read_cell_temp(lc709203_handle_t handle, uint16_t *temp_kelvin)
{
    if (!handle || !temp_kelvin) {
        return ESP_ERR_INVALID_ARG;
    }

    lc709203_dev_t *dev = (lc709203_dev_t *)handle;

    uint8_t cmd = LC709203_CMD_CELL_TEMP;
    uint8_t data[3];  /* 2 data bytes + 1 CRC byte */

    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &cmd, 1, data, 3, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read cell temperature: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Verify CRC */
    uint8_t crc_data[5];
    crc_data[0] = (dev->i2c_addr << 1);
    crc_data[1] = cmd;
    crc_data[2] = (dev->i2c_addr << 1) | 0x01;
    crc_data[3] = data[0];
    crc_data[4] = data[1];
    
    uint8_t crc_calc = crc8(crc_data, 5);
    
    if (crc_calc != data[2]) {
        ESP_LOGW(TAG, "CRC mismatch for cell temp (expected 0x%02X, got 0x%02X)", crc_calc, data[2]);
    }

    /* Temperature in 0.1K units */
    *temp_kelvin = (data[1] << 8) | data[0];

    return ESP_OK;
}

esp_err_t lc709203_read_current_direction(lc709203_handle_t handle, uint16_t *direction)
{
    if (!handle || !direction) {
        return ESP_ERR_INVALID_ARG;
    }

    lc709203_dev_t *dev = (lc709203_dev_t *)handle;

    uint8_t cmd = LC709203_CMD_CURRENT_DIR;
    uint8_t data[3];  /* 2 data bytes + 1 CRC byte */

    esp_err_t ret = i2c_master_transmit_receive(dev->i2c_dev, &cmd, 1, data, 3, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read current direction: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Verify CRC */
    uint8_t crc_data[5];
    crc_data[0] = (dev->i2c_addr << 1);
    crc_data[1] = cmd;
    crc_data[2] = (dev->i2c_addr << 1) | 0x01;
    crc_data[3] = data[0];
    crc_data[4] = data[1];
    
    uint8_t crc_calc = crc8(crc_data, 5);
    
    if (crc_calc != data[2]) {
        ESP_LOGW(TAG, "CRC mismatch for current direction (expected 0x%02X, got 0x%02X)", crc_calc, data[2]);
    }

    /* 0x0000 = Discharging, 0x0001 = Charging, 0xFFFF = Auto */
    *direction = (data[1] << 8) | data[0];

    return ESP_OK;
}

esp_err_t lc709203_set_current_direction(lc709203_handle_t handle, uint16_t direction)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    lc709203_dev_t *dev = (lc709203_dev_t *)handle;

    uint8_t buffer[4];
    buffer[0] = (dev->i2c_addr << 1);
    buffer[1] = LC709203_CMD_CURRENT_DIR;
    buffer[2] = direction & 0xFF;
    buffer[3] = (direction >> 8) & 0xFF;
    
    uint8_t crc = crc8(buffer, 4);
    
    uint8_t data[4];
    data[0] = LC709203_CMD_CURRENT_DIR;
    data[1] = direction & 0xFF;
    data[2] = (direction >> 8) & 0xFF;
    data[3] = crc;

    esp_err_t ret = i2c_master_transmit(dev->i2c_dev, data, 4, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set current direction: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t lc709203_trigger_update(lc709203_handle_t handle)
{
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    static uint32_t update_count = 0;
    update_count++;

    /* Refresh temperature in I2C mode every 5 readings (keeps algorithm active) */
    if (update_count % 5 == 0) {
        lc709203_set_cell_temp(handle, 25);
    }

    /* Force RSOC recalculation every 10 readings */
    if (update_count % 10 == 0) {
        lc709203_init_rsoc(handle);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* Reading cell temperature triggers the IC to refresh all measurements */
    uint16_t temp;
    esp_err_t ret = lc709203_read_cell_temp(handle, &temp);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Give sensor time to complete measurement cycle */
    vTaskDelay(pdMS_TO_TICKS(300));
    
    return ESP_OK;
}

esp_err_t lc709203_detect_thermistor(lc709203_handle_t handle, bool *thermistor_connected)
{
    if (!handle || !thermistor_connected) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Set to thermistor mode temporarily */
    esp_err_t ret = lc709203_set_temperature_mode(handle, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set thermistor mode for detection: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Wait for sensor to stabilize */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Read temperature multiple times to verify consistency */
    uint16_t temp_readings[3];
    bool valid_readings = true;

    for (int i = 0; i < 3; i++) {
        ret = lc709203_read_cell_temp(handle, &temp_readings[i]);
        if (ret != ESP_OK) {
            valid_readings = false;
            break;
        }
        
        /* Convert to Celsius for validation */
        float temp_celsius = (temp_readings[i] / 10.0f) - 273.15f;
        
        /* Check if temperature is in reasonable range (-20°C to +60°C) */
        if (temp_celsius < -20.0f || temp_celsius > 60.0f) {
            valid_readings = false;
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (valid_readings) {
        /* Check if readings are consistent (within 5°C) */
        float temp_min = (temp_readings[0] / 10.0f) - 273.15f;
        float temp_max = temp_min;
        
        for (int i = 1; i < 3; i++) {
            float temp = (temp_readings[i] / 10.0f) - 273.15f;
            if (temp < temp_min) temp_min = temp;
            if (temp > temp_max) temp_max = temp;
        }
        
        if ((temp_max - temp_min) < 5.0f) {
            *thermistor_connected = true;
            ESP_LOGI(TAG, "Thermistor detected! Temperature: %.1f°C", temp_min);
            return ESP_OK;
        }
    }

    /* Thermistor not detected or readings invalid */
    *thermistor_connected = false;
    ESP_LOGI(TAG, "No thermistor detected (invalid or out-of-range readings)");
    
    /* Restore to I2C mode with fixed temperature */
    ret = lc709203_set_temperature_mode(handle, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to restore I2C mode: %s", esp_err_to_name(ret));
    }
    
    ret = lc709203_set_cell_temp(handle, 25);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set default temperature: %s", esp_err_to_name(ret));
    }
    
    return ESP_OK;
}