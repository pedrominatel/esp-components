#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "lc709203.h"
#include "lc709203_config.h"

static const char *TAG = "LC709203_EXAMPLE";

static void lc709203_read_task(void *arg)
{
    lc709203_handle_t sensor = (lc709203_handle_t)arg;
    uint16_t voltage;
    uint16_t rsoc;
    uint16_t ite;
    uint16_t temp_kelvin;
    uint16_t direction;
    
    /* Track voltage history for charge/discharge detection */
    static uint16_t prev_voltage = 0;
    static uint16_t voltage_history[5] = {0};
    static uint8_t history_index = 0;

    while (1) {
        /* Trigger sensor to refresh measurements (includes delay) */
        esp_err_t ret = lc709203_trigger_update(sensor);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to trigger update: %s", esp_err_to_name(ret));
        }

        ret = lc709203_read_cell_voltage(sensor, &voltage);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Cell Voltage: %u mV", voltage);
            
            /* Auto-detect charging/discharging based on voltage trend */
            if (prev_voltage > 0) {
                voltage_history[history_index] = voltage;
                history_index = (history_index + 1) % 5;
                
                /* Calculate average voltage change over last 5 readings */
                int16_t voltage_change = 0;
                uint8_t valid_samples = 0;
                for (int i = 0; i < 5; i++) {
                    if (voltage_history[i] > 0) {
                        voltage_change += (int16_t)voltage_history[i] - (int16_t)prev_voltage;
                        valid_samples++;
                    }
                }
                
                if (valid_samples >= 3) {
                    int16_t avg_change = voltage_change / valid_samples;
                    
                    /* Set direction based on voltage trend (>10mV change = significant) */
                    if (avg_change > 10) {
                        /* Voltage rising - charging */
                        lc709203_set_current_direction(sensor, LC709203_CURRENT_DIR_CHARGE);
                        ESP_LOGI(TAG, "Detected: Charging (voltage rising)");
                    } else if (avg_change < -10) {
                        /* Voltage falling - discharging */
                        lc709203_set_current_direction(sensor, LC709203_CURRENT_DIR_DISCHARGE);
                        ESP_LOGI(TAG, "Detected: Discharging (voltage falling)");
                    }
                }
            }
            prev_voltage = voltage;
        } else {
            ESP_LOGE(TAG, "Failed to read voltage: %s", esp_err_to_name(ret));
        }

        ret = lc709203_read_cell_temp(sensor, &temp_kelvin);
        if (ret == ESP_OK) {
            /* Convert from 0.1K to Celsius */
            float temp_celsius = (temp_kelvin / 10.0f) - 273.15f;
            ESP_LOGI(TAG, "Temperature: %.1f °C", temp_celsius);
        } else {
            ESP_LOGE(TAG, "Failed to read temperature: %s", esp_err_to_name(ret));
        }

        ret = lc709203_read_current_direction(sensor, &direction);
        if (ret == ESP_OK) {
            const char *dir_str = (direction == 0x0000) ? "Discharging" : 
                                  (direction == 0x0001) ? "Charging" : 
                                  (direction == 0xFFFF) ? "Auto" : "Unknown";
            ESP_LOGI(TAG, "Current Direction: %s", dir_str);
        }

        ret = lc709203_read_rsoc(sensor, &rsoc);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "RSOC: %u.%u %%", rsoc / 10, rsoc % 10);
        } else {
            ESP_LOGE(TAG, "Failed to read RSOC: %s", esp_err_to_name(ret));
        }

        ret = lc709203_read_ite(sensor, &ite);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "ITE: %u.%u %%", ite / 10, ite % 10);
        } else {
            ESP_LOGE(TAG, "Failed to read ITE: %s", esp_err_to_name(ret));
        }

        ESP_LOGI(TAG, "---");

        vTaskDelay(pdMS_TO_TICKS(5000));  /* Read every 5 seconds */
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting LC709203F fuel gauge example");

    /* Initialize I2C master bus */
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = LC709203_I2C_PORT,
        .scl_io_num = LC709203_I2C_SCL_PIN,
        .sda_io_num = LC709203_I2C_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    i2c_master_bus_handle_t bus_handle;
    esp_err_t ret = i2c_new_master_bus(&bus_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "I2C bus initialized");

    /* Probe the sensor first to check if it is connected to the bus */
    esp_err_t err = i2c_master_probe(bus_handle, LC709203_I2C_ADDRESS, 200);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LC709203F not found at address 0x%02X", LC709203_I2C_ADDRESS);
        return;
    }

    ESP_LOGI(TAG, "LC709203F detected on I2C bus");

    /* Create LC709203 device handle */
    lc709203_handle_t sensor_handle = lc709203_create(bus_handle, LC709203_I2C_ADDRESS);
    if (!sensor_handle) {
        ESP_LOGE(TAG, "Failed to create LC709203 device");
        return;
    }

    ESP_LOGI(TAG, "LC709203F device created successfully");

    /* Read IC version */
    uint16_t version;
    ret = lc709203_read_ic_version(sensor_handle, &version);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "LC709203F IC Version: 0x%04X", version);
    } else {
        ESP_LOGE(TAG, "Failed to read version: %s", esp_err_to_name(ret));
    }

    /* Configure battery capacity for 1000mAh battery */
    ret = lc709203_set_apa(sensor_handle, LC709203_APA_1000MAH);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Battery capacity configured for 1000mAh");
    } else {
        ESP_LOGE(TAG, "Failed to set APA: %s", esp_err_to_name(ret));
    }

    /* Set battery profile (0 = default profile) */
    ret = lc709203_set_battery_profile(sensor_handle, 0);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Battery profile set to default");
    } else {
        ESP_LOGE(TAG, "Failed to set battery profile: %s", esp_err_to_name(ret));
    }

    /* Detect if thermistor is connected */
    bool thermistor_available = false;
    ret = lc709203_detect_thermistor(sensor_handle, &thermistor_available);
    if (ret == ESP_OK) {
        if (thermistor_available) {
            ESP_LOGI(TAG, "Using thermistor for automatic temperature sensing");
            /* Thermistor mode already configured by detection function */
        } else {
            ESP_LOGI(TAG, "Using I2C mode with fixed 25°C temperature");
            /* I2C mode already configured by detection function */
        }
    } else {
        ESP_LOGE(TAG, "Thermistor detection failed, using I2C mode with 25°C");
        lc709203_set_temperature_mode(sensor_handle, 0);
        lc709203_set_cell_temp(sensor_handle, 25);
    }

    /* Set current direction to auto-detect */
    ret = lc709203_set_current_direction(sensor_handle, LC709203_CURRENT_DIR_AUTO);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Current direction set to auto-detect");
    } else {
        ESP_LOGE(TAG, "Failed to set current direction: %s", esp_err_to_name(ret));
    }

    /* Initialize RSOC algorithm */
    ret = lc709203_init_rsoc(sensor_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "RSOC initialized");
    } else {
        ESP_LOGE(TAG, "Failed to initialize RSOC: %s", esp_err_to_name(ret));
    }

    /* Read status to verify sensor is operational */
    uint16_t status;
    ret = lc709203_read_status(sensor_handle, &status);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Status register: 0x%04X (0x0001 = thermistor mode)", status);
    } else {
        ESP_LOGE(TAG, "Failed to read status: %s", esp_err_to_name(ret));
    }

    /* Wait a moment for initialization to complete */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Create sensor reader task */
    ret = xTaskCreate(
        lc709203_read_task,
        "lc709203_read_task",
        2048,
        sensor_handle,
        5,
        NULL
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor reader task");
        lc709203_delete(sensor_handle);
    }
}
