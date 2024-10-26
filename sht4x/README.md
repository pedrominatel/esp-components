# SHT4x temperature and humidity sensor driver

The SHT4x is a digital sensor platform for measuring relative humidity and temperature with various accuracy options. It uses an ultra-low power I2C interface (0.4 μW) with multiple preconfigured addresses.

The sensor has an internal heater with three levels for challenging environments and comes in a four-pin SMT-compatible package with options for a PTFE membrane or protective cover. Calibration certificates compliant with ISO17025, identifiable by unique serial numbers, are also available.

## How to use

```c
void sht4x_read_task(void *pvParameters)
{
    float temperature, humidity;

    while (1) {
        
        esp_err_t err = sht4x_start_measurement(sht4x_handle, SHT4X_CMD_READ_MEASUREMENT_HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        err = sht4x_read_measurement(sht4x_handle, &temperature, &humidity);

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Temperature: %.2f C, Humidity: %.2f %%", temperature, humidity);
        } else {
            ESP_LOGE(TAG, "Failed to read temperature and humidity");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

## References

[Datasheet](https://sensirion.com/media/documents/33FD6951/662A593A/HT_DS_Datasheet_SHT4x.pdf)