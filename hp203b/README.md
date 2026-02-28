# HP203B Pressure Sensor Driver for ESP-IDF

Minimal, robust ESP-IDF driver for the HP203B barometric pressure sensor (I²C).
Provides pressure (Pa), temperature (°C) and calculated altitude (m).

## Features

- I²C interface (configurable port/pins/frequency)
- One-shot or continuous conversion (polling)
- Pressure, temperature(WIP), altitude (WIP)
- Simple error handling via esp_err_t
- Lightweight, no dynamic allocation in the hot path

## Hardware

- Bus: I²C (standard/fast mode)
- Address: 0x76 (common default; check your module/datasheet)
- Voltage: typically 3.3 V (confirm your board’s regulator)

## How to use

Example code:

```c
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_types.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "hp203b.h"
#include "driver/i2c_master.h"

#define HP203B_SDA_GPIO           CONFIG_HP203B_I2C_SDA
#define HP203B_SCL_GPIO           CONFIG_HP203B_I2C_SCL

static const char *TAG = "HP203B";
i2c_master_bus_handle_t bus_handle = NULL;
hp203b_handle_t sensor_handle = NULL;

esp_err_t init_i2c(i2c_master_bus_handle_t *i2c_bus)
{
    if (!i2c_bus) return ESP_ERR_INVALID_ARG;

    const i2c_master_bus_config_t cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = CONFIG_HP203B_I2C_NUM,   // must be I2C_NUM_0 on ESP32-C3
        .scl_io_num = HP203B_SCL_GPIO,
        .sda_io_num = HP203B_SDA_GPIO,
        .flags = {
            .enable_internal_pullup = true,   // ok for bring-up; prefer external pull-ups
        },
        .glitch_ignore_cnt = 7,
    };
    return i2c_new_master_bus(&cfg, i2c_bus);
}

void read_sensor_task(void *pvParameters)
{
    hp203b_handle_t sensor = (hp203b_handle_t)pvParameters;
    
    while (1) {
        if (hp203b_read_press(sensor) == ESP_OK) {
            uint32_t sample_pa = hp203b_get_press(sensor);
            ESP_LOGI(TAG, "Pressure: %lu Pa", sample_pa);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "This is the HP203B sensor example");

    ESP_ERROR_CHECK(init_i2c(&bus_handle));

    if (hp203b_init(bus_handle, &sensor_handle) == ESP_OK) {
        ESP_LOGI(TAG, "Sensor initialization ok!");
        xTaskCreate(&read_sensor_task, "read_sensor_task", 2048, sensor_handle, 2, NULL);
    } else {
        ESP_LOGE(TAG, "Sensor initialization error!");
    }
}

```

Expected log output:

```bash
I (0) HP203B: This is the HP203B sensor example
I (...) HP203B: Sensor initialization ok!
I (...) HP203B: Pressure: 102639 Pa
I (...) HP203B: Pressure: 102642 Pa
...
```

## Migration from v0.2.0

**⚠️ Breaking Changes in v0.3.0:** The API has changed to support handle-based architecture for thread safety.

See [MIGRATION.md](MIGRATION.md) for detailed migration instructions from v0.2.0 to v0.3.0.
