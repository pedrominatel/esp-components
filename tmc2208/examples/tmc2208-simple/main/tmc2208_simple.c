#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "tmc2208.h"

static const char *TAG = "example";

void app_main(void)
{

    ESP_LOGI(TAG, "Driver initialization");

    tmc2208_io_config_t config = {
        .step_pin = 18,
        .dir_pin = 19,
        .enable_pin = 0,
        .ms1_pin = 22,
        .ms2_pin = 23,
    };

    ESP_ERROR_CHECK(tmc2208_init(&config));
    ESP_LOGI(TAG, "Driver initialization done");

    while (1) {

    tmc2208_motor_config_t motor_config = {
        .enable_level = 0,
        .dir_clockwise = 0,
        .resolution_hz = 1000000,
        .start_freq_hz = 100,
        .end_freq_hz = 100,
        .accel_samples = 100,
        .uniform_speed_hz = 2000,
        .decel_samples = 100
    };

        tmc2208_move_steps(&config, &motor_config, 1000);
        tmc2208_move_steps(&config, &motor_config, -1000);

        motor_config.uniform_speed_hz = 500;
        tmc2208_move_steps(&config, &motor_config, -2000);

        motor_config.uniform_speed_hz = 20;
        motor_config.accel_samples = 5;
        motor_config.decel_samples = 5;
        motor_config.start_freq_hz = 10;
        motor_config.end_freq_hz = 10;
        tmc2208_move_steps(&config, &motor_config, -400);

        // task delay
        vTaskDelay(4000 / portTICK_PERIOD_MS);
    
    }

    // tmc2208_test(&config);

}
