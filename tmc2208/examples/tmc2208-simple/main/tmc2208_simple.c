#include <stdbool.h>
#include <stdint.h>
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

    tmc2208_io_config_t config_io_motor1 = {
        .step_pin = CONFIG_MOTOR_STEP_IO,
        .dir_pin = CONFIG_MOTOR_DIR_IO,
        .enable_pin = CONFIG_MOTOR_EN_IO,
        .ms1_pin = CONFIG_MOTOR_MS1_IO,
        .ms2_pin = CONFIG_MOTOR_MS2_IO,
    };

    ESP_ERROR_CHECK(tmc2208_init(&config_io_motor1));
    ESP_LOGI(TAG, "Driver initialization done");

    tmc2208_motor_config_t motor1_config = {
        .enable_level = 0,
        .dir_clockwise = 0,
        .resolution_hz = 500000,
        .start_freq_hz = 100,
        .end_freq_hz = 100,
        .accel_samples = 250,
        .uniform_speed_hz = 500,
        .decel_samples = 250,
        .microstep = TMC2208_MICROSTEP_4
    };

    // Teste the step move without acceleration and deceleration
    tmc2208_move_steps(&config_io_motor1, &motor1_config, 1600, false, false);
    tmc2208_move_steps(&config_io_motor1, &motor1_config, -1600, false, false);
    // Teste the step move with acceleration and deceleration
    tmc2208_move_steps(&config_io_motor1, &motor1_config, 1100, true, true);
    tmc2208_move_steps(&config_io_motor1, &motor1_config, -1100, true, true);

    // Test the uniform continous move
    // Enable motor with the encoder
    tmc2208_enable(&config_io_motor1, &motor1_config);
    // Set the speed
    motor1_config.uniform_speed_hz = 10;
    // Set the direction
    tmc2208_set_dir(&config_io_motor1, &motor1_config, TMC2208_DIR_CCW);
    // Move 400 steps
    uint32_t steps = 400;
    while (1) {
        tmc2208_uniform_move_steps(&config_io_motor1, &motor1_config, 10);
        steps -= 10;
        if(steps == 0) {
            break;
        }
    }
    // Disable motor
    tmc2208_disable(&config_io_motor1, &motor1_config);

}
