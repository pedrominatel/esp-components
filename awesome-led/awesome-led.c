#include <locale>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <esp_err.h>
#include "awesome-led.h"

enum led_effect_t {
    LED_EFFECT_NONE,
    LED_EFFECT_FADE,
    LED_EFFECT_BLINK
};

typedef struct led_effect {
    led_effect_t effect;
    int32_t duration;
    uint16_t fade_in_duration;
    uint16_t fade_out_duration;
    uint16_t on_duration;
    uint16_t off_duration;
    uint8_t brightness;
} led_effect_t;

typedef struct led_device {
    uint8_t gpio;
    uint8_t invert_state;
    uint8_t inital_state;
} led_device_t;

esp_err_t al_led_configure_gpio(led_device_t *led_device) {
    
    esp_err_t err = ESP_OK;

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << led_device->gpio);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    err = gpio_config(&io_conf);
    
    return err;
}

esp_err_t al_led_reset(led_device_t *led_device) {
    esp_err_t err = ESP_OK;
}


/*
led configure
gpio
invert-state
led_device
*/

/*
fade led effect
fade in duration in ms
fade out duration in ms
*/

/*
blink effect
on duration in ms
off duration in ms
*/

/*
led set
led_device
state
led effect
loop
duration
*/

/*
led stop
led_device
*/
