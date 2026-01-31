idf_component_register(
    SRCS "stepper_motor_encoder.c" "tmc2208.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio
)