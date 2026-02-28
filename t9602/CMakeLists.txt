idf_component_register(
    SRCS "t9602.c"
    INCLUDE_DIRS "include"
    REQUIRES "esp_driver_gpio" "esp_driver_i2c"
)

idf_component_optional_requires(PRIVATE espressif__i2c_bus)
