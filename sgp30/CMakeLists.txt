idf_component_register(
    SRCS "sgp30.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c
)

# i2c_bus (espressif/esp-iot-solution) is an optional dependency.
# When the component is present in the project, SGP30 automatically exposes
# sgp30_init_i2cbus() for use with i2c_bus device handles.
idf_component_optional_requires(PRIVATE espressif__i2c_bus)
