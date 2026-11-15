if(IDF_VERSION_MAJOR EQUAL 5 AND IDF_VERSION_MINOR LESS 3)
    set(driver_components driver)
else()
    set(driver_components esp_driver_gpio esp_driver_rmt)
endif()

idf_component_register(
    SRCS "stepper_motor_encoder.c" "tmc2208.c"
    INCLUDE_DIRS "include"
    REQUIRES ${driver_components}
)
