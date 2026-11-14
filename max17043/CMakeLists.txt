if(IDF_VERSION_MAJOR EQUAL 5 AND IDF_VERSION_MINOR LESS 3)
    set(i2c_driver_component driver)
else()
    set(i2c_driver_component esp_driver_i2c)
endif()

idf_component_register(
    SRCS "max17043.c"
    INCLUDE_DIRS "include"
    REQUIRES ${i2c_driver_component}
)
