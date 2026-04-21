idf_component_register(
    SRCS "tvbgone_core.c" "WORLDcodes.cpp"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_rmt
)
