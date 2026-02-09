idf_component_register(
    SRCS "tvbgone_ir.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_rmt
)
