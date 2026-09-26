idf_component_register(
    SRCS "ultraled.c" "ultraled_encoder.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES esp_driver_parlio esp_driver_bitscrambler esp_driver_gpio esp_hw_support esp_timer
    REQUIRED_IDF_TARGETS esp32p4
)

target_bitscrambler_add_src("ws2812.bsasm")
target_bitscrambler_add_src("sk6812.bsasm")
