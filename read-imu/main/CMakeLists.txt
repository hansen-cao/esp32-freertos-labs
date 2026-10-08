idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer esp_driver_uart esp_driver_gpio
    PRIV_REQUIRES sh2
)