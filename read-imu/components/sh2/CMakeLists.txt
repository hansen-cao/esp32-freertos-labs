# Build script for the CEVA SH-2 sensor hub library (UART flavor).
# Place this file in components/sh2/ alongside the library sources.

idf_component_register(
    SRCS
        "sh2.c"
        "sh2_SensorValue.c"
        "sh2_util.c"
        "shtp.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        driver
        esp_timer
)