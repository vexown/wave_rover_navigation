idf_component_register(
    SRCS "Source/GNSS_ublox.c"
    INCLUDE_DIRS "Include"
    REQUIRES driver esp_common freertos 
)