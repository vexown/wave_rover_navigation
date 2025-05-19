idf_component_register(
    SRCS "Source/WiFi_AP.c"
    INCLUDE_DIRS "Include"
    REQUIRES esp_wifi esp_http_server nvs_flash Common
)