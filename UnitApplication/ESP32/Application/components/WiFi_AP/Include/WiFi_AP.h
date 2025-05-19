/**
 * ****************************************************************************
 * WiFi_AP.h
 *
 * ****************************************************************************
 */

 #ifndef WIFI_AP_H
 #define WIFI_AP_H
 
 #include "esp_err.h"
 
 /**
  * @brief Custom WiFi Access Point configuration
  */
 typedef struct {
     const char* ssid;           // AP SSID (name)
     const char* password;       // AP password (NULL for open network)
     uint8_t channel;            // WiFi channel
     uint8_t max_connections;    // Maximum number of stations
 } wifi_ap_custom_config_t;
 
 /**
  * @brief Initialize and start WiFi Access Point
  * 
  * @param config WiFi AP configuration
  * @return esp_err_t ESP_OK on success
  */
 esp_err_t wifi_ap_init(wifi_ap_custom_config_t *config);
 
 /**
  * @brief Start HTTP server
  * 
  * @return esp_err_t ESP_OK on success
  */
 esp_err_t http_server_start(void);
 
 /**
  * @brief Stop HTTP server
  * 
  * @return esp_err_t ESP_OK on success
  */
 esp_err_t http_server_stop(void);
 
 #endif /* WIFI_AP_H */