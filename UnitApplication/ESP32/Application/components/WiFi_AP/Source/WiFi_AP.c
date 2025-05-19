/**
 * ****************************************************************************
 * WiFi_AP.c
 *
 * ****************************************************************************
 */

/*******************************************************************************/
/*                                INCLUDES                                     */
/*******************************************************************************/
#include "WiFi_AP.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "Common.h"

/*******************************************************************************/
/*                                 MACROS                                      */
/*******************************************************************************/
#define TAG "WiFi_AP"

/*******************************************************************************/
/*                               DATA TYPES                                    */
/*******************************************************************************/

/*******************************************************************************/
/*                            STATIC VARIABLES                                 */
/*******************************************************************************/
static httpd_handle_t server = NULL;

/*******************************************************************************/
/*                        STATIC FUNCTION DECLARATIONS                         */
/*******************************************************************************/
static esp_err_t root_handler(httpd_req_t *req);
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data);

/*******************************************************************************/
/*                            STATIC VARIABLES                                 */
/*******************************************************************************/
static httpd_uri_t root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_handler,
    .user_ctx  = NULL
};

/*******************************************************************************/
/*                        STATIC FUNCTION DEFINITIONS                          */
/*******************************************************************************/
static esp_err_t root_handler(httpd_req_t *req)
{
    const char* html = "<html><head><title>ESP32 AP</title></head>"
                      "<body><h1>Hello World from ESP32!</h1>"
                      "<p>This is a simple HTTP server running on an ESP32 in Access Point mode.</p>"
                      "</body></html>";
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html, strlen(html));
    
    return ESP_OK;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        LOG("Station connected, AID=%d\n", event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        LOG("Station disconnected, AID=%d\n", event->aid);
    }
}

/*******************************************************************************/
/*                        GLOBAL FUNCTION DEFINITIONS                          */
/*******************************************************************************/
esp_err_t wifi_ap_init(wifi_ap_custom_config_t *config)
{
    esp_err_t ret = ESP_OK;
    
    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize TCP/IP adapter
    ESP_ERROR_CHECK(esp_netif_init());
    
    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Create WiFi AP
    esp_netif_create_default_wifi_ap();
    
    // Configure WiFi
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    
    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                       ESP_EVENT_ANY_ID,
                                                       &wifi_event_handler,
                                                       NULL,
                                                       NULL));
    
    // Configure AP
    wifi_config_t wifi_config = {0};
    
    // Set SSID and password
    strncpy((char*)wifi_config.ap.ssid, config->ssid, sizeof(wifi_config.ap.ssid) - 1);
    wifi_config.ap.ssid_len = strlen(config->ssid);
    
    if (config->password != NULL && strlen(config->password) > 0) {
        strncpy((char*)wifi_config.ap.password, config->password, sizeof(wifi_config.ap.password) - 1);
        wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    
    wifi_config.ap.max_connection = config->max_connections;
    wifi_config.ap.channel = config->channel;
    
    // Set WiFi mode and config
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    
    // Start WiFi
    ESP_ERROR_CHECK(esp_wifi_start());
    
    LOG("WiFi AP started with SSID: %s, channel: %d\n", config->ssid, config->channel);
    
    return ESP_OK;
}

esp_err_t http_server_start(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    
    LOG("Starting HTTP server on port: %d\n", config.server_port);
    
    if (httpd_start(&server, &config) == ESP_OK) {
        // Register URI handlers
        httpd_register_uri_handler(server, &root);
        return ESP_OK;
    }
    
    LOG("Error starting HTTP server!\n");
    return ESP_FAIL;
}

esp_err_t http_server_stop(void)
{
    if (server != NULL) {
        httpd_stop(server);
        server = NULL;
        return ESP_OK;
    }
    
    return ESP_ERR_INVALID_STATE;
}