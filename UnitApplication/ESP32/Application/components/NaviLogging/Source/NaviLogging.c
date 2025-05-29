/******************************************************************************
 * @file NaviLogging.c
 * @brief Component for logging navigation data and sending it via ESP-NOW.
 * 
 ******************************************************************************/

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
/*    Include headers required for the definitions/implementation in *this*    */
/* source file. This typically includes this module's own header("template.h") */
/*      and any headers needed for function bodies, static variables, etc.     */
/*******************************************************************************/
/* ESP-IDF Includes */
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "string.h"
#include "Common.h"

/* Project Includes */
#include "NaviLogging.h"
#include "GNSS_ublox.h"
/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/
/** Tag for logging (used in ESP_LOGI, ESP_LOGE, etc.) Example usage: 
 *  ESP_LOGI(TAG, "Log message which will be appended to the tag"); */
#define TAG "NAVILOGGING"

/* NaviLogging Task Configuration */
#define NAVILOGGING_TASK_STACK_SIZE 4096
#define NAVILOGGING_TASK_PRIORITY (tskIDLE_PRIORITY + 2)
#define NAVILOGGING_SEND_INTERVAL_MS 1000 // Send data every 5 seconds

/* ESP-NOW Configuration */
#define ESPNOW_WIFI_CHANNEL 9 // Fixed channel for ESP-NOW communication (BASED ON WAVE ROVER DRIVER - CHECK IT ON THAT SIDE AND THEN SET THE SAME CHANNEL HERE)

/*******************************************************************************/
/*                                DATA TYPES                                   */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL FUNCTION DECLARATIONS                            */
/*******************************************************************************/
/*  for function defined in some other .c files, to be used here. Use extern.  */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL VARIABLES DECLARATIONS                           */
/*******************************************************************************/
/*  for variables defined in some other .c files, to be used here. Use extern. */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL VARIABLES DEFINITIONS                            */
/*******************************************************************************/
/*    for variables defined in this .c file, to be used in other .c files.     */
/*******************************************************************************/

/*******************************************************************************/
/*                     STATIC FUNCTION DECLARATIONS                            */
/*******************************************************************************/

/**
 * @brief Task to periodically fetch coordinates and send them via ESP-NOW.
 * 
 * This task runs in a loop, fetching coordinates from the GNSS_ublox component
 * and sending them via ESP-NOW every defined interval.
 * 
 * @param pvParameters Pointer to task parameters (not used).
 * 
 * @return void
 */
static void navi_logging_task(void *pvParameters);

/**
 * @brief Callback function for ESP-NOW send status.
 * 
 * This function is called when an ESP-NOW message is sent, providing the MAC address
 * of the peer and the status of the send operation.
 * 
 * @param mac_addr Pointer to the MAC address of the peer.
 * @param status Status of the send operation (success or failure).
 */
static void esp_now_send_callback(const uint8_t *mac_addr, esp_now_send_status_t status);

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/
/**
 * MAC address of the ESP-NOW peer device.
 * 
 * In ESP-NOW, a "peer" is another ESP32 device that this device will communicate with.
 * To send data to a specific ESP32, you need to know its unique MAC address.
 * 
 * How to find the MAC address of an ESP32 - just print it out on the given device (but AFTER WiFi is initialized):
        #include <esp_wifi.h>
 
        uint8_t mac_addr[6];
        esp_wifi_get_mac(ESP_IF_WIFI_STA, mac_addr);
        ESP_LOGI("MAC_INFO", "My MAC Address is: %02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
 
 * Then copy that MAC address and use it to initialize the `peer_mac` array below.
 * For example, if the MAC is AA:BB:CC:11:22:33, then:
 * static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33};
 *
 * Value {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} is the broadcast address,
 * which sends data to all ESP-NOW devices in range. For unicast (one-to-one)
 * communication, replace it with the specific MAC address of the receiver.
 */
static uint8_t receiver_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast address

/*******************************************************************************/
/*                     GLOBAL FUNCTION DEFINITIONS                             */
/*******************************************************************************/

esp_err_t NaviLogging_init(void) 
{
    ESP_LOGI(TAG, "Initializing NaviLogging component");

    /* Initialize NVS (Non-Volatile Storage) - needed for Wi-Fi and ESP-NOW */
    esp_err_t status_nvs = nvs_flash_init();
    if (status_nvs == ESP_ERR_NVS_NO_FREE_PAGES || status_nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        ESP_ERROR_CHECK(nvs_flash_erase()); // Erase NVS if it has no free pages or a new version is found
        status_nvs = nvs_flash_init(); // Try to initialize NVS again
    }
    ESP_ERROR_CHECK(status_nvs);

    /* Initialize the network interface (the underlying LwIP TCP/IP stack and esp-netif synchronization primitives) */
    ESP_ERROR_CHECK(esp_netif_init());
    
    /* Create the default event loop for handling events */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Initialize Wi-Fi in station mode with default configuration */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* IMPORTANT: Set the Wi-Fi channel for ESP-NOW (this must match the channel used by the receiver) */
    /* If you connect to a Wi-Fi network the channel is dictated by the network, but since here we are not connecting to any Wi-Fi network,
       we set a fixed channel for ESP-NOW communication matching the channel on wave rover driver */
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));

    /* Add a small delay to ensure Wi-Fi is fully initialized before proceeding */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Print the MAC address of the station interface - this is useful for other devices to know where to send data or who to receive data from */
    uint8_t mac_addr[6];
    (void)esp_wifi_get_mac(ESP_IF_WIFI_STA, mac_addr); // Get the MAC address of the station interface (for ESP-NOW)
    LOG("My MAC Address is: %02X:%02X:%02X:%02X:%02X:%02X \n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

    /* Initialize ESP-NOW. This function is not open-source so we don't know the details of it but it is necessary to set up ESP-NOW communication. */
    ESP_ERROR_CHECK(esp_now_init());

    /* Register the ESP-NOW send callback function. This function will be called whenever data is sent via ESP-NOW. (either on success or failure) */
    ESP_ERROR_CHECK(esp_now_register_send_cb(esp_now_send_callback));

    /* Print the current Wi-Fi channel. This is useful for debugging and ensuring that the ESP-NOW communication is set up correctly. */
    uint8_t primary_channel;
    wifi_second_chan_t second_channel;
    if (esp_wifi_get_channel(&primary_channel, &second_channel) == ESP_OK) 
    {
        ESP_LOGI(TAG, "Current Wi-Fi Primary Channel: %d", primary_channel);
    } 
    else 
    {
        ESP_LOGE(TAG, "Failed to get Wi-Fi channel");
    }

    /* Configure the ESP-NOW peer. This is the device that will receive the data sent by this device. */
    static esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(esp_now_peer_info_t));
    /*
     * Set the ESP-NOW peer channel.
     * A value of 0 here is a special instruction for ESP-NOW: it means that
     * ESP-NOW should use the current operational Wi-Fi channel of the interface
     * specified by 'peer_info.ifidx' (which is WIFI_IF_STA in this case).
     *
     * Since we set the Wi-Fi channel earlier with esp_wifi_set_channel(),
     * ESP-NOW will use that channel for communication.
     * You can verify the actual channel being used by checking the output of
     * esp_wifi_get_channel() earlier in this initialization function.
     *
     * IMPORTANT: Any other ESP32 devices that need to communicate
     * with this device via ESP-NOW MUST be configured to operate on this
     * exact same Wi-Fi channel.
     */
    peer.channel = 0;
    /* 
     * Set the interface type for the peer.
     * This should match the interface type used by the sender device.
     * For ESP-NOW, this is typically WIFI_IF_STA (Station mode).
     * If you are using a different interface (like WIFI_IF_AP for Soft-AP),
     * make sure to change this accordingly.
     */
    peer.ifidx = ESP_IF_WIFI_STA;
    /* 
     * Set the local master key (LMK) for the peer.
     * This is used for encryption. If you are not using encryption,
     * you can leave this as all zeros or set it to a known value.
     * For simplicity, we are not using encryption in this example.
     */
    memset(peer.lmk, 0, ESP_NOW_KEY_LEN); // No encryption, so LMK is set to all zeros
    /* 
     * Set the encryption flag.
     * If you are not using encryption, set this to false.
     * If you want to use encryption, set it to true and provide a valid LMK.
     */
    peer.encrypt = false;

    /* Finally, add the configured peer to the ESP-NOW peer list.
     * This allows the ESP-NOW stack to recognize this peer and send/receive data to/from it. */
    memcpy(peer.peer_addr, receiver_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    /* Create the NaviLogging task that will handle fetching coordinates and sending them via ESP-NOW.
     * The task will run in the background, periodically fetching coordinates and sending them to the configured peer at the defined interval. */
    BaseType_t status_task = xTaskCreate(navi_logging_task, "navi_logging_task", NAVILOGGING_TASK_STACK_SIZE, NULL, NAVILOGGING_TASK_PRIORITY, NULL);
    if (status_task != pdPASS) 
    {
        ESP_LOGE(TAG, "Failed to create NaviLogging task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "NaviLogging initialized successfully");
    return ESP_OK;
}

/*******************************************************************************/
/*                     STATIC FUNCTION DEFINITIONS                             */
/*******************************************************************************/

static void esp_now_send_callback(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
    if (status == ESP_NOW_SEND_SUCCESS) 
    {
        ESP_LOGI(TAG, "ESP-NOW send success to %02X:%02X:%02X:%02X:%02X:%02X",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    } 
    else 
    {
        ESP_LOGE(TAG, "ESP-NOW send failed to %02X:%02X:%02X:%02X:%02X:%02X",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    }
}

static void navi_logging_task(void *pvParameters) 
{
    /******************** Task Initialization ********************/
    navi_coordinates_type coordinates;
    esp_err_t status_ublox;

    ESP_LOGI(TAG, "NaviLogging task started");

    /************************* Task Loop *************************/
    while (1) 
    {
        /* Fetch coordinates from the GNSS_ublox component.
         * This function will fill the coordinates structure with latitude, longitude, and altitude.
         * If it fails, it will return an error code which we can log. */
        status_ublox = GNSS_ublox_get_coordinates(&coordinates.latitude, &coordinates.longitude, &coordinates.altitude);

        if (status_ublox == ESP_OK) 
        {
            esp_err_t send_result = esp_now_send(receiver_mac, (uint8_t *)&coordinates, sizeof(navi_coordinates_type));
            if (send_result != ESP_OK) 
            {
                ESP_LOGE(TAG, "ESP-NOW send error: %s", esp_err_to_name(send_result));
            }
        } 
        else 
        {
            ESP_LOGE(TAG, "Failed to get coordinates: %s", esp_err_to_name(status_ublox));
        }

        /* Delay for the defined interval before fetching and sending coordinates again */
        vTaskDelay(pdMS_TO_TICKS(NAVILOGGING_SEND_INTERVAL_MS));
    }
}
