/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/**
 * ****************************************************************************
 * app_main.c
 *
 * Description:
 * This is the main application file for the Moduri application.
 * It serves as the entry point and handles system initialization.
 * 
 * The application performs the following tasks:
 * 1. Prints a welcome message
 * 2. Displays detailed chip information (CPU cores, features, revision)
 * 3. Shows flash memory size and type
 * 4. Reports minimum free heap size
 * 5. Initializes the SW components
 *
 * This file implements the main control flow for the Moduri application.
 * ****************************************************************************
 */

/*******************************************************************************/
/*                                INCLUDES                                     */
/*******************************************************************************/
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "CAN_HAL.h"
#include "J1939.h"
#include "WiFi_AP.h"
#include "Common.h"

/*******************************************************************************/
/*                                 MACROS                                      */
/*******************************************************************************/

/*******************************************************************************/
/*                               DATA TYPES                                    */
/*******************************************************************************/

/*******************************************************************************/
/*                        GLOBAL FUNCTION DECLARATIONS                         */
/*******************************************************************************/

/*******************************************************************************/
/*                        STATIC FUNCTION DECLARATIONS                         */
/*******************************************************************************/

/*******************************************************************************/
/*                            STATIC VARIABLES                                 */
/*******************************************************************************/

/*******************************************************************************/
/*                            GLOBAL VARIABLES                                 */
/*******************************************************************************/

/*******************************************************************************/
/*                        STATIC FUNCTION DEFINITIONS                          */
/*******************************************************************************/

/*******************************************************************************/
/*                        GLOBAL FUNCTION DEFINITIONS                          */
/*******************************************************************************/
 
/**
 * ****************************************************************************
 * Function: app_main
 * 
 * Description: Main application entry point. This function is called by the 
 *              ESP-IDF framework after initialization. It runs in the context
 *              of a default high-priority FreeRTOS main task which is created 
 *              by the ESP-IDF framework and runs your app_main() function.
 *              app_main() can be used to start other FreeRTOS tasks that your
 *              application requires.
 * 
 * Parameters:
 *   - none
 * 
 * Returns: void
 * ****************************************************************************
 */
void app_main(void)
{
    LOG("Welcome to Moduri Application!\n");

    /******************************* Platform Information *******************************/
    esp_chip_info_t chip_info;
    uint32_t flash_size;

    /* Get chip information */
    esp_chip_info(&chip_info);

    /* Print chip information */
    LOG("This is %s chip with %d CPU core(s), %s%s%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
            (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
            (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    LOG("silicon revision v%d.%d, ", major_rev, minor_rev);

    /* Get flash size */
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) 
    {
        LOG("Get flash size failed");
        return;
    }

    /* Print flash size */
    LOG("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    LOG("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
    /***********************************************************************************/

    /* Initialize the TWAI peripheral for CAN communication */
    (void)init_twai();

    // Configure WiFi AP (disabled for now because brownout resets when powering from USB. It has to be powered from a battery/power supply)
    /*
    wifi_ap_custom_config_t ap_config = {
        .ssid = "ESP32_AP",
        .password = "password123",  // NULL for open network
        .channel = 1,
        .max_connections = 4
    };
    
    // Initialize WiFi AP
    ESP_ERROR_CHECK(wifi_ap_init(&ap_config));
    
    // Start HTTP server
    ESP_ERROR_CHECK(http_server_start());

    // Now your ESP32 is running as an access point with an HTTP server
    // Connect to "ESP32_AP" WiFi network and open http://192.168.4.1 in a browser
    */

    /* Destination address for J1939 message */
    uint8_t dest_address = ESP32_2_SRC_ADDR;

    /* Node addresses in J1939 are typically assigned via the Address Claiming process (see J1939-81) */
    /* Here we are using static addresses for simplicity (TODO - Implement Address Claiming) */
    uint8_t source_address = ESP32_1_SRC_ADDR;

    /* Data payload for J1939 message (TODO - implement SPN structures for given PGNs since a PGN implies a certain data structure) */
    /* J1939 CAN ID is  used along with the Data Field to form a PDU (Protocol Data Unit)
       For messages with 8 bytes of data or less, the PDU fits in a single CAN frame.
       For messages with more than 8 bytes of data, the PDU is split into multiple frames using the J1939 Transport Protocol (TP) */
    /* Data Field contains the actual data of the chosen PGN. This data is identified by the corresponding SPN (Source Parameter Number)
        Example of Data Field for EEC1 (PGN 61444) from J1939-71 (2003):
        Bit Start Position/Bytes    Length    SPN Description                           SPN
        1.1                         4 bits    Engine Torque Mode                        899
        2                           1 byte    Driver Demand Engine - Percent Torque     512
        and so on... */
    uint8_t data_payload[8] = {0xFF, 0xFE, 0xFD, 0xFC, 0xFB, 0xFA, 0xF9, 0xF8}; // Example data payload (8 bytes)
    uint8_t data_length = sizeof(data_payload); // Length of the data payload

    /* Task loop */
    while(1) 
    {
        /* Send a J1939 message */
        esp_err_t send_result = send_J1939_message_by_pgn(ESP32_1_PGN, dest_address, source_address, data_payload, data_length);
        if (send_result != ESP_OK) 
        {
            LOG("Error sending J1939 message: %s", esp_err_to_name(send_result));
        } 
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        /* Receive a J1939 message */
        J1939_Message_t message;
        esp_err_t recv_result = receive_J1939_message(&message);
        if (recv_result == ESP_OK) 
        {
            LOG("Received J1939 message. Priority: %d, Data Page: %d, PDU Format: %d, PDU Specifics: %d, Source Address: %d, Data Length: %d\n",
                message.priority, message.data_page, message.pdu_format, message.pdu_specifics, message.src_address, message.data_length);
            LOG("Data: ");
            for (uint8_t i = 0; i < message.data_length; i++) 
            {
                LOG("0x%02X ", message.data[i]);
            }
            LOG("\n");
        } 
        else
        {
            LOG("Error receiving J1939 message: %s", esp_err_to_name(recv_result));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

        /* Monitor the CAN bus */
        bool monitor_result = monitor_CAN_bus();
        if (!monitor_result) 
        {
            LOG("Error monitoring CAN bus");
        }

    }
}