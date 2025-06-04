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
#include "Common.h"
#include "GNSS_ublox.h"
#include "NaviLogging.h"

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
    LOG("Welcome to Wave Rover Navigation App!\n");

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

    /******************************* Initialize Modules ********************************/
    /* Initialize the GNSS module */
    if(GNSS_ublox_init() != ESP_OK) 
    {
        LOG("GNSS module initialization failed\n"); // for now just log it (TODO: handle it properly)
    }
    /* Initialize the NaviLogging module */
    if(NaviLogging_init() != ESP_OK) 
    {
        LOG("NaviLogging module initialization failed\n"); // for now just log it (TODO: handle it properly)
    }
    /***********************************************************************************/

    /***************************** Main Task Loop **************************************/
    while(1) 
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        LOG("Main task checking-in...\n");
    }
}