/******************************************************************************
 * @file GNSS_ublox.c
 * @brief Template component for ESP-IDF projects
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
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Project Includes */
#include "GNSS_ublox.h"
#include "Common.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/
/** Tag for logging (used in ESP_LOGI, ESP_LOGE, etc.) Example usage: 
 *  ESP_LOGI(TAG, "Log message which will be appended to the tag"); */
#define TAG "GNSS_UBLOX"

/* UART configuration */
#define UART_NUM UART_NUM_2
#define UART_TX_PIN GPIO_NUM_17
#define UART_RX_PIN GPIO_NUM_16
#define UART_BAUD_RATE 9600
#define UART_RX_BUF_SIZE 1024
#define UART_TX_BUF_SIZE 0
#define UART_EVENT_QUEUE_SIZE 0
#define UART_INTERRUPT_ALLOC_FLAGS 0

/* UART RX task configuration */
#define UART_RX_TASK_STACK_SIZE 2048 // in bytes (not words like in "normal" FreeRTOS)
#define UART_RX_TASK_PRIORITY (tskIDLE_PRIORITY + 1)
#define UART_RX_TIMEOUT_IN_TICKS pdMS_TO_TICKS(20) // 20 ms timeout for UART read

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
 * @brief Task to handle incoming UART data from the GNSS module.
 *
 * @param pvParameters Parameters passed to the task (not used).
 */
static void uart_rx_task(void *pvParameters);

static double convert_nmea_to_decimal_degrees(const char* nmea_coord, char indicator);

static void process_NMEA_sentence(uint8_t *data, int length);

static void process_GGA_sentence(char *sentence);

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/
/* Static variables for extracted parameters from NMEA sentences */
static double Latitude = 0.0;
static double Longitude = 0.0;
static double Altitude = 0.0;

/*******************************************************************************/
/*                     GLOBAL FUNCTION DEFINITIONS                             */
/*******************************************************************************/

esp_err_t GNSS_ublox_init(void)
{
    /* Define the UART configuration according to the ublox module's requirements */
    uart_config_t uart_config = 
    {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // We don't have the extra lines for RTS/CTS
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* Apply the defined UART configuration */
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));

    /* Set UART pins (TX, RX, RTS, CTS) and the UART port number according to our physical connections */
    /* Note: RTS and CTS are not used in this case, so we set them to UART_PIN_NO_CHANGE */
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    /* "Installing" the driver involves allocating necessary memory for the driver's internal structures (like RX/TX buffers and 
        an event queue if specified), initializing these structures, setting up the UART hardware (enabling the module and 
        configuring interrupts), and preparing it for communication. */
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, UART_EVENT_QUEUE_SIZE, NULL, UART_INTERRUPT_ALLOC_FLAGS));

    /* Create a task to handle UART data from the GNSS module */
    BaseType_t status = xTaskCreate(uart_rx_task, "uart_rx_task", UART_RX_TASK_STACK_SIZE, NULL, UART_RX_TASK_PRIORITY, NULL);
    if (status != pdPASS) 
    {
        ESP_LOGE(TAG, "Failed to create GNSS UART RX task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "GNSS_ublox_init completed and UART RX task started.");

    /* If we reach here, the initialization was successful (otherwise ESP_ERROR_CHECK logs the error and aborts or we return with ESP_FAIL) */
    return ESP_OK; 
}

/*******************************************************************************/
/*                     STATIC FUNCTION DEFINITIONS                             */
/*******************************************************************************/

static void uart_rx_task(void *pvParameters)
{
    /******************** Task Initialization ********************/
    static uint8_t received_data[UART_RX_BUF_SIZE]; // reuse the buffer size of rx ring buffer but IT IS NOT the same buffer

    ESP_LOGI(TAG, "GNSS UART RX task started.");

    /************************* Task Loop *************************/
    while (1) 
    {
        /* Read data from the UART RX buffer to the local buffer */
        int bytes_read = uart_read_bytes(UART_NUM, received_data, (UART_RX_BUF_SIZE - 1), UART_RX_TIMEOUT_IN_TICKS);

        if (bytes_read > 0) 
        {
            received_data[bytes_read] = '\0'; // Null-terminate the received data
            
            LOG("Received data from ublox module: \n\n %s\n\n", received_data);
            
            process_NMEA_sentence(received_data, bytes_read); // Process the received NMEA sentence
        }
    }

    vTaskDelete(NULL); // should never reach here but just in case
}

// Helper function to convert NMEA latitude/longitude format (DDMM.MMMMM) to decimal degrees
static double convert_nmea_to_decimal_degrees(const char* nmea_coord, char indicator) 
{
    double degrees;
    double minutes;
    char* dot = strchr(nmea_coord, '.');
    if (dot == NULL) 
    {
        return 0.0; // Invalid format
    }

    // Extract degrees (before MM.MMMMM)
    char deg_str[4]; // Max 3 digits for degrees (DDD)
    strncpy(deg_str, nmea_coord, dot - nmea_coord - 2);
    deg_str[dot - nmea_coord - 2] = '\0';
    degrees = atof(deg_str);

    // Extract minutes (MM.MMMMM)
    minutes = atof(dot - 2);

    double decimal_degrees = degrees + (minutes / 60.0);

    // Apply sign based on indicator (N/S for latitude, E/W for longitude)
    if (indicator == 'S' || indicator == 'W') 
    {
        decimal_degrees *= -1.0;
    }
    return decimal_degrees;
}

static void process_NMEA_sentence(uint8_t *data, int length) 
{
    /* Cast the uint8_t data to char for string manipulation (should be safe regardless the sign of char, regular character set ends at 127 anyway) */
    char *sentence = (char *)data;

    process_GGA_sentence(sentence);


}

static void process_GGA_sentence(char *sentence) 
{
    /* Check for the GGA message type */
    char* gga_start = strstr(sentence, "$GNGGA"); // Check for the GN first (GNA is any combination of GNSS systems, including GPS, GLONASS, Galileo, and BeiDou)
    if (gga_start == NULL) 
    {
        gga_start = strstr(sentence, "$GAGGA"); // Check for Galileo second (the European GNSS system)
        if(gga_start == NULL) 
        {
            gga_start = strstr(sentence, "$GLGGA"); // Check for GLONASS third (the Russian GNSS system)
            if(gga_start == NULL) 
            {
                ESP_LOGI(TAG, "No GGA message found in the received data."); 
            }
        }
    }

    /* Parse the GGA message if found */
    if (gga_start != NULL)
    {
        ESP_LOGI(TAG, "GGA message found: %s", gga_start);

        char *current_token;
        int field_idx = 0; // 0 for message ID, 1 for Time, 2 for Latitude, etc.

        // The first token is the message ID itself (e.g., "$GNGGA")
        current_token = strtok(gga_start, ",");

        // Loop through subsequent data fields
        // GGA has up to 14 data fields after the ID (Fields 1-14)
        while (current_token != NULL && field_idx < 14)
        {
            current_token = strtok(NULL, ","); // Get the next data field
            if (current_token == NULL || *current_token == '*') // End of sentence (checksum) or no more tokens
            {
                break;
            }
            field_idx++; // Increment for the current data field index

            switch (field_idx)
            {
                case 1: // Field 1: Time (hhmmss.ss) - TODO
                    // ESP_LOGI(TAG, "Time: %s", current_token);
                    break;
                case 2: // Field 2: Latitude (DDMM.MMMMM)
                    if (strlen(current_token) > 0)
                    {
                        char *indicator_token = strtok(NULL, ","); // This should be Field 3: N/S Indicator
                        if (indicator_token != NULL && strlen(indicator_token) > 0 && *indicator_token != '*')
                        {
                            Latitude = convert_nmea_to_decimal_degrees(current_token, indicator_token[0]);
                            ESP_LOGI(TAG, "Latitude: %f", Latitude);
                            field_idx++; // Consumed indicator token (Field 3)
                            current_token = indicator_token; // Update current_token to the last processed token for the while loop condition
                        }
                        else
                        {
                            ESP_LOGW(TAG, "Latitude N/S indicator missing or invalid.");
                            current_token = NULL; // Stop processing due to malformed sentence
                        }
                    }
                    break;
                case 3: // Field 3: N/S Indicator
                    // This field is already handled in the case for Field 2 (Latitude)
                    break;
                case 4: // Field 4: Longitude (DDDMM.MMMMM)
                    if (strlen(current_token) > 0)
                    {
                        char *indicator_token = strtok(NULL, ","); // This should be Field 5: E/W Indicator
                        if (indicator_token != NULL && strlen(indicator_token) > 0 && *indicator_token != '*')
                        {
                            Longitude = convert_nmea_to_decimal_degrees(current_token, indicator_token[0]);
                            ESP_LOGI(TAG, "Longitude: %f", Longitude);
                            field_idx++; // Consumed indicator token (Field 5)
                            current_token = indicator_token; // Update current_token
                        }
                        else
                        {
                            ESP_LOGW(TAG, "Longitude E/W indicator missing or invalid.");
                            current_token = NULL; // Stop processing
                        }
                    }
                    break;
                case 6: // Field 6: Quality Indicator
                    // if (strlen(current_token) > 0) { FixQuality = atoi(current_token); }
                    break;
                case 7: // Field 7: Number of Satellites
                    // if (strlen(current_token) > 0) { SatellitesUsed = atoi(current_token); }
                    break;
                case 8: // Field 8: HDOP
                    // if (strlen(current_token) > 0) { HDOP = atof(current_token); }
                    break;
                case 9: // Field 9: Altitude (Meters)
                    if (strlen(current_token) > 0)
                    {
                        Altitude = atof(current_token);
                        ESP_LOGI(TAG, "Altitude: %f", Altitude);
                        // Note: Field 10 is Altitude Unit (M), which is consumed by the next strtok call
                        // at the beginning of the loop if not explicitly handled here.
                    }
                    break;
                case 10: // Field 10: Altitude Unit (M) - This field is not needed for processing, just skip it
                    break;
                case 11: // Field 11: Geoidal Separation - TODO
                    break;
                case 12: // Field 12: Geoidal Separation Unit (M) - This field is not needed for processing, just skip it
                    break;
                case 13: // Field 13: Age of Diff. Corr. (seconds) - TODO
                    break;
                case 14: // Field 14: Diff. Ref. Station ID - TODO
                    break;

                default:
                    // For other fields, we just let strtok advance to the next one
                    break;
            }
        }
    }

}