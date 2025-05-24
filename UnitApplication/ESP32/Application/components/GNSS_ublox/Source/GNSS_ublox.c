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

/* Macros related to NMEA sentences */
#define NMEA_MAX_SENTENCE_LENGTH 512 // Maximum length of a NMEA sentence (as per our assumption, can be adjusted later)
#define NMEA_MAX_SENTENCES 5 // Maximum number of NMEA sentences to store in the buffer

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

static void process_NMEA_sentence(char *sentence, int length);

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
    /* According to C.12 NMEA Protocol Settings (UBX-CFG-NMEA) in ublox M8 Receiver Description document,
       by default flags-limit82 is set to 0, which means the length of the NMEA sentences is not limited to 82 characters.
       For now I will allocate 512 bytes per NMEA sentence, but I may adjust this later based on the actual data received (TODO). */
    static char received_NMEA_sentences[NMEA_MAX_SENTENCES][NMEA_MAX_SENTENCE_LENGTH];
    static char temp_sentence_buffer[NMEA_MAX_SENTENCE_LENGTH]; // Temporary buffer for reading sentences
    static uint8_t current_NMEA_sentence_index = 0; // Index for the current NMEA sentence being processed
    static uint32_t bytes_in_the_temp_buffer = 0; // Number of bytes currently in the temporary buffer
    static bool unprocessed_data_in_the_buffer = false;

    ESP_LOGI(TAG, "GNSS UART RX task started.");

    /************************* Task Loop *************************/
    while (1) 
    {
        /* Read data from the UART RX buffer to the local buffer */
        int bytes_read = uart_read_bytes(UART_NUM, 
                                         temp_sentence_buffer + bytes_in_the_temp_buffer, // Read into the free space in the temporary buffer
                                         sizeof(temp_sentence_buffer) - bytes_in_the_temp_buffer - 1, // Don't read more than the remaining space in the buffer (-1 for /0)
                                         UART_RX_TIMEOUT_IN_TICKS); // Wait for up to UART_RX_TIMEOUT_IN_TICKS ticks for data to be available

        if (bytes_read > 0)
        {
            unprocessed_data_in_the_buffer = true; // Set the flag to indicate that data was received and needs processing
            
            bytes_in_the_temp_buffer += bytes_read; // Update the number of received bytes in the temporary buffer

            temp_sentence_buffer[bytes_in_the_temp_buffer] = '\0'; // Null-terminate the temporary buffer to treat it as a string
        }
        else if (bytes_read < 0) 
        {
            ESP_LOGE(TAG, "Error reading from UART: %d", bytes_read);
        }
        else /* bytes_read == 0 */
        {
            vTaskDelay(pdMS_TO_TICKS(100)); // Wait for 100 ms before trying to read again
        }

        while ((bytes_in_the_temp_buffer > 0) && (unprocessed_data_in_the_buffer)) 
        {   
            /* Check if the received data contains a complete NMEA sentence */
            /* According to the NMEA standard (and u-blox M8 Receiver description), sentences are terminated by a carriage return and line feed (CRLF) */
            char *end_of_sentence = strstr(temp_sentence_buffer, "\r\n"); 
            if (end_of_sentence != NULL)
            {
                end_of_sentence += 2; // Add 2 to account for CRLF
                /* Calculate the length of the complete NMEA sentence */
                int sentence_length = end_of_sentence - temp_sentence_buffer;

                /* Copy the complete NMEA sentence to the received sentences array */
                /* We don't check the sentence length here because we already limit the length in uart_read_bytes() */
                if (current_NMEA_sentence_index < NMEA_MAX_SENTENCES) 
                {
                    bytes_in_the_temp_buffer -= sentence_length; // Decrease bytes_in_the_temp_buffer by the length of the complete sentence

                    strncpy(received_NMEA_sentences[current_NMEA_sentence_index], temp_sentence_buffer, sentence_length);
                    received_NMEA_sentences[current_NMEA_sentence_index][sentence_length] = '\0'; // Null-terminate the string

                    ESP_LOGI(TAG, "Received NMEA Sentence: %s", received_NMEA_sentences[current_NMEA_sentence_index]);

                    /* Process the received NMEA sentence */
                    process_NMEA_sentence(received_NMEA_sentences[current_NMEA_sentence_index], sentence_length);

                    /* Move to the next index for the next NMEA sentence or reset if we reach the maximum */
                    current_NMEA_sentence_index++;
                    if (current_NMEA_sentence_index >= NMEA_MAX_SENTENCES) 
                    {
                        ESP_LOGW(TAG, "Maximum number of NMEA sentences reached. Resetting index.");
                        current_NMEA_sentence_index = 0; // Reset index if we reach the maximum number of sentences
                    }
                }
                else 
                {
                    /* This should never happen since in the if condition we reset the index if it reaches the maximum but just in case */
                    ESP_LOGW(TAG, "Received more NMEA sentences than expected. Resetting index -- this condition should not occur, investigate why it did.");
                    current_NMEA_sentence_index = 0; // Reset index if we reach the maximum number of sentences
                }

                /* Remove the processed sentence from the temporary buffer and shift the remaining data to the front (if any) */
                if (bytes_in_the_temp_buffer > 0) 
                {
                    memmove(temp_sentence_buffer, end_of_sentence, bytes_in_the_temp_buffer); // Shift remaining data to the front
                    temp_sentence_buffer[bytes_in_the_temp_buffer] = '\0'; // Null-terminate the remaining data
                }
                else 
                {
                    /* There is no more data to process in the temporary buffer, loop will exit to wait for more data */
                    unprocessed_data_in_the_buffer = false;
                    temp_sentence_buffer[0] = '\0';
                }
            }
            else 
            {
                ESP_LOGD(TAG, "Partial NMEA sentence received: %s", temp_sentence_buffer);
                /* If we reach here, it means we have not found a complete NMEA sentence yet */
                /* The loop will exit and wait for more data to be received to gather a complete sentence */
                unprocessed_data_in_the_buffer = false;
            }
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

static void process_NMEA_sentence(char *sentence, int length) 
{
    /* Process different types of NMEA sentences */
    process_GGA_sentence(sentence);
    // Add more processing functions for other NMEA sentence types as needed (TODO)
}

static void process_GGA_sentence(char *sentence) 
{
    /* Copy the NMEA sentence to a new buffer - this is needed because strtok modifies the input string */
    char gga_sentence[NMEA_MAX_SENTENCE_LENGTH];
    strncpy(gga_sentence, sentence, NMEA_MAX_SENTENCE_LENGTH - 1); // -1 to leave space for null-terminator
    gga_sentence[NMEA_MAX_SENTENCE_LENGTH - 1] = '\0'; // Ensure null-termination

    /* Check for the GGA message type */
    char* gga_start = strstr(gga_sentence, "$GNGGA"); // Check for the GN first (GNA is any combination of GNSS systems, including GPS, GLONASS, Galileo, and BeiDou)
    if (gga_start == NULL) 
    {
        gga_start = strstr(gga_sentence, "$GAGGA"); // Check for Galileo second (the European GNSS system)
        if(gga_start == NULL) 
        {
            gga_start = strstr(gga_sentence, "$GLGGA"); // Check for GLONASS third (the Russian GNSS system)
            if(gga_start == NULL) 
            {
                return; // If no GGA message found, exit the function
            }
        }
    }

    /* Parse the GGA message if found */
    if (gga_start != NULL)
    {
        char *current_token;
        int field_idx = 0; // 0 for message ID, 1 for Time, 2 for Latitude, etc. (see ublox M8 Receiver Description document for GGA fields)

        /** strtok will replace the first comma it finds with a null terminator. It then returns pointer to the beginning of the created token.
         *  Subsequent calls to strtok with NULL will continue tokenizing the same string, using the same delimiter (comma in this case).
         *  The first token is the message ID itself (e.g., "$GNGGA"). 
         *  Example of GGA sentence so you can imagine the tokens: 
         *  $GNGGA,120337.00,5138.14222,N,01757.96508,E,2,12,0.78,148.9,M,38.6,M,,0000*4E 
         */
        current_token = strtok(gga_start, ","); 

        /* Loop through the tokens in the GGA sentence */
        while (current_token != NULL && field_idx < 14) // 14 is the maximum number of fields in a GGA sentence (not counting checksum and CRLF)
        {
            /* Get the next token and check if it's valid for processing */
            current_token = strtok(NULL, ","); // Find the next token/field in the GGA sentence
            if (current_token == NULL || *current_token == '*') // No more tokens or the token is a checksum (starts with '*')             
            {
                break; // All tokens have been processed or we reached the checksum, exit the loop
            }
            field_idx++; // Field contains data, count it

            /* Process the current token based on its field index */
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
                case 3: // Field 3: N/S Indicator - This field is already handled in the case for Field 2 (Latitude)
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
                case 6: // Field 6: Quality Indicator - TODO
                    break;
                case 7: // Field 7: Number of Satellites - TODO
                    break;
                case 8: // Field 8: HDOP - TODO
                    break;
                case 9: // Field 9: Altitude (Meters)
                    if (strlen(current_token) > 0)
                    {
                        Altitude = atof(current_token);
                        ESP_LOGI(TAG, "Altitude: %f", Altitude);
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