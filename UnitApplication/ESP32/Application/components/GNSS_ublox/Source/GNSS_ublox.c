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
#define UART_RX_TASK_STACK_SIZE 4096 // in bytes (not words like in "normal" FreeRTOS)
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

/**
 * @brief Converts NMEA coordinates to decimal degrees.
 *
 * @param nmea_coord Pointer to the NMEA coordinate string (e.g., "1234.5678").
 * @param indicator 'N' or 'S' for latitude, 'E' or 'W' for longitude.
 * @return The converted coordinate in decimal degrees.
 * 
 * Notes: - The NMEA standard coordinate format is in Degrees, Minutes, and Decimal Minutes (DMM) 
 *        - According to the u-blox M8 Receiver Description, the format is as follows:
 *              - Latitude:  "ddmm.mmmm" (degrees, minutes, and decimal minutes)
 *              - Longitude: "dddmm.mmmm" (degrees, minutes, and decimal minutes)
 *         - To convert to Degrees and Fractions of Degrees, or Degrees, Minutes, Seconds and Fractions of seconds, 
 *           the 'Minutes' and 'Fractional Minutes' parts need to be converted. 
 *           In other words: 
 *              If the GPS Receiver reports a Latitude of 4717.112671 North and Longitude of 00833.914843 East, this is:
 *                  - Latitude 47 Degrees, 17.112671 Minutes
 *                  - Longitude 8 Degrees, 33.914843 Minutes
 *              or:
 *                  - Latitude 47 Degrees, 17 Minutes, 6.76026 Seconds
 *                  - Longitude 8 Degrees, 33 Minutes, 54.89058 Seconds
 *              or:
 *                  - Latitude 47.28521118 Degrees
 *                  - Longitude 8.56524738 Degrees
 */
static double convert_coordinates_to_decimal_degrees(const char* nmea_coord, char indicator);

static void process_NMEA_sentence(char *sentence, int length);

static void process_GGA_sentence(char *sentence);

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/
/* Static variables for extracted parameters from NMEA sentences */
static double Latitude = 0.0;
static double Longitude = 0.0;
static double Altitude = 0.0;
static double Time_UTC = 0.0;
static int QualityIndicator = 0;
static int NumberOfSatellites = 0;
static double HorizontalDilutionOfPrecision = 0.0;
static double GeoidalSeparation = 0.0;
static double AgeOfDifferentialGPSData = 0.0;
static int DifferentialReferenceStationID = 0;

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

esp_err_t GNSS_ublox_get_coordinates(double *latitude, double *longitude, double *altitude)
{
    /* Check if the pointers are valid */
    if (latitude == NULL || longitude == NULL || altitude == NULL) 
    {
        ESP_LOGE(TAG, "Invalid pointers provided to GNSS_ublox_get_coordinates");
        return ESP_ERR_INVALID_ARG; // Return error if any pointer is NULL
    }

    /* Return the extracted coordinates */
    *latitude = Latitude;
    *longitude = Longitude;
    *altitude = Altitude;

    ESP_LOGI(TAG, "Coordinates retrieved: Lat: %.6f, Lon: %.6f, Alt: %.2f (Quality: %d, Sats: %d)", 
             Latitude, Longitude, Altitude, QualityIndicator, NumberOfSatellites);

    return ESP_OK; // Return success
}

esp_err_t GNSS_ublox_get_quality_info(int *quality_indicator, int *num_satellites, double *hdop)
{
    /* Check if the pointers are valid */
    if (quality_indicator == NULL || num_satellites == NULL || hdop == NULL) 
    {
        ESP_LOGE(TAG, "Invalid pointers provided to GNSS_ublox_get_quality_info");
        return ESP_ERR_INVALID_ARG; // Return error if any pointer is NULL
    }

    /* Return the extracted GPS quality information */
    *quality_indicator = QualityIndicator;
    *num_satellites = NumberOfSatellites;
    *hdop = HorizontalDilutionOfPrecision;

    ESP_LOGI(TAG, "GPS quality info retrieved: Quality: %d, Satellites: %d, HDOP: %.2f", 
             QualityIndicator, NumberOfSatellites, HorizontalDilutionOfPrecision);

    return ESP_OK; // Return success
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

static double convert_coordinates_to_decimal_degrees(const char* nmea_coord, char indicator) 
{
    /* Validate the input NMEA coordinate and indicator */
    if (!nmea_coord || strlen(nmea_coord) < 5 || 
        (indicator != 'N' && indicator != 'S' && indicator != 'E' && indicator != 'W')) 
    {
        ESP_LOGE(TAG, "Invalid NMEA coordinate or indicator: %s, %c", nmea_coord, indicator);
        return 0.0; // Invalid input or indicator
    }

    /* Locate the dot in the NMEA coordinate string. */
    /* The dot is expected to be present and in correct position for the format "DDMM.MMMM" or "DDDMM.MMMM" */
    char* dot = strchr(nmea_coord, '.');
    if (dot == NULL) 
    {
        ESP_LOGE(TAG, "Invalid NMEA coordinate format (no decimal point): %s", nmea_coord);
        return 0.0; // Invalid format (no decimal point)
    }
    int chars_before_dot = dot - nmea_coord;
    if ((indicator == 'N' || indicator == 'S') && chars_before_dot != 4) 
    {
        ESP_LOGE(TAG, "Invalid NMEA coordinate format for latitude (expected 4 characters before dot, got %d): %s", chars_before_dot, nmea_coord);
        return 0.0; // Latitude must have exactly 4 characters before dot (ddmm)
    }
    if ((indicator == 'E' || indicator == 'W') && chars_before_dot != 5) 
    {
        ESP_LOGE(TAG, "Invalid NMEA coordinate format for longitude (expected 5 characters before dot, got %d): %s", chars_before_dot, nmea_coord);
        return 0.0; // Longitude must have exactly 5 characters before dot (dddmm)
    }

    /* Extract degrees */
    char deg_str[4]; // Buffer to hold the degrees part of the coordinate. Enough for 3 digits + null terminator
    int deg_len = 0; // Degrees are 2 characters for latitude (dd) or 3 characters for longitude (ddd) before the dot
    if (indicator == 'N' || indicator == 'S') 
    {
        deg_len = 2; // Latitude: 2 digits
    } 
    else if (indicator == 'E' || indicator == 'W') 
    {
        deg_len = 3; // Longitude: 3 digits
    }
    strncpy(deg_str, nmea_coord, deg_len); // Copy the degrees part from the NMEA coordinate
    deg_str[deg_len] = '\0';
    char* endptr;
    double degrees = strtod(deg_str, &endptr); // strtod converts the string to double and sets endptr to the first character after the number (a bit more advanced than atof)
    if ((*endptr != '\0') || (degrees < 0) || 
        ((indicator == 'N' || indicator == 'S') && degrees > 90) || 
        ((indicator == 'E' || indicator == 'W') && degrees > 180)) 
    {
        ESP_LOGE(TAG, "Invalid degrees in NMEA coordinate: %s, indicator: %c", nmea_coord, indicator);
        return 0.0; // Invalid degrees
    }

    /* Extract minutes */
    /* Minutes always start 2 characters before the dot, in both latitude (DDMM.MMMM) and longitude (DDDMM.MMMM) formats so it's easier than degrees */
    /* And yes, we do include the dot since we are extracting both minutes and fractional minutes */
    double minutes = strtod(dot - 2, &endptr); 
    if ((*endptr != '\0') || (minutes < 0) || (minutes >= 60)) 
    {
        ESP_LOGE(TAG, "Invalid minutes in NMEA coordinate: %s, indicator: %c", nmea_coord, indicator);
        return 0.0; // Invalid minutes
    }

    /* Compose the decimal degrees using the extracted degrees and minutes */
    double decimal_degrees = degrees + (minutes / 60.0);

    /* Adjust the sign based on the indicator */
    if (indicator == 'S' || indicator == 'W') 
    {
        decimal_degrees *= -1.0;
    }

    /* Return the converted coordinate in decimal degrees */
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
        /* Temporary variables for parsing */
        char *current_token;
        int field_idx = 0; // 0 for message ID, 1 for Time, 2 for Latitude, etc. (see ublox M8 Receiver Description document for GGA fields)
        char latitude_value_str[32] = {0}; // Buffer to hold latitude value string (DDMM.MMMMM format)
        char longitude_value_str[32] = {0}; // Buffer to hold longitude value string (DDDMM.MMMMM format)

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

            /* Check if the current token is empty. In NMEA sentences, some fields can be empty (e.g., no fix available) 
             * In a raw NMEA sentence, this would be represented as commas with no data between them (e.g., "123456.00,,N,01234.56789,E,...") */
            if(strlen(current_token) == 0) 
            {
                continue; // Skip empty fields
            }

            /* Process the current token based on its field index */
            switch (field_idx)
            {
                case 1: // Field 1: Time (hhmmss.ss)
                    Time_UTC = atof(current_token); // Convert time to double
                    ESP_LOGI(TAG, "Time (UTC) hhmmss.ss: %f", Time_UTC);
                    break;

                case 2: // Field 2: Latitude (DDMM.MMMMM)
                    strncpy(latitude_value_str, current_token, sizeof(latitude_value_str) - 1); // Copy latitude value to the buffer
                    latitude_value_str[sizeof(latitude_value_str) - 1] = '\0'; // Ensure null-termination
                    break;

                case 3: // Field 3: N/S Indicator
                    char indicator = current_token[0]; // Get the N/S indicator (N or S)
                    ESP_LOGI(TAG, "N/S Indicator: %c", indicator); // Log the N/S indicator for debugging
                    Latitude = convert_coordinates_to_decimal_degrees(latitude_value_str, indicator);
                    ESP_LOGI(TAG, "Latitude [degrees]: %f", Latitude);
                    break;

                case 4: // Field 4: Longitude (DDDMM.MMMMM)
                    strncpy(longitude_value_str, current_token, sizeof(longitude_value_str) - 1); // Copy longitude value to the buffer
                    longitude_value_str[sizeof(longitude_value_str) - 1] = '\0'; // Ensure null-termination
                    break;

                case 5: // Field 5: E/W Indicator
                    indicator = current_token[0]; // Get the E/W indicator (E or W)
                    ESP_LOGI(TAG, "E/W Indicator: %c", indicator);
                    Longitude = convert_coordinates_to_decimal_degrees(longitude_value_str, indicator);
                    ESP_LOGI(TAG, "Longitude [degrees]: %f", Longitude);
                    break;

                case 6: // Field 6: Quality Indicator
                    /* Quality Indicator is a number that indicates the quality of the GPS fix.
                       As per Flags in NMEA 4.10 and above, the values for GGA Quality Indicator are:
                            - 0:    No position fix (at power-ip or after losing satellite lock)
                            - 0:    Also possible with GNSS fix, but user limits exceeded
                            - 1/2:  2D or 3D GNSS fix (2D fix means at least 3 satellites, 3D fix means at least 4 satellites)
                            - 1/2:  Also possible with Combined GNSS or Dead Reckoning (DR) fix
                            - 4:    RTK Fixed (Real-Time Kinematic) fix. RTK works by using two GNSS receivers: a base station at 
                                        a precisely known, fixed location, and a rover receiver (mobile unit). It provides centimeter-level
                                        accuracy (much better than DGNSS) by resolving the integer ambiguities of the carrier phase measurements 
                                        of the satellite signals in real-time, using correction data transmitted from the base station to the rover.
                            - 5:    RTK Float fix
                            - 6:    Dead Reckoning (DR) fix. Dead Reckoning is a navigation method that estimates your current 
                                        position by starting from a known position and calculating subsequent positions based on 
                                        estimated changes in distance and direction, typically using internal sensors like accelerometers, 
                                        gyroscopes, or odometers. It is commonly used to maintain navigation continuity in environments 
                                        where GNSS (Global Navigation Satellite System) signals are unreliable or unavailable, 
                                        such as tunnels, urban canyons, or indoors.
                    */
                    QualityIndicator = atoi(current_token); // Convert the quality indicator to an integer
                    ESP_LOGI(TAG, "Quality Indicator: %d", QualityIndicator);
                    break;

                case 7: // Field 7: Number of Satellites
                    /* Number of satellites used to calculate the position fix (0-12) */
                    NumberOfSatellites = atoi(current_token); // Convert the number of satellites to an integer
                    ESP_LOGI(TAG, "Number of Satellites: %d", NumberOfSatellites);
                    break;

                case 8: // Field 8: HDOP
                    /* Horizontal Dilution of Precision (HDOP) is a measure of the quality of the horizontal position fix.
                       It indicates how much the position accuracy is affected by the geometry of the satellites used for the fix.
                       Lower values indicate better satellite geometry and higher accuracy. */
                    HorizontalDilutionOfPrecision = atof(current_token); // Convert HDOP to double
                    ESP_LOGI(TAG, "Horizontal Dilution of Precision (HDOP): %f", HorizontalDilutionOfPrecision);
                    break;

                case 9: // Field 9: Altitude (Meters)
                    Altitude = atof(current_token);
                    ESP_LOGI(TAG, "Altitude [m]: %f", Altitude);
                    break;

                case 10: // Field 10: Altitude Unit (M)
                    /* Altitude unit is always 'M' (meters) in GGA sentences, so we can skip this field */
                    break;

                case 11: // Field 11: Geoidal Separation
                    /* Geoidal separation is the difference between the WGS-84 ellipsoid and the geoid (mean sea level).
                       It is used to convert between ellipsoidal height (GPS altitude) and orthometric height (height above mean sea level).
                       This field is optional in GGA sentences, so we can skip it if it's not present. */
                    GeoidalSeparation = atof(current_token); // Convert geoidal separation to double
                    ESP_LOGI(TAG, "Geoidal Separation [m]: %f", GeoidalSeparation);
                    break;

                case 12: // Field 12: Geoidal Separation Unit (M)
                    /* Geoidal separation unit is always 'M' (meters) in GGA sentences, so we can skip this field */
                    break;

                case 13: // Field 13: Age of Diff. Corr. (seconds)
                    /* Age of Differential Correction is the time in seconds since the last differential correction was applied.
                       It is used to determine how fresh the differential correction data is. If this field is empty, it means no differential correction is applied.
                       This field is optional in GGA sentences */
                    AgeOfDifferentialGPSData = atof(current_token); // Convert age of differential correction to double
                    ESP_LOGI(TAG, "Age of Differential GPS Data [s]: %f", AgeOfDifferentialGPSData);
                    /* Note: If this field is empty, it means no differential correction is applied */
                    break;
                    
                case 14: // Field 14: Diff. Ref. Station ID
                    /* Differential Reference Station ID is the ID of the reference station used for differential correction.
                       It is used to identify the reference station that provided the differential correction data.
                       This field is optional in GGA sentences */
                    DifferentialReferenceStationID = atoi(current_token); // Convert differential reference station ID to integer
                    ESP_LOGI(TAG, "Differential Reference Station ID: %d", DifferentialReferenceStationID);
                    /* Note: If this field is empty, it means no differential correction is applied */
                    break;

                default:
                    // For other fields, we just let strtok advance to the next one
                    break;
            }
        }
    }

}