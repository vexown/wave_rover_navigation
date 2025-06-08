/******************************************************************************
 * @file GNSS_ublox.h
 * @brief Header file for the GNSS ublox component.
 * 
 * ublox module version: NEO-M8N (on a GY-GPS6MV2 board)
 * 
 * ******************************************************************************
 * Component Documentation:
 * - GY-GPS6MV2: https://www.mantech.co.za/datasheets/products/GY-NEO6MV2.pdf?srsltid=AfmBOopySN4-ZiJYJiXr1-vGTqREfjrpNIgzLWcxbotFqheY5cvgyicB
 * - ublox NEO-M8N: https://content.u-blox.com/sites/default/files/NEO-M8-FW3_DataSheet_UBX-15031086.pdf
 * - ublox M8 Receiver Description Including Protocol Specification: https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf?utm_content=UBX-13003221
 * ******************************************************************************
 * NMEA Protocol Frame Structure:
 * 
 * 1.  Start character:
 *      - Always '$'
 *
 * 2.  Address field:
 *      - Contains only digits and uppercase letters.
 *      - Cannot be null.
 *      - Subdivided into two sub-fields:
 *          a. Talker Identifier (<XX>):
 *              - GP for GPS, GL for GLONASS, GA for Galileo, etc.
 *          b. Sentence Formatter (<XXX>):
 *              - Defines the message content: ZDA for time and date, GGA for fix data, etc.
 *              - See "Messages overview" in the ublox M8 Receiver Description document for full list.
 * 
 * 3.  Data field(s):
 *      - Delimited by a ','.
 *      - Length can vary, even for a certain field. Refer to the ublox M8 Receiver Description document for details.
 *
 * 4.  Checksum field:
 *      - Starts with a '*' character.
 *      - Followed by 2 characters representing a hexadecimal number.
 *      - The checksum is the exclusive OR (XOR) of all characters between '$' and '*'.
 *      - Checksum range: Address field + Data field(s).
 *
 * 5.  End sequence:
 *      - Always '<CR><LF>' (Carriage Return and Line Feed).
 *
 * Example:
 * $  GP  ZDA  ,141644.00,22,03,2002,00,00  *67  <CR><LF>
 * 
 ******************************************************************************/

#ifndef GNSS_UBLOX_H
#define GNSS_UBLOX_H

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
/*     Include headers required for the declarations in *this* header file     */
/*                 (e.g., types used in function prototypes).                  */
/*       Prefer forward declarations over full includes where possible         */
/*             to minimize dependencies (for structs, enums etc.).             */
/*******************************************************************************/

/* C Standard Libraries */
#include <stdint.h>
#include <stdbool.h> 

/* ESP-IDF Libraries */
#include "esp_err.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/

/*******************************************************************************/
/*                                DATA TYPES                                   */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL VARIABLES DECLARATIONS                           */
/*******************************************************************************/
/*   for variables defined in the corresponding .c file, for use in other .c   */
/*      files just by including this header file. Use extern for these.        */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL FUNCTION DECLARATIONS                            */
/*******************************************************************************/
/*   for functions defined in the corresponding .c file, for use in other .c   */
/*    files just by including this header file. Extern is a default linkage    */
/*    specifier for functions, so it is not necessary to use it explicitly.    */
/*******************************************************************************/

/**
 * @brief Initialize the communication with the GNSS ublox module.
 * 
 * This function performs configuration of the UART interface, sets up the necessary
 * pins, and installs the UART driver. It also creates a FreeRTOS task to handle
 * incoming data from the GNSS module.
 * 
 * @return esp_err_t ESP_OK on success. If the function fails the program will abort via ESP_ERROR_CHECK
 *         and log the error message. There is also a chance of returning ESP_FAIL if the task creation fails.
 */
esp_err_t GNSS_ublox_init(void);

/**
 * @brief Get the current coordinates from the GNSS ublox module.
 * 
 * This function provides to the user ALREADY EXTRACTED coordinates in decimal degrees format (latitude, longitude)
 * and altitude in meters. The coordinates are being extracted continuously if the GNSS_ublox_init() function was called.
 * 
 * @param[out] latitude Pointer to store the latitude in decimal degrees.
 * @param[out] longitude Pointer to store the longitude in decimal degrees.
 * @param[out] altitude Pointer to store the altitude in meters.
 * 
 * @return esp_err_t ESP_OK on success. ESP_ERR_INVALID_ARG if any of the pointers are NULL.
 * 
 */
esp_err_t GNSS_ublox_get_coordinates(double *latitude, double *longitude, double *altitude);

/**
 * @brief Get the current GPS quality information from the GNSS ublox module.
 * 
 * This function provides additional GPS quality metrics that can be used to assess
 * the reliability and accuracy of the position fix. The values are being extracted
 * continuously if the GNSS_ublox_init() function was called.
 * 
 * @param[out] quality_indicator Pointer to store the quality indicator (0-6 scale).
 *                               0 = No fix, 1/2 = 2D/3D fix, 3 = PPS fix, 4 = RTK fixed, etc.
 * @param[out] num_satellites Pointer to store the number of satellites used in the fix (0-12+).
 * @param[out] hdop Pointer to store the Horizontal Dilution of Precision.
 *                  Lower values indicate better satellite geometry and higher accuracy.
 * 
 * @return esp_err_t ESP_OK on success. ESP_ERR_INVALID_ARG if any of the pointers are NULL.
 */
esp_err_t GNSS_ublox_get_quality_info(int *quality_indicator, int *num_satellites, double *hdop);

#endif /* GNSS_UBLOX_H */