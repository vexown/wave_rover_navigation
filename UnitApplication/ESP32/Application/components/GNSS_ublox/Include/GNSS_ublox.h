/******************************************************************************
 * @file GNSS_ublox.h
 * @brief Header file for the GNSS ublox component.
 * 
 * ublox module version: NEO-M8N (on a GY-GPS6MV2 board)
 * 
 * Component Documentation:
 * - GY-GPS6MV2: https://www.mantech.co.za/datasheets/products/GY-NEO6MV2.pdf?srsltid=AfmBOopySN4-ZiJYJiXr1-vGTqREfjrpNIgzLWcxbotFqheY5cvgyicB
 * - ublox NEO-M8N: https://content.u-blox.com/sites/default/files/NEO-M8-FW3_DataSheet_UBX-15031086.pdf
 * - ublox M8 Receiver Description Including Protocol Specification: https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf?utm_content=UBX-13003221
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
 * @brief Initialize the GNSS ublox module.
 * 
 * This function initializes the GNSS ublox module by configuring the UART
 * settings and starting the UART task for communication.
 * 
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t GNSS_ublox_Init(void);

#endif /* GNSS_UBLOX_H */