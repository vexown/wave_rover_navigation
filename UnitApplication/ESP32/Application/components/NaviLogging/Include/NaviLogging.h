/******************************************************************************
 * @file NaviLogging.h
 * @brief Header file for the Template component
 *
 ******************************************************************************/

#ifndef NAVILOGGING_H
#define NAVILOGGING_H

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

typedef struct 
{
    double latitude;                        /*!< Latitude in decimal degrees */
    double longitude;                       /*!< Longitude in decimal degrees */
    double altitude;                        /*!< Altitude in meters */
    int quality_indicator;                  /*!< GPS quality indicator (0-6): 0=No fix, 1/2=2D/3D fix, 3=PPS, 4=RTK fixed, etc. */
    int number_of_satellites;               /*!< Number of satellites used in the position fix (0-12+) */
    double horizontal_dilution_of_precision; /*!< Horizontal Dilution of Precision (HDOP) - lower values indicate better accuracy */
} navi_coordinates_type;

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
 * @brief Initialize the NaviLogging component.
 *
 * @details This function initializes ESP-NOW and sets up a task to periodically
 *          fetch coordinates and send them.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL if initialization fails
 */
esp_err_t NaviLogging_init(void);

#endif /* NAVILOGGING_H */