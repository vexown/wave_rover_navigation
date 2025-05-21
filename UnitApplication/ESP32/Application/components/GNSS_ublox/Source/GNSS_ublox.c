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

/* Project Includes */
#include "GNSS_ublox.h"

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
static void GNSS_UART_Rx_Task(void *pvParameters);

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/

/*******************************************************************************/
/*                     GLOBAL FUNCTION DEFINITIONS                             */
/*******************************************************************************/

esp_err_t GNSS_ublox_Init(void)
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

    /* Set UART pins (TX, RX, RTS, CTS) and the UART port number */
    /* Note: RTS and CTS are not used in this case, so we set them to UART_PIN_NO_CHANGE */
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, UART_EVENT_QUEUE_SIZE, NULL, UART_INTERRUPT_ALLOC_FLAGS));

    /* Create a task to handle UART data from the GNSS module */
    BaseType_t status = xTaskCreate(GNSS_UART_Rx_Task, "GNSS_UART_Rx_Task", UART_RX_TASK_STACK_SIZE, NULL, UART_RX_TASK_PRIORITY, NULL);
    if (status != pdPASS) 
    {
        ESP_LOGE(TAG, "Failed to create GNSS UART RX task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "GNSS_ublox_Init completed and UART RX task started.");

    /* If we reach here, the initialization was successful (otherwise ESP_ERROR_CHECK logs the error and aborts) */
    return ESP_OK; 
}

/*******************************************************************************/
/*                     STATIC FUNCTION DEFINITIONS                             */
/*******************************************************************************/

static void GNSS_UART_Rx_Task(void *pvParameters)
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
            
            ESP_LOGI(TAG, "Received from ublox: %s", received_data);
            // Process the received data here (e.g., parse NMEA sentences, etc.) (TODO)
        }
    }

    vTaskDelete(NULL); // should never reach here but just in case
}
