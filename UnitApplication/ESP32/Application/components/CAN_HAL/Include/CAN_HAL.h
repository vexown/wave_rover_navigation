/**
 * @file CAN_HAL.h
 * @brief Header file for the CAN Hardware Abstraction Layer
 */

#ifndef CAN_HAL_H
#define CAN_HAL_H

/*******************************************************************************/
/*                                 DEFINES                                     */
/*******************************************************************************/
#define CAN_TX_PIN  GPIO_NUM_5
#define CAN_RX_PIN  GPIO_NUM_4

#define ESP32_1_CAN_ID (uint32_t)0xA1
#define ESP32_2_CAN_ID (uint32_t)0xA2

#define DEFAULT_RESPONSE_ID         0
#define ESP32_1_CAN_ID_RESPONSE_ID  1
#define ESP32_2_CAN_ID_RESPONSE_ID  2

#define STANDARD_CAN_MAX_DATA_LENGTH 8
/*******************************************************************************/
/*                               DATA TYPES                                    */
/*******************************************************************************/

/*******************************************************************************/
/*                            GLOBAL VARIABLES                                 */
/*******************************************************************************/

/*******************************************************************************/
/*                        GLOBAL FUNCTION DELCARATION                          */
/*******************************************************************************/

/**
 * @brief Initializes the TWAI peripheral (Two-Wire Automotive Interface)
 * 
 * This function initializes the TWAI peripheral with the default configuration
 * for the ESP32. The default configuration uses the following settings:
 * - Normal mode
 * - 500 kbps baud rate
 * - Accept all messages
 * 
 * @param None
 * @return esp_err_t Returns ESP_OK if the TWAI peripheral was successfully initialized or the error code if it failed
 * 
 * Note - See more about the TWAI peripheral here: 
 * https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32/api-reference/peripherals/twai.html
 */
esp_err_t init_twai(void);

/**
 * @brief Sends a CAN message
 * 
 * This function sends a CAN message with the specified message ID and data.
 * The data length is also specified.
 * 
 * @param message_id The message ID of the CAN message
 * @param data The data to be sent in the CAN message
 * @param data_length The length of the data to be sent
 * @return esp_err_t Returns ESP_OK if the message was successfully sent or the error code if it failed
 */
esp_err_t send_CAN_message(uint32_t message_id, const uint8_t *data, uint8_t data_length);

/**
 * @brief Receives a CAN message
 * 
 * This function receives a CAN message and copies the data to the buffer provided by the user.
 * The length of the data received is also returned.
 * 
 * @param buffer The buffer to store the received data
 * @param buffer_length The length of the buffer (input) and the length of the data received (output)
 * @return esp_err_t Returns ESP_OK if a message was successfully received or the error code if it failed
 */
esp_err_t receive_CAN_message(uint32_t* message_id, uint8_t* buffer, uint8_t* buffer_length);

/**
 * @brief Monitors the CAN bus checking the alerts and the status of the driver
 *
 * Alters:
 *      Critical TWAI alerts indicating significant communication issues include TWAI_ALERT_BUS_ERROR 
 *      (indicating data corruption or protocol violations) and TWAI_ALERT_BUS_OFF (signaling a complete 
 *      loss of communication capability). See full list of alerts in twai_alert_messages array.
 * 
 * Driver status:
 *      Includes the number of transmitted and received messages, counts of various 
 *      error types (transmission failures, receive misses, overruns, arbitration losses, and bus errors), 
 *      and the current communication state. In essence, it provides a comprehensive overview of the TWAI 
 *      driver's operational health and performance.
 *
 * @return bool Returns true if the monitoring was successful, false if there were issues
 */
bool monitor_CAN_bus(void);


#endif /* CAN_HAL_H */