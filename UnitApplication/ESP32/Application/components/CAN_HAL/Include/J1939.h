/**
 * @file J1939.h
 * @brief Header file for the J1939 protocol implementation
 */

#ifndef J1939_H
#define J1939_H

/*******************************************************************************/
/*                                 DEFINES                                     */
/*******************************************************************************/
#define J1939_ENABLED 1

/* PGN definitions */
#define ESP32_1_PGN         65262 // PGN (Parameter Group Number) for ESP32_1 (0xFEEE)
#define ESP32_2_PGN         65266 // PGN (Parameter Group Number) for ESP32_2 (0xFEF2)
#define PGN_EEC1            61444 // PGN for Electronic Engine Controller 1 - EEC1 (0xF004)
#define PGN_CCVS            65265 // PGN for Cruise Control/Vehicle Speed - CCVS (0xFEF1)

/* Source address identifies the sender of the message. It is a unique address assigned to each device on the CAN network. */
#define ESP32_1_SRC_ADDR    0x01  // Source Address for ESP32_1
#define ESP32_2_SRC_ADDR    0x02  // Source Address for ESP32_2

/* Address claiming and global address */
#define J1939_GLOBAL_ADDRESS 0xFF // Standard global address

/*******************************************************************************/
/*                               DATA TYPES                                    */
/*******************************************************************************/
/**
 * @brief Structure to hold J1939 message components and data.
 */
typedef struct 
{
    uint8_t priority;       ///< Message priority (0-7)
    uint8_t data_page;      ///< Data Page (0 or 1)
    uint8_t pdu_format;     ///< PDU Format (0-255)
    uint8_t pdu_specifics;  ///< PDU Specific (0-255, Destination Address or Group Extension)
    uint8_t src_address;    ///< Source Address (0-255)
    uint32_t pgn;           ///< Calculated PGN (TODO - add PGN calculation in receive function)
    uint8_t data[8];        ///< Data field payload (up to 8 bytes for standard CAN)
    uint8_t data_length;    ///< Length of the data field in bytes
    // Add timestamp or other metadata if needed later
} J1939_Message_t;

/**
 * @brief Structure to define a J1939 PGN (Parameter Group Number) and its properties.
 */
typedef struct 
{
    uint32_t pgn;                   // The Parameter Group Number itself (e.g., 61444)
    uint8_t  default_priority;      // Default priority for this PGN
    uint8_t  data_page;             // Data Page (usually 0)
    uint8_t  pdu_format;            // PDU Format (PF) derived from PGN
    uint8_t  pdu_specific_or_ge;    // PDU Specific (PS) if PDU1 (usually 0xFF for broadcast/global), or Group Extension (GE) if PDU2
    uint8_t  data_length;           // Expected data length for this PGN
} J1939_PGN_Definition_t;

/*******************************************************************************/
/*                            GLOBAL VARIABLES                                 */
/*******************************************************************************/

/*******************************************************************************/
/*                        GLOBAL FUNCTION DELCARATION                          */
/*******************************************************************************/

/**
 * @brief Receives a J1939 message.
 *
 * @param message Pointer to the J1939_Message_t structure to fill with received data.
 *
 * @return esp_err_t ESP_OK on success, ESP_FAIL or other error codes on failure.
 */
esp_err_t receive_J1939_message(J1939_Message_t *message);

/**
 * @brief Sends a J1939 message for a specific PGN using its definition from the database.
 *
 * The user provides the PGN, the data, the source address, and the destination address.
 * The function looks up the PGN to determine priority, data page, PDU format,
 * group extension (for PDU2), and data length.
 *
 * @param pgn_to_send The Parameter Group Number (PGN) to send.
 * @param dest_address The destination address. For PDU1 messages, this is the specific
 *                     target address. For PDU2 messages or broadcast PDU1, use
 *                     J1939_GLOBAL_ADDRESS (0xFF).
 * @param src_address The source address of this ECU.
 * @param data_payload Pointer to the data buffer to send. The size should ideally
 *                     match the data_length defined for the PGN.
 * @param actual_data_len The actual number of bytes to send from data_payload.
 *                        Should typically be <= the PGN's defined data_length.
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_FOUND if PGN not supported,
 *                   ESP_ERR_INVALID_ARG if arguments are invalid, or CAN HAL error.
 */
esp_err_t send_J1939_message_by_pgn(uint32_t pgn_to_send, uint8_t dest_address, uint8_t src_address, const uint8_t *data_payload, uint8_t actual_data_len);

#endif /* J1939_H */
