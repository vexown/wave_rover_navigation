/**
 * ****************************************************************************
 * @file    J1939.c
 *
 * @details SAE J1939 is a higher-layer protocol based on CAN (Controller Area Network)
 *          that is widely used in the automotive and heavy-duty vehicle industries.
 *          The particular characteristics of J1939 are:
 *          - 29-bit CAN identifiers
 *          - Bit rate of 250 kbps
 *          - Peer-to-peer and broadcast communication
 *          - Transport Protocols for large messages up to 1785 bytes
 *          - Parameter Group Numbers (PGNs) to identify messages for commercial vehicles and others
 *          - Address claiming for dynamic addressing
 *          - Network management for node status and control 
 *          - Diagnostic messages for troubleshooting
 * 
 * ****************************************************************************
 */

/*******************************************************************************/
/*                                INCLUDES                                     */
/*******************************************************************************/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "CAN_HAL.h"
#include "Common.h"
#include "J1939.h"

/*******************************************************************************/
/*                                 MACROS                                      */
/*******************************************************************************/

/* Priority for J1939 messages (0-7, occupies the 3 most significant bits (MSB) of the 29-bit identifier) */
#define PRIORITY_HIGH       (uint8_t)0b000  // 0 - highest priority
#define PRIORITY_NORMAL     (uint8_t)0b011  // 3 - normal priority
#define PRIORITY_LOW        (uint8_t)0b111  // 7 - lowest priority

/* Macros for PDU types */
#define J1939_PDU1_PF_LOWER_BOUND 0x00
#define J1939_PDU1_PF_UPPER_BOUND 0xEF
#define J1939_PDU2_PF_LOWER_BOUND 0xF0
#define J1939_PDU2_PF_UPPER_BOUND 0xFF

/* Macro to check if a PF value corresponds to PDU1 (PDU Specific Field is then a destination address) */
#define J1939_IS_PDU1(pf) (pf <= J1939_PDU1_PF_UPPER_BOUND)

/* Macro to check if a PF value corresponds to PDU2 (PDU Specific Field is then a group extension) */
#define J1939_IS_PDU2(pf) ((pf >= J1939_PDU2_PF_LOWER_BOUND) && (pf <= J1939_PDU2_PF_UPPER_BOUND))

/* Macro to extract the destination address from the PS field, if the message is PDU1 */
#define J1939_DESTINATION_ADDRESS(pf, ps) (J1939_IS_PDU1(pf) ? ps : 0xFF) //0xff is commonly used to signify no destination address.

/* Macro to extract the group extension from the PS field, if the message is PDU2 */
#define J1939_GROUP_EXTENSION(pf, ps) (J1939_IS_PDU2(pf) ? ps : 0xFF) //0xff is commonly used to signify no group extension.

/* Define masks and shifts for J1939 CAN ID fields */
#define J1939_PRIORITY_MASK     0x07 // 3 bits
#define J1939_DP_MASK           0x01 // 1 bit
#define J1939_PF_MASK           0xFF // 8 bits
#define J1939_PS_MASK           0xFF // 8 bits
#define J1939_SA_MASK           0xFF // 8 bits
#define J1939_PRIORITY_SHIFT    26
#define J1939_DP_SHIFT          24
#define J1939_PF_SHIFT          16
#define J1939_PS_SHIFT          8
#define J1939_SA_SHIFT          0

/*******************************************************************************/
/*                               DATA TYPES                                    */
/*******************************************************************************/

/*******************************************************************************/
/*                        GLOBAL FUNCTION DECLARATIONS                         */
/*******************************************************************************/

/*******************************************************************************/
/*                        STATIC FUNCTION DECLARATIONS                         */
/*******************************************************************************/

/**
 * @brief Assembles a J1939 29-bit CAN message identifier.
 *
 * @param priority     Message priority (0-7).
 * @param data_page    Data Page (0 or 1).
 * @param pdu_format   PDU Format (0-255). Determines message type.
 * @param pdu_specific PDU Specific (0-255). Destination Address (if PF < 240)
 * or Group Extension (if PF >= 240).
 * @param src_address  Source Address (0-255, excluding 254/255 usually).
 * @return uint32_t The assembled 29-bit J1939 identifier.
 */
static uint32_t assembleJ1939MessageID( uint8_t priority,
                                        uint8_t data_page,
                                        uint8_t pdu_format,
                                        uint8_t pdu_specific,
                                        uint8_t src_address);


/**
* @brief Disassembles a J1939 29-bit CAN message identifier into its components.
*
* @param messageID    The 29-bit J1939 identifier to disassemble.
* @param priority     Pointer to store the extracted priority (0-7).
* @param data_page    Pointer to store the extracted Data Page (0 or 1).
* @param pdu_format   Pointer to store the extracted PDU Format (0-255).
* @param pdu_specifics Pointer to store the extracted PDU Specific (0-255).
* @param src_address  Pointer to store the extracted Source Address (0-255).
*/
static void disassembleJ1939MessageID(  uint32_t messageID,
                                        uint8_t *priority,
                                        uint8_t *data_page,
                                        uint8_t *pdu_format,
                                        uint8_t *pdu_specifics,
                                        uint8_t *src_address);

/**
 * @brief Finds the definition for a given PGN in the local database.
 *
 * @param pgn The PGN to look for.
 * @return Pointer to the PGN definition, or NULL if not found.
 */
static const J1939_PGN_Definition_t* find_pgn_definition(uint32_t pgn);

/*******************************************************************************/
/*                            STATIC VARIABLES                                 */
/*******************************************************************************/
/**
 * @brief Database of PGNs this ECU can send
 * @note For PDU1 PGNs where the PS is *always* the DA, set pdu_specific_or_ge to 0xFF or similar placeholder
 *       if the PGN definition itself doesn't imply a specific PS value beyond the DA.
 *       For details regarding PGN definitions, see comments next to the PGN definitions and the J1939-71 standard.
 */
static const J1939_PGN_Definition_t supported_tx_pgns[] = 
{
    // PGN          Priority         DP PF    PS/GE  Len 
    { PGN_EEC1,     PRIORITY_NORMAL, 0, 240,  4,     8 },
    { PGN_CCVS,     PRIORITY_NORMAL, 0, 254,  241,   8 },
    { ESP32_1_PGN,  PRIORITY_NORMAL, 0, 254,  238,   8 },
    { ESP32_2_PGN,  PRIORITY_NORMAL, 0, 254,  242,   8 }, 
    // Add other PGNs your application needs to send here
};

/**
 * @brief Number of supported PGNs
 */
static const size_t num_supported_tx_pgns = sizeof(supported_tx_pgns) / sizeof(supported_tx_pgns[0]);


/*******************************************************************************/
/*                            GLOBAL VARIABLES                                 */
/*******************************************************************************/

/*******************************************************************************/
/*                        STATIC FUNCTION DEFINITIONS                          */
/*******************************************************************************/

static const J1939_PGN_Definition_t* find_pgn_definition(uint32_t pgn)
{
    /* Iterate through the supported PGNs and find the one that matches the given PGN */
    for (size_t i = 0; i < num_supported_tx_pgns; i++) 
    {
        if (supported_tx_pgns[i].pgn == pgn) 
        {
            return &supported_tx_pgns[i];
        }
    }

    LOG("PGN %lu not found in supported PGNs\n", pgn);
    return NULL; // PGN not found in our database of supported PGNs
}

static uint32_t assembleJ1939MessageID( uint8_t priority,
                                        uint8_t data_page,
                                        uint8_t pdu_format,
                                        uint8_t pdu_specific,
                                        uint8_t src_address)
{
    uint32_t messageID = 0;

    /* J1939 CAN ID Structure (based on J1939-21 Rev Dec 2006):
    *      Bits 28-26: Priority (P)
    *      Bit 25:     Extended Data Page (EDP) - Must be 0 for standard messages per this revision.
    *      Bit 24:     Data Page (DP)
    *      Bits 23-16: PDU Format (PF)
    *      Bits 15-8:  PDU Specific (PS)
    *      Bits 7-0:   Source Address (SA)
    */

    /* Validate the non-8-bits elements to throw an error if they exceed their limits (should never happen) */
    assert((priority & ~J1939_PRIORITY_MASK) == 0); // Make sure priority fits in 3 bits
    assert((data_page & ~J1939_DP_MASK) == 0); // Make sure data_page fits in 1 bit
    /* No need to assert PF, PS, SA if uint8_t, as they fit 8 bits. */

    /* Assemble the message ID using masks and shifts */
    /* Masking ensures inputs don't exceed their allocated bits. */
    messageID |= ((uint32_t)(priority    & J1939_PRIORITY_MASK) << J1939_PRIORITY_SHIFT);
    /* Bit 25 (EDP) is implicitly 0 because we start with messageID = 0 and the priority shift doesn't affect it.*/
    messageID |= ((uint32_t)(data_page   & J1939_DP_MASK)       << J1939_DP_SHIFT);
    messageID |= ((uint32_t)(pdu_format  & J1939_PF_MASK)       << J1939_PF_SHIFT);
    messageID |= ((uint32_t)(pdu_specific& J1939_PS_MASK)       << J1939_PS_SHIFT);
    messageID |= ((uint32_t)(src_address & J1939_SA_MASK)       << J1939_SA_SHIFT); // Shift is 0, but shown for consistency

    return messageID;
}

static void disassembleJ1939MessageID(  uint32_t messageID,
                                        uint8_t *priority,
                                        uint8_t *data_page,
                                        uint8_t *pdu_format,
                                        uint8_t *pdu_specifics,
                                        uint8_t *src_address) 
{
    /* Extract the components from the message ID */
    *priority = (uint8_t)((messageID >> J1939_PRIORITY_SHIFT) & J1939_PRIORITY_MASK);
    *data_page = (uint8_t)((messageID >> J1939_DP_SHIFT) & J1939_DP_MASK);
    *pdu_format = (uint8_t)((messageID >> J1939_PF_SHIFT) & J1939_PF_MASK);
    *pdu_specifics = (uint8_t)((messageID >> J1939_PS_SHIFT) & J1939_PS_MASK);
    *src_address = (uint8_t)((messageID >> J1939_SA_SHIFT) & J1939_SA_MASK);
}

/*******************************************************************************/
/*                        GLOBAL FUNCTION DEFINITIONS                          */
/*******************************************************************************/

esp_err_t send_J1939_message_by_pgn(uint32_t pgn_to_send, uint8_t dest_address, uint8_t src_address, const uint8_t *data_payload, uint8_t actual_data_len)
{
    const J1939_PGN_Definition_t* pgn_def = find_pgn_definition(pgn_to_send);

    if (pgn_def == NULL) {
        // Optionally log: printf("Error: PGN %lu not supported for sending.\n", pgn_to_send);
        return ESP_ERR_NOT_FOUND;
    }

    if (data_payload == NULL || actual_data_len > pgn_def->data_length) {
         // Optionally log: printf("Error: Invalid data payload or length for PGN %lu (expected %u, got %u)\n", pgn_to_send, pgn_def->data_length, actual_data_len);
         return ESP_ERR_INVALID_ARG;
    }

    /* Check if TP (Transport Protocol) is required to send the message (i.e., if the data length exceeds 8 bytes) */
    /* J1939 messages with more than 8 bytes of data require the Transport Protocol (TP) to split the message into multiple frames */
    if (actual_data_len > STANDARD_CAN_MAX_DATA_LENGTH) 
    {
        /* TODO - Implement Transport Protocol (TP) for messages larger than 8 bytes. For now, return an error. */
        LOG("Error: Data length %u exceeds max CAN frame size for PGN %lu\n", actual_data_len, pgn_to_send);
        return ESP_ERR_INVALID_ARG;
    }

    /* Below core values are often taken from one of the predefined PGNs defined in the J1939-71 standard (except manufacturer specific PGNs)
        Example of a standard PGN: 
        EEC1 (Electronic Engine Controller 1) with PGN 61444 (0xF004)
            -> Data Length: 8 bytes
            -> Data Page: 0
            -> PDU Format: 240
            -> PDU Specifics: 3
            -> Default Priority: 3 */
    /* PGN (Parameter Group Number) is a key to understanding the content of a J1939 message. It enables you to find the definitions 
        of standard messages within the J1939 documentation. It is the primary way that the J1939 standard organizes its message definitions.
        PGN is not directly placed into the message ID, but is used to derive the PDU format and PDU specifics. */
    uint8_t priority = pgn_def->default_priority; // Decide the priority level for the message (0-7, lower number = higher priority)
    uint8_t data_page = pgn_def->data_page; // Data page (0 or 1, used for multi-page messages)
    uint8_t pdu_format = pgn_def->pdu_format; // PDU format (0-255, decides whether PDU Specific Field is a destination address (PDU1) or group ext (PDU2))
    uint8_t pdu_specific; // PDU Specifics (0-255, used for destination address in PDU1 or group extension in PDU2)

    /* Determine PDU Specific field based on PDU Format derived from the PGN definition */
    if (J1939_IS_PDU1(pdu_format))
    {
        /* For PDU1 messages, PS is the Destination Address provided by the user */
        pdu_specific = dest_address;
    }
    else
    { /* PDU2 */
        /* For PDU2 messages, PS is the Group Extension defined by the PGN definition */
        pdu_specific = pgn_def->pdu_specific_or_ge;
        /* For PDU2, the destination address parameter is effectively ignored in ID assembly,
           but the standard implies broadcast (address FF). */
    }

    /* Shift all the values to their respective positions in the 29-bit ID */
    uint32_t message_id = assembleJ1939MessageID(priority, data_page, pdu_format, pdu_specific, src_address);

    /* Send the message using the actual data length provided by the caller */
    return send_CAN_message(message_id, data_payload, actual_data_len);
}

esp_err_t receive_J1939_message(J1939_Message_t *message)
{
    uint32_t received_id;
    uint8_t received_dlc;
    uint8_t received_data[8]; // Max standard CAN data length

    if (message == NULL) 
    {
        return ESP_ERR_INVALID_ARG; // Ensure the message pointer is valid
    }

    /* Assume a function receive_CAN_message exists in CAN_HAL */
    /* This is a placeholder signature - adapt to your actual CAN_HAL function */
    esp_err_t recv_status = receive_CAN_message(&received_id, received_data, &received_dlc);
    if (recv_status != ESP_OK) 
    {
        return recv_status; // Handle timeout or other reception errors
    }

    /* Disassemble the received 29-bit ID into J1939 components */
    disassembleJ1939MessageID(received_id, &message->priority, &message->data_page, &message->pdu_format, &message->pdu_specifics, &message->src_address);

    /* Copy the received data payload */
    /* Ensure the provided buffer is large enough (typically 8 bytes for standard CAN) */
    /* J1939 TP messages would require additional handling for > 8 bytes */
    if (received_dlc <= 8) 
    { 
        memcpy(message->data, received_data, received_dlc);
        message->data_length = received_dlc;
    } 
    else 
    {
        // This should not happen for a single standard CAN frame
        // Handle error or potential TP frame start
        message->data_length = 0;
        return ESP_FAIL; // Indicate an unexpected data length
    }

    /* TODO: Add further processing based on PGN (derived from PF/PS) */
    /* Example: Look up PGN, parse SPNs from data_field based on PGN definition */
    // uint32_t pgn = calculate_PGN(message->pdu_format, message->pdu_specifics); // Need a function for this
    // process_PGN(pgn, message->data, message->data_length);

    return ESP_OK;
}