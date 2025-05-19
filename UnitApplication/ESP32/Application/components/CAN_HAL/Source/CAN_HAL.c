/**
 * ****************************************************************************
 * @file    CAN_HAL.c
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
#include <inttypes.h>
#include "sdkconfig.h"
#include "CAN_HAL.h"
#include "J1939.h"
#include "Common.h"

/*******************************************************************************/
/*                                 MACROS                                      */
/*******************************************************************************/

/*******************************************************************************/
/*                               DATA TYPES                                    */
/*******************************************************************************/
/**
 * @brief Struct to hold TWAI alert flag and corresponding message
 */
typedef struct 
{
    uint32_t flag;
    const char *message;
} twai_alert_message_t;

/*******************************************************************************/
/*                        GLOBAL FUNCTION DECLARATIONS                         */
/*******************************************************************************/

/*******************************************************************************/
/*                        STATIC FUNCTION DECLARATIONS                         */
/*******************************************************************************/
/**
 * @brief Handles analyzing and responding to a Remote Frame Request
 * 
 * @param message Pointer to the received TWAI message
 * @return esp_err_t Returns ESP_OK if the response was successfully sent or the error code if it failed
 * 
 */
static esp_err_t remote_frame_responder(twai_message_t *message);

/**
 * @brief Process and print active TWAI alerts
 * 
 * @param alerts Combined alerts value from twai_read_alerts
 */
static void process_twai_alerts(uint32_t alerts);

/*******************************************************************************/
/*                            STATIC VARIABLES                                 */
/*******************************************************************************/
/**
 * @brief Lookup table for CAN message responses
 */
static const uint8_t response_templates[3][STANDARD_CAN_MAX_DATA_LENGTH] = 
{
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
    {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xF0, 0xF7, 0xFF},
    {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88}
};

/**
 * @brief Lookup table for TWAI alert messages
 */
static const twai_alert_message_t twai_alert_messages[] = 
{
    {TWAI_ALERT_TX_IDLE,              "No more messages to transmit"},
    {TWAI_ALERT_TX_SUCCESS,           "Previous transmission was successful"},
    {TWAI_ALERT_RX_DATA,              "Frame received and added to RX queue"},
    {TWAI_ALERT_BELOW_ERR_WARN,       "Error counters dropped below error warning limit"},
    {TWAI_ALERT_ERR_ACTIVE,           "TWAI controller has become error active"},
    {TWAI_ALERT_RECOVERY_IN_PROGRESS, "TWAI controller is undergoing bus recovery"},
    {TWAI_ALERT_BUS_RECOVERED,        "TWAI controller has completed bus recovery"},
    {TWAI_ALERT_ARB_LOST,             "Previous transmission lost arbitration"},
    {TWAI_ALERT_ABOVE_ERR_WARN,       "Error counter(s) exceeded warning limit"},
    {TWAI_ALERT_BUS_ERROR,            "Bus error occurred (Bit, Stuff, CRC, Form, ACK)"},
    {TWAI_ALERT_TX_FAILED,            "Previous transmission failed (for single shot)"},
    {TWAI_ALERT_RX_QUEUE_FULL,        "RX queue full, causing frame loss"},
    {TWAI_ALERT_ERR_PASS,             "TWAI controller has become error passive"},
    {TWAI_ALERT_BUS_OFF,              "Bus-off condition, controller cannot influence bus"},
    {TWAI_ALERT_RX_FIFO_OVERRUN,      "RX FIFO overrun occurred"},
    {TWAI_ALERT_TX_RETRIED,           "Message transmission retried due to errata workaround"},
    {TWAI_ALERT_PERIPH_RESET,         "TWAI controller was reset"}
};

/* Convert TWAI state enum to a descriptive string */
static const char* twai_state_to_string(twai_state_t state)
{
    switch (state) 
    {
        case TWAI_STATE_STOPPED:
            return "STOPPED - Controller not participating in bus activities";
        case TWAI_STATE_RUNNING:
            return "RUNNING - Controller can transmit and receive messages";
        case TWAI_STATE_BUS_OFF:
            return "BUS_OFF - Controller cannot participate in bus activities";
        case TWAI_STATE_RECOVERING:
            return "RECOVERING - Controller is undergoing bus recovery";
        default:
            return "UNKNOWN STATE";
    }
}
/*******************************************************************************/
/*                            GLOBAL VARIABLES                                 */
/*******************************************************************************/

/*******************************************************************************/
/*                        STATIC FUNCTION DEFINITIONS                          */
/*******************************************************************************/

static esp_err_t remote_frame_responder(twai_message_t *message)
{
    const uint8_t* response = NULL;

    /* Analyze the received message and prepare a response (for now just respond based on the message ID) */
    switch (message->identifier)
    {
        case ESP32_1_CAN_ID:
            response = response_templates[ESP32_1_CAN_ID_RESPONSE_ID];
            break;
        case ESP32_2_CAN_ID:
            response = response_templates[ESP32_2_CAN_ID_RESPONSE_ID];
            break;
        default: // Respond with all FFs for unsupported message IDs
            response = response_templates[DEFAULT_RESPONSE_ID];
            LOG("Unsupported message ID received\n");
            break;
    }

    /* Send the response under the same message ID */
    esp_err_t status = send_CAN_message(message->identifier, response, STANDARD_CAN_MAX_DATA_LENGTH);

    return status;
}

static void process_twai_alerts(uint32_t alerts) 
{
    if (alerts == 0) 
    {
        LOG("No TWAI alerts active\n");
    }
    else
    {
        LOG("TWAI Alerts detected:\n");
    
        /* Iterate through all defined alerts and check if each is present */
        uint32_t num_of_alerts_messages = sizeof(twai_alert_messages) / sizeof(twai_alert_messages[0]);
        for (uint32_t i = 0; i < num_of_alerts_messages; i++)
        {
            /* Check if this specific flag is set in the combined alerts value */
            if (alerts & twai_alert_messages[i].flag) 
            {
                LOG("  - %s\n", twai_alert_messages[i].message);
            }
        }
    }
}
/*******************************************************************************/
/*                        GLOBAL FUNCTION DEFINITIONS                          */
/*******************************************************************************/

esp_err_t init_twai(void) 
{
    esp_err_t status = ESP_OK;
    /*  Set the general configuration of TWAI to default values plus specify the CAN pins plus the mode.
        For the default values, see the TWAI_GENERAL_CONFIG_DEFAULT_V2 macro in twai.h
        The CAN pins are the TX and RX pins of the CAN/TWAI controller which connect to the CAN transceiver which connects to the CAN bus.
        For the modes see twai_mode_t - in NORMAL mode, the controller can send/receive/acknowledge messages. */
    twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    
    /*  Set the timing configuration of TWAI to default values plus specify the bit rate 
        Available bit rates are 25kbps, 50kbps, 100kbps, 125kbps, 250kbps, 500kbps, 800kbps, 1Mbps */
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_500KBITS();

    /*  Set the filter configuration of TWAI. In this case, accept all messages.
        This acceptance filter allows you to selectively receive only certain CAN messages based on their identifiers.
        For example .acceptance_code = (0x123 << 21), .acceptance_mask = ~(0x7FF << 21) would accept only messages with ID 0x123 
         We shift the value 21 bits because that's the location 11-bit message CAN ID in some TWAI register (bits 21-31) */
    twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    /*  Install TWAI driver using the previously set configurations. Required memory is allocated 
        and the driver is placed in the stopped state */
    status = twai_driver_install(&general_config, &timing_config, &filter_config);
    if (status == ESP_OK) 
    {
        LOG("TWAI driver installed successfully\n");
    } 
    else 
    {
        LOG("Failed to install TWAI driver. Error code: %d\n", status);
        return status;
    }

    /*  Start the TWAI driver. Puts the driver into the running state. This allows the TWAI driver to participate
        in TWAI bus activities such as transmitting/receiving messages. This can only be called when the 
        TWAI driver is in the stopped state. */
    status = twai_start();
    if (status == ESP_OK) 
    {
        LOG("TWAI driver started successfully\n");
    } 
    else 
    {
        LOG("Failed to start TWAI driver. Error code: %d\n", status);
        return status;
    }

    return status; // ESP_OK, otherwise it would have returned earlier with an error code
}

esp_err_t send_CAN_message(uint32_t message_id, const uint8_t *data, uint8_t data_length) 
{
    esp_err_t status = ESP_OK;
    twai_message_t message;

    if((data_length > 8) || (data == NULL))
    {
        LOG("Data length is greater than 8 bytes or data is NULL\n"); // For now we only support the standard CAN frame
        status = ESP_ERR_INVALID_SIZE;
    }
    else
    {
        /*  Formulate a TWAI message to transmit. The message contains the identifier, data length code, and data */
        message.identifier = message_id;
        message.data_length_code = data_length;

        /*  Populate the message buffer with the data provided by the user */
        LOG("Sending data...\n");
        for (uint8_t i = 0; i < data_length; i++)
        {
            message.data[i] = data[i];
            LOG("Data[%d] = 0x%X\n", i, message.data[i]);
        }

        /* Specify the TWAI flags for the message. The flags are used to specify the message type, such as standard or extended frame, 
        remote frame, single shot transmission, self reception request, and data length code non-compliant. */
#if (J1939_ENABLED == 1)
        message.extd = 1;         // 1 - Extended Frame Format (29bit ID) or 0 - Standard Frame Format (11bit ID)
#else
        message.extd = 0;         // 1 - Extended Frame Format (29bit ID) or 0 - Standard Frame Format (11bit ID)
#endif
        message.rtr = 0;          // 1 - Message is a Remote Frame or 0 - Message is a Data Frame
        message.ss = 0;           // 1 - Transmit as a Single Shot Transmission (the message will not be retransmitted upon error or arbitration loss)
        message.self = 0;         // 1 - Transmit as a Self Reception Request (the msg will be received by the transmitting device) or 0 for normal transmission
        message.dlc_non_comp = 0; // 1 - Message's Data length code is larger than 8 (this will break compliance with ISO 11898-1) or 0 the opposite

        /*  Transmit a CAN message. The message is copied to the driver's internal buffer.
            The driver will transmit the message as soon as the bus is available. */
        status = twai_transmit(&message, pdMS_TO_TICKS(1000));
        if (status == ESP_OK) 
        {
            LOG("Message with ID=0x%lX sent successfully\n", (unsigned long)message_id);
        } 
        else 
        {
            LOG("Failed to send message. Error code: %d\n", status);
        }
    }

    return status;
} 

esp_err_t receive_CAN_message(uint32_t* message_id, uint8_t* buffer, uint8_t* buffer_length)
{
    bool message_received = false;
    esp_err_t status = ESP_OK;
    twai_message_t message;

    /* Check for NULL pointers */
    if (buffer == NULL || buffer_length == NULL || message_id == NULL) 
    {
        LOG("Invalid parameters: buffer, buffer_length or message_id are NULL\n");
        status = ESP_ERR_INVALID_ARG;
    }
    else
    {
        /* Attempt to receive a CAN message */
        if (twai_receive(&message, pdMS_TO_TICKS(1000)) == ESP_OK) 
        {
            message_received = true;
            LOG("Message received with ID=0x%lX (Extended=%d, RTR=%d, SS=%d, Self=%d, DLC Non-Comp=%d)\n", 
                (unsigned long)message.identifier, message.extd, message.rtr, message.ss, message.self, message.dlc_non_comp);
                
            *message_id = message.identifier; // Store the received message ID
        } 
        else 
        {
            LOG("Failed to receive message\n");
            status = ESP_FAIL;
        }

        /* Process the received message if one was received */
        if(message_received)
        {
            /* Verify the length of the received data */
            if((message.data_length_code > 8) || (*buffer_length < message.data_length_code))
            {
                LOG("Data length %d is greater than 8 bytes or too big for the buffer of size %d\n", message.data_length_code, *buffer_length);
                status = ESP_ERR_INVALID_SIZE;
            }
            else
            {
                /*  Copy the received data to the buffer provided by the user */
                LOG("Receiving data...\n");
                for (uint8_t i = 0; i < message.data_length_code; i++)
                {
                    buffer[i] = message.data[i];
                    LOG("Data[%d] = 0x%X\n", i, buffer[i]);
                }
                *buffer_length = message.data_length_code; // Store the actual length of the data received
            }

            /* Handle remote frames (frames that request data) */
            if(message.rtr)
            {
                LOG("Received message is a Remote Frame. Responding with a Data Frame...\n");
                status = remote_frame_responder(&message);
            }
        }
    }
    
    return status;
}

bool monitor_CAN_bus(void)
{
    bool status = true;

    /* Read the alerts raised by the TWAI driver */
    uint32_t alerts = 0;
    esp_err_t alerts_status = twai_read_alerts(&alerts, pdMS_TO_TICKS(100));
    if (alerts_status == ESP_OK) 
    {
        /* Process and print all active alerts */
        process_twai_alerts(alerts);
    } 
    else if (alerts_status == ESP_ERR_TIMEOUT) 
    {
        /* This is normal - it means no alerts were generated during the timeout period */
        LOG("No TWAI alerts during monitoring period\n");
    }
    else 
    {
        LOG("Failed to read TWAI alerts. Error code: %d\n", alerts_status);
        status = false;
    }

    /* Read the status information of the TWAI driver */
    twai_status_info_t driver_status_info;
    esp_err_t twai_status = twai_get_status_info(&driver_status_info);
    if (twai_status == ESP_OK) 
    {
        LOG("TWAI Status Information:\n");
        LOG("  - TX Messages: %lu\n", driver_status_info.msgs_to_tx);
        LOG("  - RX Messages: %lu\n", driver_status_info.msgs_to_rx);
        LOG("  - TX Failed Count: %lu\n", driver_status_info.tx_failed_count);
        LOG("  - RX Missed Count: %lu\n", driver_status_info.rx_missed_count);
        LOG("  - RX Overrun Count: %lu\n", driver_status_info.rx_overrun_count);
        LOG("  - Arb Lost Count: %lu\n", driver_status_info.arb_lost_count);
        LOG("  - Bus Error Count: %lu\n", driver_status_info.bus_error_count);
        LOG("  - State: %s\n", twai_state_to_string(driver_status_info.state));
    } 
    else 
    {
        LOG("Failed to get TWAI status information. Error code: %d\n", twai_status);
        status = false;
    }
    
    return status;
}
    



