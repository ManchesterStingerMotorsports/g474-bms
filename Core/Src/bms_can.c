/*
 * bms_can.c
 *
 *  Created on: Jun 8, 2025
 *      Author: amrlxyz
 */

#include "bms_can.h"
#include "uartDMA.h"
#include <string.h>
#include <stdbool.h>

FDCAN_TxHeaderTypeDef TxHeader;


#define BUFFER_LEN (7 * 16 + 64)       // TODO: Accurate buffer size
CanTxMsg txBuffer[BUFFER_LEN];
volatile uint32_t txBufferHeadIndex = 0;
volatile uint32_t txBufferTailIndex = 0;
volatile bool isBufferTransmitting = false;


void BMS_CAN_Config(void)
{
//  FDCAN_FilterTypeDef sFilterConfig;
//
//  /* Configure Rx filter */
//  sFilterConfig.IdType = FDCAN_STANDARD_ID;
//  sFilterConfig.FilterIndex = 0;
//  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
//  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
//  sFilterConfig.FilterID1 = 0x321;
//  sFilterConfig.FilterID2 = 0x7FF;
//  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /* Configure global filter:
//     Filter all remote frames with STD and EXT ID
//     Reject non matching frames with STD ID and EXT ID */
//    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
//    {
//        Error_Handler();
//    }

    /* Start the FDCAN module */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_FIFO_EMPTY, 0) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }

}


void static recursiveTransmit(void)
{
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1))
    {
        if (txBufferTailIndex >= txBufferHeadIndex)
        {
            return;
        }

        CanTxMsg msg = txBuffer[txBufferTailIndex];
        BMS_CAN_SendMsg(msg);

        txBufferTailIndex++;
    }
}


void BMS_CAN_Test(void)
{
    /* Prepare Tx Header */
    TxHeader.Identifier = 0xFF;
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = 8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    const uint32_t TOTAL_MSG_ID = 7 * 16;
    uint8_t TxData[8] = {0x00, 0x00, 0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF};
    static int16_t test_voltage = 3.7 * 1000;
    static int16_t test_temp    = 25 * 100;
    static bool    test_isDicharging = false;
    static bool    test_isFaultDetected = false;

    test_isDicharging    = !test_isDicharging;
    test_isFaultDetected = !test_isFaultDetected;

    isBufferTransmitting = false;       // Disable recursive callback in case some are still being sent

    /* Start the Transmission process */
    for (int i = 0; i < TOTAL_MSG_ID; i++)
    {
        TxData[0] = (uint8_t)(test_voltage & 0xFF);
        TxData[1] = (uint8_t)((test_voltage >> 8) & 0xFF);
        TxData[2] = (uint8_t)(test_temp & 0xFF);
        TxData[3] = (uint8_t)((test_temp >> 8) & 0xFF);
        TxData[4] = (uint8_t)((test_isDicharging << 0) | (test_isFaultDetected << 1));

        memcpy(txBuffer[i].data, TxData, 8);
        TxHeader.Identifier = BASE_CAN_ID + i;
        txBuffer[i].header = TxHeader;
    }

    txBufferHeadIndex = TOTAL_MSG_ID;
    txBufferTailIndex = 0;
    isBufferTransmitting = true;

    recursiveTransmit();
}


void BMS_CAN_SendMsg(CanTxMsg msg)
{
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &msg.header, msg.data) != HAL_OK)
    {
        /* Transmission request Error */
        Error_Handler();
    }
}


void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &hfdcan1)
    {
        if (txBufferTailIndex >= txBufferHeadIndex || isBufferTransmitting == false)
        {
            isBufferTransmitting = false;
            return;
        }
        else
        {
            recursiveTransmit();
        }
    }
}


void static CAN_AbortTx()
{
    HAL_FDCAN_AbortTxRequest(&hfdcan1, FDCAN_TX_BUFFER0);
    HAL_FDCAN_AbortTxRequest(&hfdcan1, FDCAN_TX_BUFFER1);
    HAL_FDCAN_AbortTxRequest(&hfdcan1, FDCAN_TX_BUFFER2);
}



void BMS_CAN_SendBuffer(CanTxMsg* msgArr, uint32_t len)
{
    if (len > BUFFER_LEN)
    {
        printfDma("Error CANTX: len larger than buffer \n");
        return;
    }

    if (isBufferTransmitting)
    {
        isBufferTransmitting = false;       // Disable recursive callback in case some are still being sent
        CAN_AbortTx();
        printfDma("Error CANTX: previous buffer tx overwritten \n");
    }

    memcpy(txBuffer, msgArr, len * sizeof(CanTxMsg));
    txBufferHeadIndex = len;
    txBufferTailIndex = 0;
    isBufferTransmitting = true;

    recursiveTransmit();
}

ChargerStatus chargerStatus = {0};

/**
 * @brief Parses a CAN message received from the charger.
 *
 * This function decodes the data payload of a CAN message with the
 * charger's status ID and populates a ChargerStatus struct.
 * The data mapping is based on a common protocol:
 * - Bytes 0-1: Output Voltage (0.1V/bit, little-endian)
 * - Bytes 2-3: Output Current (0.1A/bit, little-endian)
 * - Byte 5: Status flags
 *
 * @param data can message data.
 */
void static parse_charger_status(uint8_t* data)
{
    // Unpack the data from the byte array.
    // Voltage (2 bytes, little-endian, 0.1V resolution)
    float raw_voltage = (data[0] << 8) | data[1];
    chargerStatus.output_voltage = raw_voltage / 10;

    // Current (2 bytes, little-endian, 0.1A resolution)
    float raw_current = (data[2] << 8) | data[3];
    chargerStatus.output_current = raw_current / 10;

    // Status Flags (1 byte)
    chargerStatus.hardware_fault        = (data[4] >> 0) & 0x01;
    chargerStatus.over_temp_fault       = (data[4] >> 1) & 0x01;
    chargerStatus.input_voltage_fault   = (data[4] >> 2) & 0x01;
    chargerStatus.charging_state        = (data[4] >> 3) & 0x01;

    return;
}

/**
 * @brief Transmits a configuration message to the charger.
 *
 * This function takes charging parameters, packs them into a CAN message
 * payload, and sends it using the defined charger configuration ID.
 * The data mapping is based on a common protocol:
 * - Bytes 0-1: Target Voltage (0.1V/bit, little-endian)
 * - Bytes 2-3: Max Current (0.1A/bit, little-endian)
 * - Byte 4: Charging Enable (0x00 = Disable, 0x01 = Enable)
 */
void BMS_CAN_GetChargerMsg(const ChargerConfiguration* config, uint8_t* data)
{
    // Pack Target Voltage (0.1V resolution)
    uint16_t raw_voltage = (uint16_t)(config->target_voltage * 10);
    data[0] = (uint8_t)((raw_voltage >> 8) & 0xFF);
    data[1] = (uint8_t)((raw_voltage >> 0) & 0xFF);

    // Pack Max Current (0.1A resolution)
    uint16_t raw_current = (uint16_t)(config->max_current * 10);
    data[2] = (uint8_t)((raw_current >> 8) & 0xFF);
    data[3] = (uint8_t)((raw_current >> 0) & 0xFF);

    // Pack Charging Enable flag
    data[4] = config->disable_charging ? 0x01 : 0x00;

    // Bytes 5, 6, 7 are unused but sent as 0.
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;

    // Send the message over the CAN bus.
    return;
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan == &hfdcan1)
    {
        if (RxFifo0ITs == FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
        {
            FDCAN_RxHeaderTypeDef rxHeader;
            uint8_t rxData[8];
            HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData);

            if (rxHeader.Identifier == CHARGER_STATUS_CAN_ID)
            {
                parse_charger_status(rxData);
            }
        }
    }
}



