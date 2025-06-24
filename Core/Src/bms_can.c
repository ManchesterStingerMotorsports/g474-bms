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


#define BUFFER_LEN (7 * 16 + 32)       // TODO: Accurate buffer size
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
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

    /* Start the FDCAN module */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_FIFO_EMPTY, 0) != HAL_OK)
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
        if (txBufferTailIndex >= txBufferHeadIndex)
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


void BMS_CAN_SendBuffer(CanTxMsg* msgArr, uint32_t len)
{
    if (len > BUFFER_LEN)
    {
        printfDma("Error CANTX: len larger than buffer \n");
        return;
    }
    if (isBufferTransmitting)
    {
        printfDma("Error CANTX: previous buffer tx overwritten \n");
    }

    isBufferTransmitting = false;       // Disable recursive callback in case some are still being sent
    memcpy(txBuffer, msgArr, len * sizeof(CanTxMsg));
    txBufferHeadIndex = len;
    txBufferTailIndex = 0;
    isBufferTransmitting = true;

    recursiveTransmit();
}




