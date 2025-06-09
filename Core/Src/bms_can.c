/*
 * bms_can.c
 *
 *  Created on: Jun 8, 2025
 *      Author: amrlxyz
 */

#include "bms_can.h"
#include <string.h>

FDCAN_TxHeaderTypeDef TxHeader;


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

uint32_t id = 0xB000;

void BMS_CAN_Test(void)
{
    /* Prepare Tx Header */
    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = 8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    CanTxMsg msg;
    uint8_t TxData[8] = {0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF};
    static int16_t test_val = 0;

    /* Start the Transmission process */
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1))
    {
        test_val++;

        TxData[0] = (uint8_t)(test_val & 0xFF);
        TxData[1] = (uint8_t)((test_val >> 8) & 0xFF);

        memcpy(msg.data, TxData, sizeof(TxData));
        TxHeader.Identifier = id;
        msg.header = TxHeader;
        BMS_CAN_SendMsg(msg);

        id++;
        if (id >= 0xB000 + 16*7)
        {
            id = 0xB000;
            return;
        }
    }
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
//        while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1))
//        {
//            uint8_t TxData[8] = {0};
//            TxData[1] = 'A';
//            TxData[2] = 'B';
//            TxData[3] = 'C';
//            TxData[4] = 'D';
//            if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
//            {
//                /* Transmission request Error */
//                Error_Handler();
//            }
//        }
        if (id != 0xB000)
        {
            BMS_CAN_Test();
        }
    }
}


