/*
 * bms_can.h
 *
 *  Created on: Jun 8, 2025
 *      Author: amrlxyz
 */

#ifndef INC_BMS_CAN_H_
#define INC_BMS_CAN_H_

#include "main.h"


#define INC_BMS_CAN_H_

#define BASE_CAN_ID 0xB000


typedef struct
{
    uint8_t data[8];
    FDCAN_RxHeaderTypeDef header;
} CanRxMsg;


typedef struct
{
    uint8_t data[8];
    FDCAN_TxHeaderTypeDef header;
} CanTxMsg;


void BMS_CAN_Config(void);

void BMS_CAN_SendBuffer(CanTxMsg* msgArr, uint32_t len);




void BMS_CAN_Test(void);
void BMS_CAN_SendMsg(CanTxMsg msg);

#endif /* INC_BMS_CAN_H_ */
