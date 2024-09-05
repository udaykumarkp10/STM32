/*
 * can.c
 *
 *  Created on: Jul 18, 2024
 *      Author: udaykumar
 */

#include "can.h"

/*Set target position - 0x44 (Incremental)
Get target position - 0x45 (ABS)
start homing - 0x50
stop homing - 0x51
stop - 0x52
quick stop - 0x00*/

// External variables
extern CAN_HandleTypeDef hcan;
CAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];
uint8_t received_flag = 0;


BytesUnion txDataUnion;
BytesUnion rxDataUnion;

///**
//  * @brief CAN Initialization Function
//  * @param None
//  * @retval None
//  */
//void MX_CAN_Init(void)
//{
//  /* Configure the CAN peripheral */
//  hcan.Instance = CAN1;
//  hcan.Init.Prescaler = 18;
//  hcan.Init.Mode = CAN_MODE_NORMAL; // Change to normal mode for real communication
//  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
//  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
//  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
//  hcan.Init.TimeTriggeredMode = DISABLE;
//  hcan.Init.AutoBusOff = DISABLE;
//  hcan.Init.AutoWakeUp = DISABLE;
//  hcan.Init.AutoRetransmission = DISABLE;
//  hcan.Init.ReceiveFifoLocked = DISABLE;
//  hcan.Init.TransmitFifoPriority = DISABLE;
//  if (HAL_CAN_Init(&hcan) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  // Configure CAN filters
//  CAN_FilterTypeDef canfilterconfig;
//  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
//  canfilterconfig.FilterBank = 0;
//  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//  canfilterconfig.FilterIdHigh = 0x610 << 5;
//  canfilterconfig.FilterIdLow = 0x0000;
//  canfilterconfig.FilterMaskIdHigh = 0x610 << 5;
//  canfilterconfig.FilterMaskIdLow = 0x0000;
//  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
//  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
//  canfilterconfig.FilterBank = 0;
//  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
//
//  // Start the CAN peripheral
//  if (HAL_CAN_Start(&hcan) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  // Activate CAN RX notification
//  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
//}


void SendPosition(float position) {
    CAN1_TxFloat(0x590, position);
}

void CAN1_TxFloat(uint32_t id, float value) {
    CAN_TxHeaderTypeDef txHeader;
    uint32_t txMailbox;
    uint8_t data[sizeof(float)];

    txHeader.DLC = sizeof(float);
    txHeader.StdId = id;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.TransmitGlobalTime = DISABLE;

    // Copy float value into data buffer
    memcpy(data, &value, sizeof(float));

    if (HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &txMailbox) != HAL_OK) {
        // Transmission request Error
        Error_Handler();
    }
}

//void CAN1_Tx(uint32_t id, uint8_t* data, uint8_t len) {
//    CAN_TxHeaderTypeDef txHeader;
//    uint32_t txMailbox;
//
//    txHeader.DLC = len;
//    txHeader.StdId = id;
//    txHeader.IDE = CAN_ID_STD;
//    txHeader.RTR = CAN_RTR_DATA;
//    txHeader.TransmitGlobalTime = DISABLE;
//
//    if (HAL_CAN_AddTxMessage(&hcan, &txHeader, data, &txMailbox) != HAL_OK) {
//        // Transmission request Error
//        Error_Handler();
//    }
//}
//
//void SendFloatData(uint32_t id, float value) {
//    BytesUnion dataUnion;
//    memcpy(dataUnion.bytes, &value, sizeof(float));
//    CAN1_Tx(id, dataUnion.bytes, sizeof(float));
//}
//
//void SendPosition(float position) {
//    SendFloatData(0x590, position);
//}

void SendSpeed(float speed) {
    SendFloatData(0x590, speed);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxDataUnion.bytes) != HAL_OK) {
        // Reception Error
        Error_Handler();
    } else {
        // Process received message
        received_flag = 1;
    }
}

uint8_t CheckCANMessage(void) {
    return received_flag;
}
