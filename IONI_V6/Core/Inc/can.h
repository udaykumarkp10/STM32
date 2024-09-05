/*
 * can.h
 *
 *  Created on: Jul 18, 2024
 *      Author: udaykumar
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "main.h"
#include <stdint.h>
#include <string.h>

extern CAN_HandleTypeDef hcan;
extern CAN_RxHeaderTypeDef rxHeader;
extern uint8_t rxData[8];
extern uint8_t received_flag;

// Define BytesUnion as a global variable
typedef union {
    uint64_t value;
    struct {
        uint32_t low;
        uint32_t high;
    };
    struct {
        uint16_t s0;
        uint16_t s1;
        uint16_t s2;
        uint16_t s3;
    };
    uint8_t bytes[8];
    uint8_t byte[8];
} BytesUnion;

extern BytesUnion txDataUnion;
extern BytesUnion rxDataUnion;

void CAN1_Tx(uint32_t id, uint8_t* data, uint8_t len);
void SendFloatData(uint32_t id, float value);
void SendPosition(float position);
void SendSpeed(float speed);
uint8_t CheckCANMessage(void);
void CAN1_TxFloat(uint32_t id, float value);


#endif /* INC_CAN_H_ */
