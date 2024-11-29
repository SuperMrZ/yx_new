#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__
#include "main.h"
#include "can.h"


typedef struct {
	CAN_RxHeaderTypeDef header;
	uint8_t 			data[8];
} CAN_RxFrameTypeDef;

typedef struct {
	CAN_TxHeaderTypeDef header;
	uint8_t				data[8];
} CAN_TxFrameTypeDef;


void BspCan1Init();
void BspCan2Init();
uint8_t CAN_SendData(int8_t can, uint32_t stdId, int16_t *dat);


#endif
