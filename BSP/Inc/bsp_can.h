#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__
#include "main.h"
#include "can.h"
#include "damiao.h"


typedef struct {
	CAN_RxHeaderTypeDef header;
	uint8_t 			data[8];
} CAN_RxFrameTypeDef;

typedef struct {
	CAN_TxHeaderTypeDef header;
	uint8_t				data[8];
} CAN_TxFrameTypeDef;

typedef struct {
	int16_t ecd;
	int16_t speed_rpm;
	int16_t given_current;
	int8_t temperate;
	int16_t last_ecd;
} motorReceiveInfo;


void BspCan1Init();
void BspCan2Init();


uint8_t CAN_SendData(int8_t can, uint32_t stdId, int16_t *dat);


#endif
