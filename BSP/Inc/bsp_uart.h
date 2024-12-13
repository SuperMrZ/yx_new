#ifndef __BSP_USART_H__
#define __BSP_USART_H__
#include "usart.h"
#include "main.h"
#include "motorCmd.h"
#include "bsp_can.h"




// typedef struct{//25ֽSBUS洢ṹ
// 	uint8_t Start;
// 	uint16_t Ch1;
// 	uint16_t Ch2;
// 	uint16_t Ch3;
// 	uint16_t Ch4;
// 	uint16_t Ch5;
// 	uint16_t Ch6;
// 	uint16_t Ch7;
// 	uint16_t Ch8;
// 	uint16_t Ch9;
// 	uint16_t Ch10;
// 	uint16_t Ch11;
// 	uint16_t Ch12;
// 	uint16_t Ch13;
// 	uint16_t Ch14;
// 	uint16_t Ch15;
// 	uint16_t Ch16;
// 	uint8_t Flag;
// 	uint8_t End;
	
// 	uint16_t SF;
// 	uint16_t SF_last;
// 	uint16_t SH;
// 	uint16_t SH_last;
// 	uint16_t SE;
// 	uint16_t SE_last;
// 	uint16_t SG;
// 	uint16_t SG_last;

// }SBUS_Buffer;


typedef struct {
    int8_t bullet;
    int8_t pushrot_position;
} gambleState;

extern gambleState gamble_state;
extern int32_t pushrot_M2006_positionTarget;
extern int16_t pushrot_M2006_speedTarget;
extern int32_t Load_M3508_positionTarget;
extern int16_t Load_M3508_speedTarget;
extern int16_t M3508Friction_speedTarget[3];
extern float YAW_D4310_positiontarget;
extern int16_t YawPitch6020_speedtarget;
extern float Yaw6020_positiontarget;
extern int8_t back_flag;

 

void remoteDecode();

#endif
