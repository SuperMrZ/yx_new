#ifndef  TASKS_H
#define TASKS_H

#include "usart.h"
#include "bsp_can.h"
#include "damiao.h"
#include "main.h"
#include "bsp_uart.h"
#include "motorCmd.h"
#include "ins_task.h"
#include "crc.h"
#include "usbd_cdc_if.h"
#include "stdbool.h"

typedef struct
{
    uint8_t detect_color:1;
    bool reset_tracker:1;
    uint8_t mode:3;
    uint8_t reserved:3;
}SendPacket __attribute__((packed));

typedef struct  
{
    bool is_tracking;          // 是否识别到目标
    bool can_shoot;           // 是否可以开火
    uint8_t armor_id;         // 装甲板ID
    uint8_t bullet_freq;      // 射频
    float pitch_angle;        // pitch角度
    float yaw_angle;          // yaw角度
    bool data_valid;          // 数据是否有效
}ReceivePacket ;


extern SendPacket Up_SendPacket;
extern  ReceivePacket Up_ReceivePacket;



void up_receive(uint8_t* Buf, uint32_t *Len);

 void StartTask02(void *argument);
 void Sendmessage(void *argument);




#endif


