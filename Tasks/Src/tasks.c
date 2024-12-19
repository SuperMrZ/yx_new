#include "tasks.h"
#include "stdbool.h"

 int16_t down_MEG[4];
uint16_t down_MEG2[4];

void M3508Load_Move();

extern SBUS_Buffer SBUS;
extern INS_t INS;



int16_t jishu;



SendPacket Up_SendPacket;


/*第一个任务，用于计算数据*/
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  INS_Init();

  for(;;)
  {

    /*imu解算开始*/
    INS_Task();

    /*imu解算结束*/

    /*电机电流控制计算开始*/

    if(SBUS.SF == 1695)
    {
      if(SBUS.SG ==353 && back_flag ==1)
      {
        cmd_M2006pushrop_speed(pushrot_M2006_speedTarget);

      }
      if(SBUS.SG ==353 && back_flag ==0)
      {
        M2006Pushrop_currnt =0;

      }
      if(SBUS.SG !=353)
      {
        M2006PushRop_Move();
      }
      
      M3508Load_Move();
      //  cmd_M3508Laod_speed(5*(SBUS.Ch1 - 1024));



      cmd_M3508Friction_speed(M3508Friction_speedTarget);

        cmd_M6020Yaw_angle(Yaw6020_positiontarget);
      // cmd_M6020Yaw_speed(SBUS.Ch1-1024);


    }

   /*电机电流控制计算结束*/


    osDelay(1);   
  }
  /* USER CODE END StartDefaultTask */
}


/*第二个任务，用于发送数据*/
void Sendmessage(void *argument)
{
  while(1)
  {
   Cmd_gamble2006_currnt();
   Cmd_gamble3508_currnt();
   Down_SendMEG();
   Down_SendMEG2();

    jishu ++;
  //  ctrl_position_damiao_motor(0x01,YAW_D4310_positiontarget);
    if(jishu >10)
    {
      up_send();
      jishu =0;

    }
    
  //   Cmd_gamble6020_currnt();
    osDelay(1);
  }
}



void M3508Load_Move()
{
  if(Load_M3508_positionTarget>8192)
  {
    cmd_M3508Laod_speed(1000);
    if(M3508Load.last_ecd-M3508Load.ecd > 4096)
    {
      Load_M3508_positionTarget=Load_M3508_positionTarget-8192;
    }
    M3508Load.last_ecd = M3508Load.ecd;
  }

  if(Load_M3508_positionTarget<8192)
  {
    cmd_M3508Load_angle(Load_M3508_positionTarget);
  }
}


void M2006PushRop_Move()
{
  if(pushrot_M2006_positionTarget > 8192)
  {
    cmd_M2006pushrop_speed(4000);
    if(M2006Pushrop.last_ecd-M2006Pushrop.ecd>4096)
    {
      pushrot_M2006_positionTarget -= 8192;
    }
    M2006Pushrop.last_ecd = M2006Pushrop.ecd;
  }
  if(pushrot_M2006_positionTarget < 0)
  {
    cmd_M2006pushrop_speed(-4000);
    if(M2006Pushrop.ecd - M2006Pushrop.last_ecd>4096)
    {
      pushrot_M2006_positionTarget += 8192;
    }
    M2006Pushrop.last_ecd = M2006Pushrop.ecd;
  } 

  if(0 < pushrot_M2006_positionTarget && pushrot_M2006_positionTarget <8192)
  {
    cmd_M2006pushrop_angle(pushrot_M2006_positionTarget);
  }
}


void Down_SendMEG()
{
  down_MEG[0]=SBUS.Ch3;
  down_MEG[1]=SBUS.Ch4;
  down_MEG[2]=SBUS.Ch1;
  down_MEG[3]=SBUS.SA;
  CAN_SendData(2,0x123,down_MEG);
} 


uint32_t temp;


void Down_SendMEG2()
{
   temp = *(uint32_t*)&INS.Yaw;

  //  temp = &INS.Yaw;
  down_MEG2[0] = (uint16_t)(temp >> 16);  // 获取高16位
  down_MEG2[1] = (uint16_t)(temp & 0xFFFF);  // 获取低16位
  down_MEG2[2] = (uint16_t)(M6020Yaw.ecd);
  down_MEG2[3] = 0;
  
  CAN_SendData(2,0x124,down_MEG2);

}


 ReceivePacket Up_ReceivePacket;


void up_receive(uint8_t* Buf, uint32_t *Len)
{
  Up_ReceivePacket.data_valid = false;

    // 检查数据长度是否为19字节


    // 验证CRC16校验和
    // if (!Verify_CRC16_Check_Sum(Buf, Len)) {
    //     return;  // CRC校验失败
    // }

    // // 检查包头
    // if (Buf[0] != 0xA5) {
    //     return;  // 包头错误
    // }

    // 解析第二个字节中的bit位
    uint8_t status_byte = Buf[1];
    Up_ReceivePacket.is_tracking = (status_byte & 0x01);          // 最低位
    Up_ReceivePacket.can_shoot = (status_byte & 0x02) >> 1;      // 第二位
    Up_ReceivePacket.armor_id = (status_byte & 0x3C) >> 2;       // 中间四位
    Up_ReceivePacket.bullet_freq = (status_byte & 0xC0) >> 6;    // 最高两位

    // 解析pitch角度（4字节浮点数）
    float* pitch_ptr = (float*)&Buf[2];
    Up_ReceivePacket.pitch_angle = *pitch_ptr;

    // 解析yaw角度（4字节浮点数）
    float* yaw_ptr = (float*)&Buf[6];
    Up_ReceivePacket.yaw_angle = *yaw_ptr;

    // checksum在最后2个字节，但已经通过CRC16验证过了
    
    Up_ReceivePacket.data_valid = true;  // 标记数据为有效

}



SendPacket a2;
int8_t up_send_data[19];

void up_send()
{


a2.detect_color =1;
a2.mode = 3 ;
a2.reserved = 0;
a2.reset_tracker =0 ;

up_send_data[0] = 0x5A;

float data1 =(-INS.Pitch);

float data2 =INS.Yaw*0.01745329;

float data3 = 0.0f;
int8_t data4 = 0;
int8_t data5 = 0;
int8_t data6 = 0;




 uint16_t offset = 1; // 从 t[1] 开始
 memcpy(up_send_data + offset, &a2, sizeof(a2));
 offset += sizeof(a2);

 memcpy(up_send_data + offset, &data1, sizeof(data1));
 offset += sizeof(data1);
 memcpy(up_send_data + offset, &data2, sizeof(data2));
 offset += sizeof(data2);
 memcpy(up_send_data + offset, &data3, sizeof(data3));
 offset += sizeof(data3);

 memcpy(up_send_data + offset, &data4, sizeof(data4));
 offset += sizeof(data4);
 memcpy(up_send_data + offset, &data5, sizeof(data5));
 offset += sizeof(data5);
 memcpy(up_send_data + offset, &data6, sizeof(data6));
 offset += sizeof(data6);

Append_CRC16_Check_Sum(up_send_data,19);




 CDC_Transmit_FS(up_send_data,sizeof(up_send_data));  

}


