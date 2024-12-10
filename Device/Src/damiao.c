#include "damiao.h"


int16_t D4310Pitch_currnt[4];
extern  damiao_recieve damiao_recieve_pitch;
extern SBUS_Buffer SBUS;
float	uint_to_float(uint16_t x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
  
		
    uint16_t max_input = (1 << (bits)) - 1; // 2^(y-1) - 1 
		uint16_t	x_int1 = x_int;
		float   out = ((float)x_int / (float)max_input) * (x_max - x_min) + x_min;
    return  out;
}


int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits)-1 ) / span));
}


static CAN_TxHeaderTypeDef  damiao_tx_message;//发送数据的数据头
static uint8_t              damiao_can_send_data[8];//要发送的数据数组
void ctrl_damiao_motor( uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    // 数据归一化并转换为无符号整型
    pos_tmp = float_to_uint(_pos, -12.5, 12.5, 16);//注意所有调用的float_to_uint函数后三个参数均由调参软件上读出
    vel_tmp = float_to_uint(_vel, -30, 30, 12);    //如果更改会导致最后的解码错误
    kp_tmp = float_to_uint(_KP, 0, 500, 12);
    kd_tmp = float_to_uint(_KD, 0, 5, 12);
    tor_tmp = float_to_uint(_torq, -10, 10, 12);

    // CAN 数据帧配置
		uint32_t send_mail_box;
    damiao_tx_message.StdId = id;//查阅C620手册，ID为1-4时发送标识为0x200
    damiao_tx_message.IDE = CAN_ID_STD;
    damiao_tx_message.RTR = CAN_RTR_DATA;
    damiao_tx_message.DLC = 0x08;
    // 数据分配到 CAN 数据帧
    damiao_can_send_data[0] = (pos_tmp >> 8);
    damiao_can_send_data[1] = pos_tmp;
    damiao_can_send_data[2] = (vel_tmp >> 4);
    damiao_can_send_data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    damiao_can_send_data[4] = kp_tmp;
    damiao_can_send_data[5] = (kd_tmp >> 4);
    damiao_can_send_data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    damiao_can_send_data[7] = tor_tmp;
		
		HAL_CAN_AddTxMessage(&hcan1, &damiao_tx_message, damiao_can_send_data, &send_mail_box);
}


void enable_damiao_motor(uint16_t id)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

   
    // CAN 数据帧配置
		uint32_t send_mail_box;
    damiao_tx_message.StdId = id;
    damiao_tx_message.IDE = CAN_ID_STD;
    damiao_tx_message.RTR = CAN_RTR_DATA;
    damiao_tx_message.DLC = 0x08;
    // 数据分配到 CAN 数据帧
    damiao_can_send_data[0] = 0xFF;
    damiao_can_send_data[1] = 0xFF;
    damiao_can_send_data[2] = 0xFF;
    damiao_can_send_data[3] = 0xFF;
    damiao_can_send_data[4] = 0xFF;
    damiao_can_send_data[5] = 0xFF;
    damiao_can_send_data[6] = 0xFF;
    damiao_can_send_data[7] = 0xFC;
		
		HAL_CAN_AddTxMessage(&hcan1, &damiao_tx_message, damiao_can_send_data, &send_mail_box);
}

void disable_damiao_motor(uint16_t id)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

   
    // CAN 数据帧配置
		uint32_t send_mail_box;
    damiao_tx_message.StdId = id;//查阅C620手册，ID为1-4时发送标识为0x200
    damiao_tx_message.IDE = CAN_ID_STD;
    damiao_tx_message.RTR = CAN_RTR_DATA;
    damiao_tx_message.DLC = 0x08;
    // 数据分配到 CAN 数据帧
    damiao_can_send_data[0] = 0xFF;
    damiao_can_send_data[1] = 0xFF;
    damiao_can_send_data[2] = 0xFF;
    damiao_can_send_data[3] = 0xFF;
    damiao_can_send_data[4] = 0xFF;
    damiao_can_send_data[5] = 0xFF;
    damiao_can_send_data[6] = 0xFF;
    damiao_can_send_data[7] = 0xFD;
		
		HAL_CAN_AddTxMessage(&hcan1, &damiao_tx_message, damiao_can_send_data, &send_mail_box);
}



void ctrl_torq_damiao_motor( uint16_t id, float _torq)
{
    CAN_TxHeaderTypeDef  damiao_tx_message1;//发送数据的数据头
    uint8_t              damiao_can_send_data1[8];//要发送的数据数组  
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    // 数据归一化并转换为无符号整型
    pos_tmp = float_to_uint(0, -12.5, 12.5, 16);//注意所有调用的float_to_uint函数后三个参数均由调参软件上读出
    vel_tmp = float_to_uint(0, -30, 30, 12);    //如果更改会导致最后的解码错误
    kp_tmp = float_to_uint(0, 0, 500, 12);
    kd_tmp = float_to_uint(0, 0, 5, 12);
    tor_tmp = float_to_uint(_torq, -10, 10, 12);

    // CAN 数据帧配置
		uint32_t send_mail_box;
    damiao_tx_message1.StdId = id;//查阅C620手册，ID为1-4时发送标识为0x200
    damiao_tx_message1.IDE = CAN_ID_STD;
    damiao_tx_message1.RTR = CAN_RTR_DATA;
    damiao_tx_message1.DLC = 0x08;
    // 数据分配到 CAN 数据帧
    damiao_can_send_data1[0] = (pos_tmp >> 8);
    damiao_can_send_data1[1] = pos_tmp;
    damiao_can_send_data1[2] = (vel_tmp >> 4);
    damiao_can_send_data1[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    damiao_can_send_data1[4] = kp_tmp;
    damiao_can_send_data1[5] = (kd_tmp >> 4);
    damiao_can_send_data1[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    damiao_can_send_data1[7] = tor_tmp;
		
		HAL_CAN_AddTxMessage(&hcan1, &damiao_tx_message1, damiao_can_send_data1, &send_mail_box);
}

void ctrl_speed_damiao_motor( uint16_t id, float speed)
{
  float torq;
  torq = pid_output(&pid_D4310Yaw_speed,damiao_recieve_pitch.velocity,speed);
  ctrl_torq_damiao_motor(id,torq);
} 

void ctrl_position_damiao_motor( uint16_t id, float position)
{
  float speed;
  speed = (SBUS.Ch2-1024)*0.005f + pid_output(&pid_D4310Yaw_angle,damiao_recieve_pitch.position,position);
  ctrl_speed_damiao_motor(id,speed);



} 
