#include "tasks.h"

 
#define ALPHA 0.85
acc_raw_data_t acc_raw_data={0,0,0};
gyro_raw_data_t gyro_raw_data={0,0,0};
float *time;
float *temp;
double quaternion[4]={1.0 , 0.0 , 0.0 , 0.0};
double roll=0.0,pitch=0.0,yaw=0.0;
double dt=0.133;
double gyro[3]={0.0,0.0,0.0};
double accel[3]={0.0,0.0,0.0}; 


void pushrop_Move();
void M3508Load_Move();

extern SBUS_Buffer SBUS;


void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
   (void)argument;
   bmi088_error_e BMI088_INIT();
   BMI088_CONF_INIT();

  for(;;)
  {
    // pushrop_Move(pushrot_M2006_positionTarget);
    osDelay(1);
    ReadAccData(&acc_raw_data);
    ReadGyroData(&gyro_raw_data);
    gyro[0]=gyro_raw_data.roll;
    gyro[1]=gyro_raw_data.pitch;
    gyro[2]=gyro_raw_data.yaw;
    accel[0]=acc_raw_data.x;
    accel[1]=acc_raw_data.y;
    accel[2]=acc_raw_data.z; 
    updateQuaternion(quaternion,gyro,dt);
    quaternionToEuler(quaternion,&roll,&pitch,&yaw);
    calculateAnglesFromAccel(accel,&roll,&pitch);
    yaw += ALPHA*gyro[2]*dt;
    ReadAccSensorTime(time);
    ReadAccTemperature(temp);
    if(SBUS.SF == 1695)
    {

    M3508Load_Move(Load_M3508_positionTarget);
    cmd_M3508Friction_speed(M3508Friction_speedTarget);
    cmd_M2006pushrop_speed(Load_M3508_speedTarget);
    cmd_M6020Yaw_angle(Yaw6020_positiontarget);
    }




    osDelay(1);   
  }
  /* USER CODE END StartDefaultTask */
}

void Sendmessage(void *argument)
{
  while(1)
  {
   Cmd_gamble2006_currnt();
   Cmd_gamble3508_currnt();
    
   // osDelay(1);
    ctrl_position_damiao_motor(0x01,YAW_D4310_positiontarget);
    
    osDelay(1);
    Cmd_gamble6020_currnt();
    osDelay(1);
  }
}







void pushrop_Move()
{
  if(pushrot_M2006_positionTarget>8192)
  {
    cmd_M2006pushrop_speed(2000);
    if(M2006Pushrop.last_ecd-M2006Pushrop.ecd > 4096)
    {
      pushrot_M2006_positionTarget = pushrot_M2006_positionTarget - 8192;
    }
    M2006Pushrop.last_ecd = M2006Pushrop.ecd;

  }
    if(pushrot_M2006_positionTarget<0)
  {
    cmd_M2006pushrop_speed(-2000);
    if(M2006Pushrop.ecd-M2006Pushrop.last_ecd > 4096)
    {
      pushrot_M2006_positionTarget = pushrot_M2006_positionTarget + 8192;
    }
    M2006Pushrop.last_ecd = M2006Pushrop.ecd;

  }


  if(pushrot_M2006_positionTarget<8192 && pushrot_M2006_positionTarget>0)
  {
    cmd_M2006pushrop_angle(pushrot_M2006_positionTarget);
  }
}


void M3508Load_Move()
{
  if(Load_M3508_positionTarget>8192)
  {
    cmd_M3508Laod_speed(500);
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