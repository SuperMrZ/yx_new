#include "math.h"
#include "stdio.h"
#include "main.h"

#define PI 3.141592653589793238462643383279
#define DEG_TO_RAD (PI/180.0)
#define RAD_TO_DEG (180.0/PI)

extern double yaw;
extern double dt;

void normalizeQuaternion(double q[4])//四元数归一化
{
    double norm =sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;

}

//更新四元数
void updateQuaternion(double q[4],double gyro[3],double dt)
{
  double p=gyro[0];
  double q_dot=gyro[1];
  double r=gyro[2];

  //当前四元数
  double q0=q[0],q1=q[1],q2=q[2],q3=q[3];

  //四元数微分公式
  double dq0=0.5 * (-q1 * p - q2 * q_dot - q3 * r);
  double dq1=0.5 * ( q0 * p + q2 * r - q3 * q_dot);
  double dq2=0.5 * ( q0 * q_dot - q1 * r + q3 * p);
  double dq3=0.5 * ( q0 * r + q1 * q_dot - q2 * p);

  //用欧拉法更新四元数
  q[0] += dq0 * dt;
  q[1] += dq1 * dt;
  q[2] += dq2 * dt;
  q[3] += dq3 * dt;

  //归一化四元数
  normalizeQuaternion(q);
}

//加速度矫正
void calculateAnglesFromAccel(double accel[3], double* roll, double* pitch) 
{
    double ax = accel[0];
    double ay = accel[1];
    double az = accel[2];

    // 计算Roll和Pitch
    //*roll = atan2(ay, az) * RAD_TO_DEG; // 转换为角度
    *roll = atan2(ay, az);
    //*pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    *pitch = atan2(-ax, sqrt(ay * ay + az * az)) ;

}

//四元数转欧拉角
void quaternionToEuler(double q[4], double* roll, double* pitch, double* yaw)
{
    double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    // 欧拉角公式
    //*roll = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
    *roll = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2)) ;
    //*pitch = asin(2.0 * (q0 * q2 - q3 * q1)) * RAD_TO_DEG;
    *pitch = asin(2.0 * (q0 * q2 - q3 * q1)) ;
    //*yaw = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
    //*yaw = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
}

void updateYawFromGyro(double  gyro_z,double* yaw)
{
    // 根据累计角速度更新Yaw角
    *yaw += gyro_z * dt;

    if(*yaw>PI)
    {
        *yaw-=2*PI;
    }
    if(*yaw<-PI)
    {
        *yaw+=2*PI;
    }

}
