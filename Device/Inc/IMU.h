#ifndef _IMU_H
#define _IMU_H

void normalizeQuaternion(double q[4]);
void updateQuaternion(double q[4],double gyro[3],double dt);
void calculateAnglesFromAccel(double accel[3], double* roll, double* pitch) ;
void quaternionToEuler(double q[4], double* roll, double* pitch, double* yaw);
void updateYawFromGyro(double  gyro_z,double* yaw);

#endif
