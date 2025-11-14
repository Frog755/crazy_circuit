#ifndef _GRYO_H_
#define _GRYO_H_
#include "zf_common_headfile.h"

#define delta_T 0.001f //1ms

extern float ang_z;
extern int32_t GyroOn;
extern uint8 GyroINT;
extern float current_yaw;
extern float lv_gyro_z;
extern float calibratedGyro_Z1;
float calibrateZ_Gyro(float rawZRate);
float GetFusedZAngle(void);//�������˲���Ԫ������
float GetContinuousZAngle(void);
void calibrateGyro(void);//����Ư
void ResetYawZero(void);//����
float GetCurrentYaw(void);
void GyroResolve(void);


#endif

