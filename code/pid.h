#ifndef __PID_H 
#define __PID_H

#include "encoder.h"

#define bang_error_max  200
#define bang_out_max    


typedef struct{
  float Kp;
  float Ki;
  float Kd;
  float Kp2;
  float Kd2;
  float p_out;
  float i_out;
  float d_out;
  float d2_out;
  float Err;
  float Last_Err;       // 上次误差
  float Previous_Err;   // 上上次误差
  float Output;
  float Target;
  float Measure;    
}RS_Incremental_PID;

void PID_Init(void);
float IncrementalPID_L(RS_Incremental_PID *pid ,int16 target_speed, int16 measured_value );
float IncrementalPID_R(RS_Incremental_PID *pid ,int16 target_speed, int16 measured_value );
float Position_PID (RS_Incremental_PID *pid , int16 error , int16 gyro_z);
float Angle_PID(RS_Incremental_PID *pid ,float target_angle, float measured_angle );
float w_PID(RS_Incremental_PID *pid ,float target_w, float measured_w );
float angle(RS_Incremental_PID *pid , int16 error);
float Angular_V(RS_Incremental_PID *pid, int16 angle_output) ;
int Abs(int x);

extern RS_Incremental_PID pid_L;
extern RS_Incremental_PID pid_R;
extern RS_Incremental_PID turn_pid;
extern RS_Incremental_PID pid_angle;
extern RS_Incremental_PID pid_w;
extern RS_Incremental_PID angle_pid;   //new
extern RS_Incremental_PID Angular_pid; 
extern int16 pwm_l ;
extern int16 pwm_r ;

extern int16 turn_out;

#endif
