#include "pid.h"
#include "Gyro.h"
int16 pwm_l = 0;
int16 pwm_r = 0;

RS_Incremental_PID pid_L;
RS_Incremental_PID pid_R;
RS_Incremental_PID turn_pid;
RS_Incremental_PID pid_angle;
RS_Incremental_PID Angular_pid;


RS_Incremental_PID pid_w;
RS_Incremental_PID angle_pid;
/*
 * 函数名称: void PID_Init(void)
 * 功能描述: PID相关参数初始化
 * 输入参数: 无
 * 输出参数: 无
 * 使用说明: 在系统初始化时调用一次
 */
void PID_Init(void)
{
    // 左轮速度PID参数初始化 - 降低Kp，适当降低Ki，减小直线振荡
    pid_L.Kp =11.5;    // 比例系数
    pid_L.Ki = 0.5;    // 积分系数
    pid_L.Kd = 0.5;    // 微分系数
    pid_L.Target = 40; // 速度目标值

    // 右轮速度PID参数初始化 - 与左轮保持一致以保证走直线一致性
    pid_R.Kp = 11.5;
    pid_R.Ki = 0.5;
    pid_R.Kd = 0.5;
    pid_R.Target = 40;

    // 角度PID参数初始化（用于方向控制等）- 可适当增加微分，抑制方向变化抖动
    angle_pid.Kp = 2.8;    // 比例系数
    angle_pid.Kd = 2.09;    // 微分系数
    angle_pid.Kp2 = 0.0038;

    // 角速度PID参数初始化（稳健转向，无需大改）
    Angular_pid.Kp = 0.237;
    Angular_pid.Kd = 0.95;

    // 备用角度PID参数初始化（可用于特殊情况如转弯等）
    pid_angle.Kp = 1.5;    // 比例系数
    pid_angle.Kd = 2.0;    // 微分系数

    // 角速度补偿用PID参数初始化
    pid_w.Kp = -0.1;       // 比例系数
    pid_w.Kd = 0;          // 微分系数
}



float IncrementalPID_L (RS_Incremental_PID *pid ,int16 target_speed, int16 measured_value )
{
  pid->Err = target_speed - measured_value ;
  
  pid->p_out = pid->Kp * (pid->Err - pid->Last_Err);
  pid->i_out = pid->Ki * pid->Err;
  pid->d_out = pid->Kd * (pid->Err - 2*pid->Last_Err + pid->Previous_Err);
  
  pid->Output += pid->p_out + pid->i_out + pid->d_out; 
  if(pid->Output > 6666)
  {
    pid->Output=6666;
  }
  if(pid->Output<-6666)
  {
    pid->Output=-6666;
  }
  
  pid->Previous_Err = pid->Last_Err;
  pid->Last_Err =  pid->Err;
  
  return pid->Output;
  
}

float IncrementalPID_R (RS_Incremental_PID *pid ,int16 target_speed, int16 measured_value )
{
  pid->Err = target_speed - measured_value ;
  
  pid->p_out = pid->Kp * (pid->Err - pid->Last_Err);
  pid->i_out = pid->Ki * pid->Err;
  pid->d_out = pid->Kd * (pid->Err - 2*pid->Last_Err + pid->Previous_Err);
  
  pid->Output += pid->p_out + pid->i_out + pid->d_out; 
  
  if(pid->Output > 6666)
  {
    pid->Output=6666;
  }
  if(pid->Output<-6666)
  {
    pid->Output=-6666;
  }
  
  pid->Previous_Err = pid->Last_Err;
  pid->Last_Err =  pid->Err;
  
  return pid->Output;
}

float Position_PID (RS_Incremental_PID *pid , int16 error , int16 gyro_z)
{
  pid->Err = error;
  pid->p_out = pid->Kp * pid->Err + Abs((int)(pid->Err)) * pid->Err * pid->Kp2;
  pid->d_out = pid->Kd * (pid->Err - pid->Last_Err) ;
  pid->d2_out = turn_pid.Kd2 * gyro_z;
  pid->Output = pid->p_out + pid->d_out + pid->d2_out; 
  pid->Last_Err =  pid->Err;
  
  return pid->Output;
}

float angle(RS_Incremental_PID *pid , int16 error) {
  
    pid->Err = error;
  
    
    
    float_t dynamic_p2 = 0.0f;        // 锟斤拷态锟斤拷锟斤拷锟斤拷系锟斤拷
   
    
    // 锟斤拷锟斤拷锟斤拷锟斤拷30时锟斤拷锟矫凤拷锟斤拷锟皆匡拷锟狡ｏ拷锟斤拷锟斤拷锟斤拷偏锟斤拷时锟斤拷锟斤拷应
//    if (abs(error) > 30) {
        // 锟斤拷锟皆癸拷锟缴硷拷锟姐动态锟斤拷锟斤拷锟斤拷系锟斤拷
//        dynamic_p2 = pid->Kp2 * (abs(error) - 30) / 10.0f;
        // 锟斤拷使锟斤拷平锟斤拷锟斤拷锟缴ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷: dynamic_p2 = p2 * pow((abs(chazhi) - 30)/30.0f, 2);
//    }
    
    // 锟角度伙拷锟斤拷锟姐：锟斤拷锟斤拷锟斤拷 + 锟斤拷锟斤拷锟斤拷锟斤拷 + 微锟斤拷锟斤拷
     pid->Output = (error *  pid->Kp
                         
                 + error * abs(error) * pid->Kp2
                   
                 + (pid->Err - pid->Last_Err) * pid->Kd);
    
    
    pid->Last_Err =  pid->Err;             // 锟斤拷锟斤拷锟斤拷史锟斤拷锟�
    
    return pid->Output;
}


float Angular_V(RS_Incremental_PID *pid, int16 angle_output) {
    
  
//    calibratedGyro_Z1= calibratedGyro_Z1 /100;
    float temp_gyro = calibratedGyro_Z1 / 100.0f;
    // 锟斤拷锟斤拷锟斤拷睿猴拷嵌然锟斤拷锟斤拷 - 锟斤拷锟斤拷锟斤拷Z锟斤拷锟斤拷俣锟�
    pid->Err = (angle_output + temp_gyro);
    
    // 锟斤拷锟劫度伙拷锟斤拷锟姐：锟斤拷锟斤拷锟斤拷 + 微锟斤拷锟斤拷
    pid->Output =( pid->Err * pid->Kp + (pid->Err - pid->Last_Err) * pid->Kd);
    pid->Last_Err =  pid->Err;             // 锟斤拷锟斤拷锟斤拷史锟斤拷锟�
     
    return pid->Output;
}


float Angle_PID(RS_Incremental_PID *pid ,float target_angle, float measured_angle )
{
  pid->Err = target_angle- measured_angle;
  
  pid->p_out = pid->Kp * pid->Err;
  pid->d_out = pid->Kd * (pid->Err - pid->Last_Err) ;
  
  pid->Output= pid->p_out + pid->d_out;
  
  pid->Last_Err =  pid->Err;
  
  return pid->Output;
}



float w_PID(RS_Incremental_PID *pid ,float target_w, float measured_w )
{
  pid->Err = target_w- measured_w;
  
  pid->p_out = pid->Kp * pid->Err;
  pid->d_out = pid->Kd * (pid->Err - pid->Last_Err) ;
  
  pid->Output= pid->p_out + pid->d_out;
  
  pid->Last_Err =  pid->Err;
  
  return pid->Output;
}



int Abs(int x)
{
  if(x < 0) x = -x;
  
  return x;
}
