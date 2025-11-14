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
 * 功能描述: PID参数初始化函数
 * 参数说明: 无
 * 返回参数: 无
 * 调用说明: 系统初始化时调用一次
 */
void PID_Init(void)
{
    // 左轮速度环PID参数（增
    pid_L.Kp =12.0;// 比例系 
    pid_L.Ki =3.0;   // 积分系数
    pid_L.Kd = 0;    // 微分系数
    pid_L.Target =100;//速度（单位：编码器脉冲/控制周期）

    // 右轮速度环PID参数（增量式）
    pid_R.Kp = 12.0;  // 比例系数 
    pid_R.Ki = 4.0;   // 积分系数 
    pid_R.Kd = 0;   // 微分系数
    pid_R.Target =100; //速度
                                 
    angle_pid.Kp=2.9;       //60:2.0           //70:2.4                      //80:2.4      //85:2.6        //95:3
    angle_pid.Kd=3.3;       //60:1.2           //70:1.9                      //80:2.6         //3.0         3.2
    angle_pid.Kp2=0.0;     //60:-0.09（35）    //-0.12(35)或-0.15（30）     //80:-0.12(30)     //-0.14       -0.14
    
    
    Angular_pid.Kp=0.5;       //60:0.9        //70:0.9                     //80:0.85       //0.8          0.8
    Angular_pid.Kd=0.1;      //60:0.15        //70:0.15                    //80:0.15     //0.15         0.2

    
    //角度环参数
    
    pid_angle.Kp = 1.5;
    pid_angle.Kd = 2.0;
    
    //角速度环
    pid_w.Kp = -0.1;
    pid_w.Kd = 0;
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
  
    
    
    float_t dynamic_p2 = 0.0f;        // 动态非线性系数
   
    
    // 当误差大于30时启用非线性控制，减弱大偏差时的响应
//    if (abs(error) > 30) {
        // 线性过渡计算动态非线性系数
//        dynamic_p2 = pid->Kp2 * (abs(error) - 30) / 10.0f;
        // 或使用平方过渡（更激进）: dynamic_p2 = p2 * pow((abs(chazhi) - 30)/30.0f, 2);
//    }
    
    // 角度环计算：比例项 + 非线性项 + 微分项
     pid->Output = (error *  pid->Kp
                         
                 + error * abs(error) * pid->Kp2
                   
                 + (pid->Err - pid->Last_Err) * pid->Kd);
    
    
    pid->Last_Err =  pid->Err;             // 更新历史误差
    
    return pid->Output;
}


float Angular_V(RS_Incremental_PID *pid, int16 angle_output) {
    
  
    calibratedGyro_Z1= calibratedGyro_Z1 /100;      
    // 计算误差：角度环输出 - 陀螺仪Z轴角速度
    pid->Err = (   angle_output + calibratedGyro_Z1);    
    
    // 角速度环计算：比例项 + 微分项
    pid->Output =( pid->Err * pid->Kp + (pid->Err - pid->Last_Err) * pid->Kd);
    pid->Last_Err =  pid->Err;             // 更新历史误差
     
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
  if(x > 0) x = x;
  else if(x < 0) x = -x;
  
  return x;
}