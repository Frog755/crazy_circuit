#include "motor.h"
#include "pid.h"
#include "enconder.h"
#include <math.h>
#include <stdint.h>
#include "stdlib.h"
#include "ins.h"
#include "image.h"
#include "Gyro.h"
#include "zf_driver_delay.h"


#define SPEED_LOW    -5 // 低速模式增加速度
#define SPEED_MEDIUM 15   // 中速模式增加速度
#define SPEED_HIGH    21   // 高速模式增加速度

uint8 run_flag=0;


uint16 angle_1_time=0;
uint16 Speed_time=0;
float left_target_speed;
float right_target_speed;
float speed_diff = 0;
float  diff_angle=0;
int turn_delay=0;
int current_base_speed=0;
int base_speed_target =0;
int speed_ramp_step=1;
int ramp_flag=0;

float last_stright_anglez=0;
int strategy_num=0;
int expedite_mode=0;

void motor_init(void)
{
    pwm_init(PWM_L, 17000, 0);//PWM通道L1初始化频率17khz，
    pwm_init(PWM_R, 17000, 0);//PWM通道R1初始化频率17khz，
    
    gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL);
}
void set_motor_pwm(int16 left_pwm , int16 right_pwm )
{
      
      if(left_pwm > PWM_MAX) left_pwm = PWM_MAX;
      else if(left_pwm < -PWM_MAX) left_pwm = -PWM_MAX;
      
      if(left_pwm >= 0)
      {
        gpio_set_level (DIR_L , 1);
        
        pwm_set_duty(PWM_L, left_pwm);//设置左轮电机占空比       
      }
      else 
      {
        gpio_set_level (DIR_L , 0);
        pwm_set_duty(PWM_L, -left_pwm);     
      }  
        
      if(right_pwm > PWM_MAX) right_pwm = PWM_MAX;
      else if(right_pwm < -PWM_MAX) right_pwm = -PWM_MAX;
      
      if(right_pwm >= 0)
      {
        gpio_set_level (DIR_R , 1);
        
        pwm_set_duty(PWM_R, right_pwm);//设置右轮电机占空比
      }
      else 
      {
        gpio_set_level (DIR_R , 0);
        
        pwm_set_duty(PWM_R, -right_pwm);
      }

}
  //int16 turn_sign=0;
// 全局变量定义
int is_in_turn = 0;       // 转弯标志位
uint32_t turn_start_time = 0;  // 进入转弯的时间记录
uint32_t turn_OK_time = 0;  // 进入转弯的时间记录
static const uint32_t TURN_DELAY_MS = 300; // 转弯结束后的延时时间(ms)
static const float TURN_THRESHOLD = 30.0f; // 转弯阈值
int last_turn_state = 0;

int current_turn_state=0;

void Turn_delay(float err) {
  
  
    
  if(fabsf(err) > TURN_THRESHOLD){
     current_turn_state = 1;
  }else{ current_turn_state = 0;}
  
  
    // 检测转弯开始
    if(current_turn_state==1) {
      
       is_in_turn = 1;
//       turn_start_time = timer_get(TC_TIME2_CH0); // 记录进入时间
        
        
  }
    
    // 检测转弯结束
    if(current_turn_state==0&&fabsf(err) < TURN_THRESHOLD ) {
        // 记录退出时间，但不立即加速
        turn_OK_time = timer_get(TC_TIME2_CH0);
        
        
        if(timer_get(TC_TIME2_CH0) - turn_start_time > TURN_DELAY_MS) {
          
              is_in_turn = 0;
        
        
   }
    
  
    //last_turn_state = current_turn_state; // 更新状态
  }



    //last_turn_state = current_turn_state; // 更新状态

}



float Speed_Strategy(int16 err){

    pid_L.Target=80;
    pid_R.Target=80;
    float smooth_factor=0.2f;   //平滑因子
    float temp_target_speed=0;
     // 根据图像误差调整速度（误差越大，速度越低）
    float error_adjustment = abs(err) ;
     
    
        // 限制最大调整量（不超过最高速）
   //if (error_adjustment < 8) {
      
       //error_adjustment =  SPEED_HIGH   ;
       
   //}
  // else if(error_adjustment >= 8 && error_adjustment <13 ){
    
        //error_adjustment =  SPEED_MEDIUM ;
    
  //  }else 
      if(error_adjustment>=13 ){
        //if(left_stright==1||right_stright==1){
      //if(is_in_turn==1)
        error_adjustment =  SPEED_LOW ;
        
    
   }
    
    error_adjustment -= abs(err) * smooth_factor;
    
     // 计算临时目标速度
    temp_target_speed =  pid_L.Target + error_adjustment;
    
    
    return  temp_target_speed;
//***********************************************************

}

void contral(void)
{
   
     angle_1_time++;
     if(angle_1_time==2)
     {  
        diff_angle= angle(&angle_pid , image_error);
        angle_1_time=0;
     }
     if(replay_mode==1)
     {  
      speed_diff= -Angle_PID( &pid_angle ,target_yaw , current_yaw);
      left_target_speed= pid_L.Target -  speed_diff;
      right_target_speed = pid_R.Target + speed_diff;
//       left_target_speed= 0;
//       right_target_speed= 0;
     }
     else //正常循迹
     {
       base_speed_target = pid_L.Target;
       if(current_base_speed < base_speed_target&& ramp_flag==0)
       {
          current_base_speed+= speed_ramp_step;   //慢速起步，防止起步翘头
          if(current_base_speed> base_speed_target)
          {
            current_base_speed= base_speed_target;
            ramp_flag=1;
          }
       }
//        if(in_cir_left==1 && Benzene_turn_flag_up==2 && Benzene_turn_flag_down==2)
//        {
//          current_base_speed = 100;
//        }
//      speed_strategy();
//      if(expedite_mode==1)
//      {
//        pid_L.Target=145;
//      }
//      else if(expedite_mode==2)
//      {
//        pid_L.Target=135;
//      }
//      else if(expedite_mode==0)
//      {
//        pid_L.Target=130;
//      } 
      speed_diff=Angular_V(&Angular_pid,diff_angle);      
      left_target_speed= current_base_speed -  speed_diff;
      right_target_speed = current_base_speed + speed_diff;
//      
     }

      
    pwm_l = IncrementalPID_L(&pid_L , left_target_speed ,left_encoder);
    pwm_r = IncrementalPID_R(&pid_R , right_target_speed ,right_encoder);
    
    set_motor_pwm(pwm_l, pwm_r);
}



void speed_strategy(void)
{
  if(fabs(image_error)<5 && Straight_Judge(3,1,90)<2) //判断是否直道
  {
    if(fabs(stright_anglez- last_stright_anglez)<5) //检查车身姿态
    {
      strategy_num++;
    }
  }
  else
  {
    strategy_num=0;
  }
  last_stright_anglez= stright_anglez;

  
  if(strategy_num==10)
  {
    expedite_mode=1;
  }
  else if(strategy_num>=5 && strategy_num<10)
  {
    expedite_mode=2;
  }
  else
  {
    expedite_mode=0;
  }
  
}








