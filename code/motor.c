#include <encoder.h>
#include "motor.h"
#include "pid.h"
#include <math.h>
#include <stdint.h>
#include "stdlib.h"
#include "ins.h"
#include "image.h"
#include "Gyro.h"
#include "zf_driver_delay.h"
#include "zf_driver_uart.h"



uint8 run_flag=0;

uint16 angle_1_time=0;
uint16 Speed_time=0;
float left_target_speed;
float right_target_speed;
float speed_diff = 0;
float  diff_angle=0;
int current_base_speed=0;
int base_speed_target =0;
int speed_ramp_step=1;
int ramp_flag=0;



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
        
        pwm_set_duty(PWM_L, left_pwm);//设置电机PWM占空比
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
        
        pwm_set_duty(PWM_R, right_pwm);//设置电机PWM占空比
      }
      else 
      {
        gpio_set_level (DIR_R , 0);
        
        pwm_set_duty(PWM_R, -right_pwm);
      }

}

float Speed_Strategy(int16 err){

    //pid_L.Target=80;
    //pid_R.Target=80;
    //float smooth_factor=0.2f;   //平滑系数
    //float temp_target_speed=0;
     // 根据图像误差调整速度，误差越大速度越低，
    //float error_adjustment = abs(err) ;


        // 根据误差调整基础速度，误差越大速度越低，
   //if (error_adjustment < 8) {
   //    error_adjustment =  SPEED_MEDIUM;
   //}
   //else if(error_adjustment >= 8 && error_adjustment <13 ){
   //     error_adjustment =  0 ;
   //}else if(error_adjustment>=13 ){
        //if(left_stright==1||right_stright==1){
      //if(is_in_turn==1)
   //     error_adjustment =  SPEED_LOW ;
   //}

   // error_adjustment -= abs(err);// * smooth_factor;

     // 计算此时目标速度
   //temp_target_speed =  pid_L.Target + error_adjustment;
    //return  temp_target_speed;
    float base_speed = 35;
    float abs_err = abs(err);
    float adjustment = 0;

    if(abs_err < 10)
    {
        adjustment = 6;
    }// 直道飞起
    else if(abs_err < 20)
    {
        adjustment = 4.5;
    }
    else if(abs_err < 30)
    {
        adjustment = 3;
    }
    else if(abs_err < 40)
    {
        adjustment = 1.5;
    }
    else
    {
        adjustment = 0;
    }

    base_speed_target = base_speed + adjustment;

    return base_speed_target;

//***********************************************************

}

void control(void)
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
       //base_speed_target = pid_L.Target;
//       if(left_stright==1 || right_stright==1) {
//        base_speed_target = 60;  // 直角转弯降速
//      } else {
        base_speed_target = Speed_Strategy(image_error);
//      }
       if(current_base_speed < base_speed_target)
       {
          current_base_speed+= speed_ramp_step;   //斜坡上升，防止猛抬头
       }
       else if(current_base_speed> base_speed_target)
       {
            // 修复：应该逐渐减小，而不是直接跳变到speed_ramp_step
            current_base_speed -= speed_ramp_step;  // 斜坡下降，平滑降低速度
            if(current_base_speed < 0) current_base_speed = 0;  // 防止变为负数
       }

      speed_diff=Angular_V(&Angular_pid,diff_angle);
      left_target_speed= current_base_speed -  speed_diff;
      right_target_speed = current_base_speed + speed_diff;

        if (left_target_speed < 0)
            left_target_speed = 0;
        if (right_target_speed < 0)
            right_target_speed = 0;

     }


    pwm_l = IncrementalPID_L(&pid_L , left_target_speed ,left_encoder);
    pwm_r = IncrementalPID_R(&pid_R , right_target_speed ,right_encoder);

    // 娣诲姞PID杈撳嚭姝诲尯锛岄槻姝㈠皬鍊兼姈鍔�
    // 褰撹緭鍑哄緢灏忔椂锛岀洿鎺ヨ涓�0锛岄伩鍏嶇數鏈哄湪灏廝WM鍊间笅鎶栧姩
    if(pwm_l > -50 && pwm_l < 50) pwm_l = 0;
    if(pwm_r > -50 && pwm_r < 50) pwm_r = 0;

    set_motor_pwm(pwm_l, pwm_r);
    


}

