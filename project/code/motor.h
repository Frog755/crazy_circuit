#ifndef __MOTOR_H 
#define __MOTOR_H

#include "zf_common_headfile.h"
//#include "image.h"

#define PWM_L TCPWM_CH50_P18_7
#define PWM_R TCPWM_CH51_P18_6 


#define PWM_MAX      (6666)

#define DIR_L                (P00_3)
#define DIR_R                (P00_2)

void motor_init(void);
void set_motor_pwm(int16 left_pwm , int16 right_pwm );
void Turn_delay(float err);
float Speed_Strategy(int16 err);
void control(void);
void speed_strategy(void);

extern float left_target_speed;
extern float right_target_speed;
extern uint8 run_flag;
extern int is_in_turn;
extern float speed_diff;
extern float  diff_angle;
extern int turn_delay;
extern int ramp_flag;
extern int current_base_speed;

#endif

