#ifndef __MOTOR_H 
#define __MOTOR_H

#include "zf_common_headfile.h"
//#include "image.h"

#define PWM_L ATOM0_CH2_P21_4
#define PWM_R ATOM0_CH0_P21_2


#define PWM_MAX      (6666)

#define DIR_L                (P21_5)
#define DIR_R                (P21_3)

void motor_init(void);
void set_motor_pwm(int16 left_pwm , int16 right_pwm );
void Turn_delay(float err);
float Speed_Strategy(int16 err);
void control(void);


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

