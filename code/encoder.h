#ifndef __ENCODER_H
#define __ENCODER_H

#include "zf_common_headfile.h"

#define ENCOND  (CCU60_CH0)
extern float left_Speed;
extern float right_Speed;
void encoder_init(void);
void Get_conder(void);

extern int16 left_encoder ;
extern int16 right_encoder ;

#endif

