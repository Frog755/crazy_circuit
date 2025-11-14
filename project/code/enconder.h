#ifndef __ENCONDER_H 
#define __ENCONDER_H

#include "zf_common_headfile.h"

#define ENCOND  (PIT_CH1)
extern float left_Speed;
extern float right_Speed;
void encoder_init(void);
void Get_conder(void);

extern int16 left_encoder ;
extern int16 right_encoder ;

#endif

