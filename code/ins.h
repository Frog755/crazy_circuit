#ifndef __INS_H 
#define __INS_H

#include "zf_common_headfile.h"
#include "encoder.h"
//#include "image.h"

#define point_max   (12000)
#define BREAK_PAGE   (4)


void ins_track(void);

extern uint8 record_mode;
extern uint8 replay_mode;
extern float target_yaw;
extern uint8 record_stop;
extern int16 record_encoder;
extern int16 replay_encoder;
extern uint32 replay_index;
extern uint32 record_index;
extern float record_yaw[point_max];
extern uint8 stop_replay;
extern volatile uint8 current_replay_page;
extern uint8 flash_page ;


#endif


