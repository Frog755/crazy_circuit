#ifndef __KEY_H_
#define __KEY_H_

#include "zf_common_headfile.h"

enum
{
  KEY1 = 1,
  KEY2,
  KEY3,
  KEY4,
};

void KEY_INIT(void);
uint8_t Key_Get(void);
void PID(void);
void Navigate(void);
void deaprt();
void menu();
void Write_Flash(void);
void Read_flash(void);


extern uint8 keyOld;
extern uint16 exposure_time;


extern uint8  KEY_VALUE;

#endif

