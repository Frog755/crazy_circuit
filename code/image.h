#ifndef __IMAGE_H_
#define __IMAGE_H_

#include "zf_common_typedef.h"
#include "math.h"
#include "zf_device_mt9v03x.h"

#define white_point  255
#define black_point  0
#define MAX_INTERPOLATE_GAP 40
#define JUMP_STEP_MAX_xia  40
#define JUMP_STEP_max_shang 20
#define JUMP_STEP 5
#define JUMP_STEP_shang 2

extern uint8 otsuThreshold(uint8 *image, uint16 width, uint16 height);
void get_bin_image(void);
void serach_line(void);

//uint8_t otsuThreshold(uint8_t *image, uint16_t width, uint16_t height);
void Image_Binarization(void);
void find_line(void);
void get_error(void);
void drawkline(void);
float Straight_Judge(uint8 dir, uint8 start, uint8 end);
void element_cow_col(void);
void stright_angle(void);
void T_corner_h(void);
void T_corner(void);
void cross(void);
void Add_Line(uint8 x1,uint8 y1,uint8 x2,uint8 y2);
void Add_left_Line(uint8 x1,uint8 y1,uint8 x2,uint8 y2);
void Add_right_Line(uint8 x1,uint8 y1,uint8 x2,uint8 y2);


extern uint8 image_two_value[MT9V03X_H][MT9V03X_W];
extern uint8 threshold;
extern uint8 limite_th;
extern float image_error;
extern int16 in_cir_left;
extern int16 in_cir_right;
extern int16 left_stright;
extern int16 right_stright;
extern int16 Benzene_turn_flag_left;
extern int16 Benzene_turn_flag_right;
extern int16 Benzene_turn_flag_up;
extern int16 Benzene_turn_flag_down;
extern int16 left_cir_flag;
extern int16 right_cir_flag;
extern int16 out_cir_left;
extern int16 out_cir_right;
extern int16 left_cir_height;
extern int16 last_left_cir_height;
extern int16 right_cir_height;
extern int16 last_right_cir_height;
extern int16 out_cir_encoder;
extern int16 out_cir_right;
extern int16 rode_wide_list[MT9V03X_H];
extern int16 cross_flag;
extern int16 break_flag;
extern float stright_anglez;
extern int16 left_T_h_flag;
extern int16 right_T_h_flag;
extern int16 T_turn_flag;
extern int16_t T_flag;
extern int16 T_Debounce_Count;
extern uint8 circuit_count;


//extern int16 in_left_encoder;

#endif

