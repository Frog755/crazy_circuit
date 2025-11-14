#include "enconder.h"
#include "ins.h"


#define PI 3.14159265358979f  
#define WHEEL_RADIUS 2.0f

int16 left_encoder = 0;
int16 right_encoder = 0;
int16 last_left_encoder=0;
int16 last_right_encoder=0;
float left_Speed=0;
float right_Speed=0;

void encoder_init(void)
{
    encoder_dir_init (TC_CH09_ENCODER , TC_CH09_ENCODER_CH1_P05_0 , TC_CH09_ENCODER_CH2_P05_1);
    encoder_dir_init (TC_CH07_ENCODER , TC_CH07_ENCODER_CH1_P02_0 , TC_CH07_ENCODER_CH2_P02_1 );

    pit_ms_init(ENCOND , 5);
}




void Get_conder(void)
{
    left_encoder = encoder_get_count (TC_CH09_ENCODER );
    
    left_encoder = left_encoder*0.8+last_left_encoder*0.2;
    last_left_encoder=left_encoder;
    
    encoder_clear_count (TC_CH09_ENCODER );
        
    right_encoder = -encoder_get_count (TC_CH07_ENCODER );
    
    right_encoder = right_encoder*0.8+last_right_encoder*0.2;
    last_right_encoder=right_encoder;
    encoder_clear_count (TC_CH07_ENCODER );
    
}
