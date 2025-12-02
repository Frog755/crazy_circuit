#include <encoder.h>

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
    encoder_dir_init (TIM4_ENCODER , TIM4_ENCODER_CH1_P02_8 , TIM4_ENCODER_CH2_P00_9);
    encoder_dir_init (TIM6_ENCODER , TIM6_ENCODER_CH1_P20_3 , TIM6_ENCODER_CH2_P20_0 );

    pit_ms_init(ENCOND , 5);
}




void Get_conder(void)
{
    left_encoder = -encoder_get_count (TIM4_ENCODER );
    
    left_encoder = left_encoder*0.8+last_left_encoder*0.2;
    last_left_encoder=left_encoder;
    
    encoder_clear_count (TIM4_ENCODER );
        
    right_encoder = +encoder_get_count (TIM6_ENCODER );
    
    right_encoder = right_encoder*0.8+last_right_encoder*0.2;
    last_right_encoder=right_encoder;
    encoder_clear_count (TIM6_ENCODER );
    
}
