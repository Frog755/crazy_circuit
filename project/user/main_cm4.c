#include "zf_common_headfile.h"
#include "image.h"
#include "enconder.h"
#include "motor.h"
#include "pid.h"
#include "key.h"
#include "ins.h"
#include "vofa_function.h"
#include "vofa_uart.h"
#include "image.h"
#include "Gyro.h"
#include "stright.h"

int main(void)
{
    clock_init(SYSTEM_CLOCK_160M);      // ʱ�����ü�ϵͳ��ʼ��<��ر���>
    
    debug_init();                       // ���Դ��ڳ�ʼ��
    // �˴���д�û����� ���������ʼ�������
    
    ips200_init(IPS200_TYPE_SPI);
    encoder_init();
    motor_init();
    PID_Init();
    imu660ra_init();
    KEY_INIT();
    calibrateGyro();
    ResetYawZero();
    pit_ms_init(PIT_CH0 , 2);//�����ǲ�������
    pit_ms_init(PIT_CH2 ,10);//����
    flash_init();
    vofaJustFloatInit();
    wireless_uart_init();
    mt9v03x_init();
    gpio_init(P23_7, GPO, 0, GPO_PUSH_PULL); 
    timer_init(TC_TIME2_CH0, TIMER_MS);

//    Read_flash(); 
    // �˴���д�û����� ���������ʼ�������
    for(;;)
    {
      //   �˴���д��Ҫѭ��ִ�еĴ���
        if(Key_Get()==1)
        {
          system_delay_ms(2000);
          run_flag = 1;     
//          replay_mode=1;
        }
   


//       vofa_uart_proc();//���ߴ��ڵ���

        if(mt9v03x_finish_flag==1)
        {
            Image_Binarization();//��ֵ��
            find_line(); //ɨ��
            element_cow_col();
            circular();
            stright_angle();//ֱ��
            cross();
//            break_rode();//��·
            get_error();
//            drawkline();
            ips200_displayimage03x((const uint8 *)image_two_value,120,100);
            mt9v03x_finish_flag = 0;
        }
        
    }
}
