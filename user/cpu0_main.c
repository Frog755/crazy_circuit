/*********************************************************************************************************************
 * TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 TC264 开源库的一部分
 *
 * TC264 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          cpu0_main
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          ADS v1.10.2
 * 适用平台          TC264D
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-09-15       pudding            first version
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_disarm"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

#include "image.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "key.h"
#include "ins.h"
#include "image.h"
#include "Gyro.h"
#include "vofa_function.h"
#include "vofa_uart.h"

int core0_main (void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等
    ips200_init(IPS200_TYPE_SPI);
    encoder_init();
    motor_init();
    PID_Init();
    imu660ra_init();
    KEY_INIT();
    calibrateGyro();
    ResetYawZero();
    pit_ms_init(CCU60_CH1, 2);
    //flash_init();
    vofaJustFloatInit();
    //wireless_uart_init();
    mt9v03x_init();
    //Read_flash();

    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    while (TRUE)
    {
//        set_motor_pwm(1000,1000);
        //此处编写需要循环执行的代码
        if (Key_Get() == 1)
        {
            system_delay_ms(2000);

            run_flag = 1;
            //  replay_mode=1;
        }

         vofa_uart_proc();

        if (mt9v03x_finish_flag == 1)
        {
            Image_Binarization();
            find_line();
            element_cow_col();
            T_corner_h();
            T_corner();
            stright_angle();
            cross();

            get_error();
            drawkline();
            ips200_displayimage03x((const uint8* )image_two_value, 120, 100);

            mt9v03x_finish_flag = 0;
        }
//            ips200_show_int(20, 200, pwm_l, 3);
//            ips200_show_int(40, 200, pwm_r, 3);
            ips200_show_int(20, 200, current_yaw, 3);
            ips200_show_int(20, 220, image_error, 2);
            ips200_show_int(20, 240, T_flag, 1);
            ips200_show_int(20, 260, left_T_h_flag, 1);
            ips200_show_int(20, 280, right_T_h_flag, 1);
            ips200_show_int(20, 300, cross_flag, 1);

        // 此处编写需要循环执行的代码
    }
}

#pragma section all restore
// **************************** 代码区域 ****************************
