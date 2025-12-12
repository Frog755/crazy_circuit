#include "image.h"
#include "zf_device_ips200.h"
#include "zf_common_font.h"
#include "Gyro.h"
#include "encoder.h"
#include "ins.h"

//锟斤拷值锟斤拷锟斤拷乇锟斤拷锟�?
uint8 threshold;
uint8 limite_th = 120;
uint8 image_two_value[MT9V03X_H][MT9V03X_W];

//扫锟斤拷锟斤拷乇锟斤拷锟�?
int16 left_line_list[MT9V03X_H]; //左线数组
int16 right_line_list[MT9V03X_H]; //右线数组
int16 mid_line_list[MT9V03X_H];  //中线数组
int16 rode_wide_list[MT9V03X_H]; //车道宽度数组
int16 left_lost_flag[MT9V03X_H]; //左线丢失标志位
int16 right_lost_flag[MT9V03X_H]; //右线丢失标志位
int16 left_end_flag = 0; //左线结束标志位
int16 right_end_flag = 0; //右线结束标志位
uint8 left_end = 0; //左线结束点
uint8 right_end = 0; //右线结束点
uint8 end_line = 0; //图像结束点

int16 cir_out_encoder = 0;

//锟斤拷锟斤拷权锟斤拷锟斤拷锟斤拷
float mid_weight_list[MT9V03X_H] = {12, 12, 11.5, 11.5, 11.5, 11.5, 11, 11, 11, 11,                       //0-9琛�
        10.5, 10.5, 10.5, 10.5, 10.5, 10.5, 10, 10, 10, 10,   //10-19琛�
        9.5, 9.5, 9.5, 9.5, 9.5, 9, 9, 9, 9, 9,             //20-29琛�
        8.5, 8.5, 8.5, 8.5, 8.5, 8, 8, 8, 8, 8,             //30-39琛�
        7.5, 7.5, 7.5, 7.5, 7.5, 7, 7, 7, 7, 7,             //40-49琛�
        6.5, 6.5, 6.5, 6.5, 6.5, 6, 6, 6, 6, 6,             //50-59琛�
        5.5, 5.5, 5.5, 5.5, 5.5, 5, 5, 5, 5, 5,             //60-69琛�
        4.5, 4.5, 4.5, 4.5, 4.5, 4, 4, 4, 4, 4,             //70-79琛�
        3.5, 3.5, 3.5, 3.5, 3, 3, 3, 3, 2.5, 2.5,           //80-89琛�
        2, 2, 2, 1.5, 1.5, 1.5, 1, 1, 0.5, 0.5              //90-99琛�
        };

float mid_value = 0;
float image_error = 0;
int16 mid = 47;

//元素行列标志位
int16 Benzene_turn_flag_up = 0;
int16 Benzene_turn_point_up1 = 0;
int16 Benzene_turn_point_up2 = 0;
int16 Benzene_turn_flag_down = 0;
int16 Benzene_turn_point_down1 = 0;
int16 Benzene_turn_point_down2 = 0;
int16 Benzene_turn_flag_left = 0;
int16 Benzene_turn_point_left1 = 0;
int16 Benzene_turn_point_left2 = 0;
int16 Benzene_turn_flag_right = 0;
int16 Benzene_turn_point_right1 = 0;
int16 Benzene_turn_point_right2 = 0;

//直角标志位
int16 left_stright = 0;
int16 right_stright = 0;
int16 left_strflag = 0;
int16 right_strflag = 0;
float stright_anglez = 0;

//圆环标志位
int16 left_cir_flag = 0;
int16 right_cir_flag = 0;
int16 left_cir_point = 0;
int16 right_cir_point = 0;
int16 left_cir_height = 0;
int16 last_left_cir_height = 0;
int16 right_cir_height = 0;
int16 last_right_cir_height = 0;
int16 Benzene_height_cont = 0;
int16 in_cir_left = 0;
int16 in_cir_right = 0;
int16 out_cir_left = 0;
int16 out_cir_right = 0;
int16 out_cir_encoder = 0;

//十字标志位
int16 cross_flag = 0;

//断路标志位
int16 check_row = 0;
int16 start_check_row = MT9V03X_H - 1;
int16 end_check_row = 10;
float percent_lost = 0.f;
int16 break_cont = 0;
int16 break_flag = 0;


//T字路口标志位
int16_t T_flag = 0;
uint8_t T_State = 0; //状态（最终判定）

//T字路口转向记录
int16 T_turn_flag = 0;
int16_t T_turn_sequence[total_T_count] = {0, 1};// 根据实际赛道顺序填写  0:左转, 1:右转
uint8_t T_count = 0; // 记录当前通过了第几个T字

//横T路口标志位
int16_t left_T_h_flag = 0;
int16_t right_T_h_flag = 0;
int16_t T_turn_h_sequence[total_h_count] = {1, 1, 1, 1};// 根据实际赛道顺序填写  0:不转向直走, 1:转向  //待完善暂时
uint8_t h_count = 0; // 记录当前通过了第几个横T字

uint8_t is_in_h_junction = 0; // 当前是否正在路口检测范围内
uint8_t T_h_feature_cnt = 0;  // 特征防抖计数


//回路个数
uint8 circuit_count = 0;

//

/*************************************大津法********************************************/
uint8_t otsuThreshold (uint8_t *image, uint16_t width, uint16_t height)
{
#define GrayScale 256

    int pixelCount[GrayScale] = {0};       // 统计每个灰度值的像素数量
    float pixelPro[GrayScale] = {0.0f};    // 每个灰度值的概率
    int Sumpix = width * height;           // 图像总像素数
    uint8_t Threshold = 0;                 // 最终确定的最佳阈值
    uint8_t *data = image;                 // 图像数据指针

    float u = 0.0f;                        // 图像全局平均灰度
    float maxVariance = 0.0f;              // 最大类间方差
    float w0 = 0.0f;                       // 前景(或背景)像素的权重/概率
    float avgValue = 0.0f;                 // 前景(或背景)的加权平均灰度值

    // Step 1: 统计每个灰度值的像素数量 (即计算直方图)
    for (int i = 0; i < Sumpix; ++i)
    {
        pixelCount[data[i]]++;
    }

    // Step 2: 计算每个灰度值的概率，并计算图像的全局平均灰度 u
    for (int i = 0; i < GrayScale; ++i)
    {
        pixelPro[i] = (float) pixelCount[i] / Sumpix;
        u += i * pixelPro[i]; // 全局平均灰度 u = sum(i * P(i))
    }

    // Step 3: 遍历所有可能的灰度值作为阈值，计算类间方差，找出最佳阈值
    for (int i = 0; i < GrayScale; ++i)
    {
        w0 += pixelPro[i];         // 累积前景像素的概率 (权重)
        avgValue += i * pixelPro[i]; // 累积前景的加权平均灰度

        if (w0 == 0 || w0 == 1)
            continue; // 边界情况，跳过

        // 计算类间方差 (使用简化公式：sigma_B^2 = (u*w0 - avgValue)^2 / (w0 * (1-w0)))
        // 注：avgValue 是前景的加权平均 u0*w0，公式可简化为 (u0*w0 - u*w0)^2 / (w0 * (1-w0))
        float variance = powf((avgValue - u * w0), 2) / (w0 * (1 - w0));

        if (variance > maxVariance)
        {
            maxVariance = variance;
            Threshold = (uint8_t) i;
        }
    }
    return Threshold;
}

/******************************二值化**********************************/
void Image_Binarization (void)
{
    uint8 i, j;

    threshold = otsuThreshold(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
    if (threshold < limite_th)
        threshold = limite_th;
    for (i = 0; i < MT9V03X_H - 1; i++)
    {
        for (j = 0; j < MT9V03X_W - 1; j++)
        {
            if (j < 10 || j > 84)
            {
                image_two_value[i][j] = mt9v03x_image[i][j] > (threshold + 10) ? 255 : 0; //鍥惧儚涓よ竟闃堝�艰緝灏�
            }
            else
            {
                image_two_value[i][j] = mt9v03x_image[i][j] > threshold ? 255 : 0;
            }
        }
    }
}

/******************************扫描线*******************************************/
void find_line (void)
{
    uint8 current_y = MT9V03X_H - 2;
    left_end_flag = 0;
    right_end_flag = 0;
    //锟斤拷始锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
    memset(left_line_list, -1, sizeof(left_line_list));
    memset(right_line_list, -1, sizeof(right_line_list));
    memset(mid_line_list, 0, sizeof(mid_line_list));

    while (current_y > 5 && left_end_flag == 0)
    {
        uint8 find_left_end = MT9V03X_W - 2;
//      if(right_cir_flag==1 || out_cir_right==1)
//      {
//        find_left_end= mid+10;//10
//      }
//      if(in_cir_left==1 && Benzene_turn_flag_down==2)
//      {
//        find_left_end= mid+5;//10
//      }
        for (uint8 i = 2; i < find_left_end; i++)
        {
            if (image_two_value[current_y][i] == black_point && image_two_value[current_y][i + 1] == white_point
                    && image_two_value[current_y][i + 2] == white_point)
            {
                left_line_list[current_y] = i;
                left_end_flag = 1;
                left_end = current_y;
                break;
            }
        }
        if (current_y < 20)
            break;
        if (left_end_flag == 0)
            current_y -= 5;
    }
    //补低线
    if (left_end < MT9V03X_H - 10)
    {
        for (uint8 j = MT9V03X_H - 1; j > left_end + 1; j--)
        {
            left_line_list[j] = left_line_list[left_end];
        }
    }

    while (current_y > 5 && right_end_flag == 0)
    {
        uint8 find_right_end = 2;
//      if(left_cir_flag==1 || out_cir_left==1 || in_cir_right==1)
//      {
//        find_right_end= mid-5;
//      }
        for (uint8 i = MT9V03X_W - 2; i > find_right_end; i--)
        {
            if (image_two_value[current_y][i] == black_point && image_two_value[current_y][i - 1] == white_point
                    && image_two_value[current_y][i - 2] == white_point)
            {
                right_line_list[current_y] = i;
                right_end_flag = 1;
                right_end = current_y;
                break;
            }
        }
        if (current_y < 20)
            break;
        if (right_end_flag == 0)
            current_y -= 5;
    }

    //补低线
    if (right_end < MT9V03X_H - 10)
    {
        for (uint8 j = MT9V03X_H - 1; j > right_end + 1; j--)
        {
            right_line_list[j] = right_line_list[right_end];
        }
    }

    //确定结束点
    end_line = (left_end > right_end) ? right_end : left_end;

    //从结束点开始扫描
    if (right_end_flag == 1) //锟斤拷锟斤拷冶呓锟斤拷锟窖碉拷锟斤拷 锟斤拷始锟斤拷锟斤拷锟揭憋拷锟斤拷
    {
        for (uint8 i = right_end - 1; i > 0; i--)
        {
            int start_col;
            int end_col;
            //锟斤拷锟斤拷锟竭诧拷锟斤拷
            start_col = right_line_list[i + 1] + 15;
            end_col = right_line_list[i + 1] - 15;

            //防止越界
            if (end_col < 2)
                end_col = 2;
            if (start_col > MT9V03X_W - 2)
                start_col = MT9V03X_W - 2;

            for (uint8 j = start_col; j > end_col; j--)
            {
                if (image_two_value[i][j] == black_point && image_two_value[i][j - 1] == white_point
                        && image_two_value[i][j - 2] == white_point) //锟揭碉拷锟斤拷一锟斤拷锟斤拷色
                {
                    right_line_list[i] = j;
                    break;
                }
            }
            if (right_line_list[i] == -1 && right_line_list[i + 1] != -1 && i > 20) //锟斤拷锟竭诧拷锟斤拷
            {

                int max_jump = (i >= MT9V03X_H / 2) ? JUMP_STEP_MAX_xia : JUMP_STEP_max_shang; //锟斤拷锟叫讹拷锟斤拷图锟斤拷锟斤拷母锟斤拷锟斤拷?
                int step = (i >= MT9V03X_H / 2) ? JUMP_STEP : JUMP_STEP_shang;
                uint8 found = 0;
                int jump_end = i - max_jump;
                if (jump_end < 0)
                    jump_end = 0;

                for (uint8 jump = i; jump > jump_end; jump -= step)
                {
                    int start_target = right_line_list[i + 1] + 10;
                    int end_target = right_line_list[i + 1] - 10;

                    if (jump > MT9V03X_H / 2 && jump < MT9V03X_H * 3 / 4)
                    {
                        start_target = right_line_list[i + 1] + 30;
                        end_target = right_line_list[i + 1] - 30;
                    }
                    if (jump > MT9V03X_H * 3 / 4)
                    {
                        start_target = MT9V03X_W - 2;
                        end_target = 2;
                    }

                    if (left_cir_flag == 1 || out_cir_left == 1)
                    {
                        start_target = right_line_list[i + 1] + 5;
                        end_target = right_line_list[i + 1] - 5;
                    }
                    if (in_cir_right == 1)
                    {
                        start_target = right_line_list[i + 1] + 15;
                        end_target = right_line_list[i + 1] - 15;
                    }

                    //锟睫凤拷
                    if (start_target > MT9V03X_W - 2)
                        start_target = MT9V03X_W - 2;
                    if (end_target < 2)
                        end_target = 2;

                    int junp_found = -1;
                    for (uint8 j = start_target; j > end_target; j--)
                    {

                        if (image_two_value[jump][j] == black_point && image_two_value[jump][j - 1] == white_point
                                && image_two_value[jump][j - 2] == white_point) //锟揭碉拷锟斤拷一锟斤拷锟斤拷色
                        {
                            junp_found = j;
                            break;
                        }
                    }
                    if (junp_found != -1)
                    {
                        ;
                        if (i + 5 < MT9V03X_H - 1)
                            Add_right_Line(right_line_list[i + 5], i + 1, junp_found, jump);
                        else
                            Add_right_Line(right_line_list[i + 1], i + 1, junp_found, jump);

                        i = jump;
                        found = 1;
                        break;
                    }
                    if (!found && i > 40)
                    {
                        right_line_list[i] = right_line_list[i + 1];
                    }
                }
            }
            if (right_line_list[i] == -1)
                break; //全扫描完了都没扫到，退出
        }

    }

    //从结束点开始扫描
    if (left_end_flag == 1)
    {
        for (uint8 i = left_end - 1; i > 0; i--)
        {
            //锟斤拷锟斤拷锟竭诧拷锟斤拷
            int start_col;
            int end_col;

            start_col = left_line_list[i + 1] - 15;
            end_col = left_line_list[i + 1] + 15;

            //防止越界
            if (end_col > MT9V03X_W - 2)
                end_col = MT9V03X_W - 2;
            if (start_col < 2)
                start_col = 2;

            for (uint8 j = start_col; j < end_col; j++)
            {
                if (image_two_value[i][j] == black_point && image_two_value[i][j + 1] == white_point
                        && image_two_value[i][j + 2] == white_point) //锟揭碉拷锟斤拷一锟斤拷锟斤拷色
                {
                    left_line_list[i] = j;
                    break;
                }
            }
            if (left_line_list[i] == -1 && left_line_list[i + 1] != -1 && i > 20) //锟斤拷锟竭诧拷锟斤拷
            {

                int max_jump = (i >= MT9V03X_H / 2) ? JUMP_STEP_MAX_xia : JUMP_STEP_max_shang; //锟斤拷锟叫讹拷锟斤拷图锟斤拷锟斤拷母锟斤拷锟斤拷?
                int step = (i >= MT9V03X_H / 2) ? JUMP_STEP : JUMP_STEP_shang;
                uint8 found = 0;
                int jump_end = i - max_jump;
                if (jump_end < 0)
                    jump_end = 0;

                for (uint8 jump = i; jump > jump_end; jump -= step)
                {
                    int start_target = left_line_list[i + 1] - 10;
                    int end_target = left_line_list[i + 1] + 10;

                    if (jump > MT9V03X_H / 2 && jump < MT9V03X_H * 3 / 4)
                    {
                        start_target = left_line_list[i + 1] - 30;
                        end_target = left_line_list[i + 1] + 30;
                    }
                    if (jump > MT9V03X_H * 3 / 4)
                    {
                        start_target = 2;
                        end_target = MT9V03X_W - 2;
                    }

                    //锟睫凤拷
                    if (start_target < 2)
                        start_target = 2;
                    if (end_target > MT9V03X_W - 2)
                        end_target = MT9V03X_W - 2;

                    int junp_found = -1;
                    for (uint8 j = start_target; j < end_target; j++)
                    {
                        if (image_two_value[jump][j] == black_point && image_two_value[jump][j + 1] == white_point
                                && image_two_value[jump][j + 2] == white_point) //锟揭碉拷锟斤拷一锟斤拷锟斤拷色
                        {
                            junp_found = j;
                            break;
                        }
                    }
                    if (junp_found != -1)
                    {
                        if (i + 5 < MT9V03X_H - 1)
                            Add_left_Line(left_line_list[i + 5], i + 1, junp_found, jump);
                        else
                            Add_left_Line(left_line_list[i + 1], i + 1, junp_found, jump);

                        i = jump; // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
                        found = 1;
                        break; // 锟剿筹拷锟斤拷锟竭达拷锟斤拷
                    }
                    if (!found && i > 40)
                    {
                        left_line_list[i] = left_line_list[i + 1];
                    }
                }
            }
            //锟斤拷锟竭诧拷锟斤拷

            if (left_line_list[i] == -1)
                break; //全锟斤拷锟斤拷锟疥还锟斤拷没锟窖碉拷 锟斤拷锟斤拷
        }
    }
    for (int i = MT9V03X_H - 1; i > 0; i--)
    {
        rode_wide_list[i] = 0;
        if (right_line_list[i] != -1 && left_line_list[i] != -1 && right_line_list[i] > left_line_list[i]
                && right_line_list[i] < MT9V03X_W - 2 && left_line_list[i] > 2)
        {
            end_line = i;
            rode_wide_list[i] = right_line_list[i] - left_line_list[i];
            mid_line_list[i] = (right_line_list[i] + left_line_list[i]) / 2;
        }
        else
            mid_line_list[i] = 0;
    }

}

/*****************************获取误差************************************/
void get_error (void)
{
    int16 i = 0;
    mid_value = 0;
    float weight_middle_sum = 0; //锟斤拷权锟斤拷锟斤拷锟桔硷拷值
    float weight_sum = 0; //权锟斤拷锟桔硷拷值

    for (i = 99; i > 0; i--)
    {
        if (mid_line_list[i] != 0)
        {
            weight_middle_sum += mid_line_list[i] * mid_weight_list[i];
            weight_sum += mid_weight_list[i];
        }
    }

    mid_value = weight_middle_sum / weight_sum;
    image_error = (MT9V03X_W / 2) - mid_value;
}

/****************************************************画线*************************************************************/
void drawkline (void)
{
    for (uint8 i = MT9V03X_H - 1; i > 1; i--)
    {
        if (left_line_list[i] > MT9V03X_W - 1)
            left_line_list[i] = MT9V03X_W - 1;
        else if (left_line_list[i] < 0)
            left_line_list[i] = 0;
        ips200_draw_point((int16) left_line_list[i], i, RGB565_YELLOW);

        if (right_line_list[i] > MT9V03X_W - 1)
            right_line_list[i] = MT9V03X_W - 1;
        else if (right_line_list[i] < 0)
            right_line_list[i] = 0;
        ips200_draw_point((int16) right_line_list[i], i, RGB565_RED);

        if (mid_line_list[i] > MT9V03X_W - 1)
            mid_line_list[i] = MT9V03X_W - 1;
        else if (mid_line_list[i] < 0)
            mid_line_list[i] = 0;
        ips200_draw_point((int16) mid_line_list[i], i, RGB565_GREEN);

        ips200_draw_point(Benzene_turn_point_up1, i, RGB565_GREEN);
        ips200_draw_point(Benzene_turn_point_down1, i, RGB565_GREEN);
    }

    for (uint8 j = MT9V03X_W - 1; j > 1; j--)
    {
        ips200_draw_point(j, Benzene_turn_point_left1, RGB565_GREEN);
        ips200_draw_point(j, Benzene_turn_point_right1, RGB565_GREEN);
    }

}

/******************直道判断****************/
float Straight_Judge (uint8 dir, uint8 start, uint8 end)     //返回值为1表示直道
{
    int i;
    float S = 0, Sum = 0, Err = 0, k = 0;
    switch (dir)
    {
        case 1 :
            k = (float) (left_line_list[start] - left_line_list[end]) / (start - end);
            for (i = 0; i < end - start; i++)
            {
                Err = (left_line_list[start] + k * i - left_line_list[i + start])
                        * (left_line_list[start] + k * i - left_line_list[i + start]);
                Sum += Err;
            }
            S = Sum / (end - start);
            break;
        case 2 :
            k = (float) (right_line_list[start] - right_line_list[end]) / (start - end);
            for (i = 0; i < end - start; i++)
            {
                Err = (right_line_list[start] + k * i - right_line_list[start + i])
                        * (right_line_list[start] + k * i - right_line_list[start + i]);
                Sum += Err;
            }
            S = Sum / (end - start);
            break;
        case 3 :
            k = (float) (mid_line_list[start] - mid_line_list[end]) / (start - end);
            for (i = 0; i < end - start; i++)
            {
                Err = (mid_line_list[start] + k * i - mid_line_list[start + i])
                        * (mid_line_list[start] + k * i - mid_line_list[start + i]);
                Sum += Err;
            }
            S = Sum / (end - start);
    }
    return S;
}

/*******************************元素行列判断******************************************/
void element_cow_col (void)
{
    int i, j;

    for (i = 3; i < 15; i++)     //上方元素行列
    {
        Benzene_turn_flag_up = 0;
        for (j = 3; j < MT9V03X_W - 3; j++)
        {
            if (Benzene_turn_flag_up == 0)
            {
                if (image_two_value[i][j] == black_point && image_two_value[i][j + 1] == white_point
                        && image_two_value[i][j + 2] == white_point)
                {
                    Benzene_turn_flag_up = 1;     //锟斤拷一锟斤拷锟斤拷锟斤拷
                    Benzene_turn_point_up1 = j;
                }
            }
            else if (Benzene_turn_flag_up == 1)
            {
                if (image_two_value[i][j] == black_point && image_two_value[i][j + 1] == white_point
                        && image_two_value[i][j + 2] == white_point)
                {
                    Benzene_turn_flag_up = 2;     //锟节讹拷锟斤拷锟斤拷锟斤拷
                    Benzene_turn_point_up2 = j;
                    break;
                }
            }
        }
        if (Benzene_turn_flag_up == 2)
            break;
    }

    for (i = MT9V03X_H - 10; i < MT9V03X_H - 3; i++)     //下方元素行列
    {
        Benzene_turn_flag_down = 0;
        for (j = 3; j < MT9V03X_W - 3; j++)
        {
            if (Benzene_turn_flag_down == 0)
            {
                if (image_two_value[i][j] == black_point && image_two_value[i][j + 1] == white_point
                        && image_two_value[i][j + 2] == white_point)
                {
                    Benzene_turn_flag_down = 1; //锟斤拷一锟斤拷锟斤拷锟斤拷
                    Benzene_turn_point_down1 = j;
                }
            }
            else if (Benzene_turn_flag_down == 1)
            {
                if (image_two_value[i][j] == black_point && image_two_value[i][j + 1] == white_point
                        && image_two_value[i][j + 2] == white_point)
                {
                    Benzene_turn_flag_down = 2; //锟节讹拷锟斤拷锟斤拷锟斤拷
                    Benzene_turn_point_down2 = j;
                    break;
                }
            }
        }
        if (Benzene_turn_flag_down == 2)
            break;
    }

    for (j = 3; j < 10; j++)     //左方元素行列
    {
        Benzene_turn_flag_left = 0;
        for (i = 10; i < MT9V03X_H + 10; i++)
        {
            if (Benzene_turn_flag_left == 0)
            {
                if (image_two_value[i][j] == black_point && image_two_value[i + 1][j] == white_point
                        && image_two_value[i + 2][j] == white_point)
                {
                    Benzene_turn_flag_left = 1;
                    Benzene_turn_point_left1 = i;
                    break;
                }
                else
                {
                    Benzene_turn_flag_left = 0;
                }
            }
        }
        if (Benzene_turn_flag_left == 1)
            break;
    }

    for (j = MT9V03X_W - 10; j < MT9V03X_W - 3; j++)  //右方元素行列
    {
        Benzene_turn_flag_right = 0;
        for (i = 10; i < MT9V03X_H - 10; i++)
        {
            if (Benzene_turn_flag_right == 0)
            {
                if (image_two_value[i][j] == black_point && image_two_value[i + 1][j] == white_point
                        && image_two_value[i + 2][j] == white_point)
                {
                    Benzene_turn_flag_right = 1;
                    Benzene_turn_point_right1 = i;
                    break;
                }
                else
                {
                    Benzene_turn_flag_right = 0;
                }
            }
        }
        if (Benzene_turn_flag_right == 1)
            break;
    }
}

/***********************************ֱ直角转弯逻辑***********************************************************************************************************************************************************/
void stright_angle (void)
{
    int i, j;
    left_stright = 0;
    right_stright = 0;
    int horizontal_white_count = 0;  // 水平黑点计数
    
    if (Benzene_turn_flag_left == 1 && Benzene_turn_flag_right == 0 && Benzene_turn_flag_up == 0)  //左直角
    {
        for (i = Benzene_turn_point_left1 - 10; i > 15; i--)
        {
            for (j = 0; j < MT9V03X_W - 3; j++)
            {
                if (image_two_value[i][j] == black_point && image_two_value[i][j + 1] == white_point
                        && image_two_value[i][j + 2] == white_point)
                {
                    left_stright = 0;
                }
                else
                {
                    left_stright = 1;
                    break;
                }
            }
            if (left_stright == 1)
                break;
        }

    }
    else if (Benzene_turn_flag_left == 0 && Benzene_turn_flag_right == 1 && Benzene_turn_flag_up == 0
            && cross_flag == 0) //右直角
    {

        for (i = Benzene_turn_point_right1 - 10; i > 15; i--)
        {
            for (j = 0; j < MT9V03X_W - 3; j++)
            {
                if (image_two_value[i][j] == black_point && image_two_value[i][j + 1] == white_point
                        && image_two_value[i][j + 2] == white_point)
                {
                    right_stright = 0;
                }
                else
                {
                    right_stright = 1;
                    break;
                }
            }
            if (right_stright == 1)
                break;
        }
    }

    if (left_stright == 1)
    {
        Add_Line(1, Benzene_turn_point_left1, mid_line_list[MT9V03X_H - 2], MT9V03X_H - 2);
        Add_Line(1, 0, 1, Benzene_turn_point_left1);
    }
    else if (right_stright == 1)
    {
        Add_Line(MT9V03X_W - 1, Benzene_turn_point_right1, mid_line_list[MT9V03X_H - 2], MT9V03X_H - 2);
        Add_Line(MT9V03X_W - 1, 0, MT9V03X_W - 1, Benzene_turn_point_right1);
    }
    
}

/*******************************************************横T字路口"|——""——|"*********************************************************************************************************************/

void T_corner_h(void)
{
    //待完善
//    // 获取当前跑的总距离
//        int32_t current_dist = get_total_distance();
//
//        // 获取当前目标路口的预设距离
//        int32_t target = target_distances[h_count];
//
//        // --- 核心约束 ---
//        // 只有当 (当前距离 > 目标 - 窗口) 且 (当前距离 < 目标 + 窗口) 时
//        // 才允许开启图像检测逻辑
//        uint8_t enable_detection = 0;
//
//        if (current_dist > (target - dist_window) && current_dist < (target + dist_window))
//        {
//            enable_detection = 1;
//        }
//
//        // 如果不在距离窗口内，强制清零标志位，直接退出
//        if (enable_detection == 0)
//        {
//            left_T_h_flag = 0;
//            right_T_h_flag = 0;
//            return; // 直接退出
//        }


    // ==========================================
    // 第一步：特征识别 (每一帧都独立判断)
    // ==========================================
    left_T_h_flag = 0;
    right_T_h_flag = 0;

    // 设定有效的检测区域 (避免太远误判，太近补线来不及)
    uint8_t valid_min = 25;
    uint8_t valid_max = 80;

    // 识别左横T (左边有拐点，右边无拐点)
    if (Benzene_turn_flag_left == 1 && Benzene_turn_flag_right == 0 &&
        Benzene_turn_point_left1 > valid_min && Benzene_turn_point_left1 < valid_max)
    {
        left_T_h_flag = 1;
    }
    // 识别右横T
    else if (Benzene_turn_flag_right == 1 && Benzene_turn_flag_left == 0 &&
             Benzene_turn_point_right1 > valid_min && Benzene_turn_point_right1 < valid_max)
    {
        right_T_h_flag = 1;
    }

    // 简单的防抖：当前帧有特征，计数器+1；没有，清零
    // 有特征就开始补线，利用物理惯性
    uint8_t current_has_feature = (left_T_h_flag || right_T_h_flag);

    // ==========================================
    // 第二步：执行补线
    // ==========================================
    if (current_has_feature)
    {
        is_in_h_junction = 1; // 标记：到路口了

        // 读取当前路口应该做的动作
        // 0: 直行 (封死路口), 1: 转弯 (切入路口)
        uint8_t action = T_turn_h_sequence[h_count];
        if (T_count >= 2)
            action = 0;
        else
            action = 1;

        // --- 左路口处理 ---
        if (left_T_h_flag)
        {
            if (action == 1) // 【转弯模式】
            {
                // 往左补线，引进去
                Add_Line(1, Benzene_turn_point_left1, MT9V03X_W / 2 + 10, MT9V03X_H - 1);
                Add_Line(1, 0, 1, Benzene_turn_point_left1);
            }
            else // 【直行模式】
            {
                // 强行封死左边缺口：连接底部左边线起点 -> 左上角
                // 这样中线计算出来就是直的
                int start_col = left_line_list[MT9V03X_H - 5];
                if (start_col < 0) start_col = 0;

                // 使用 Add_left_Line 强制修改左边线数组
                Add_left_Line(start_col, MT9V03X_H - 5, Benzene_turn_point_left1, 0);
            }
        }
        // --- 右路口处理 ---
        else if (right_T_h_flag)
        {
            if (action == 1) // 【转弯模式】
            {
                // 往右补线，引进去
                Add_Line(MT9V03X_W - 2, Benzene_turn_point_right1, MT9V03X_W / 2 - 10, MT9V03X_H - 1);
                Add_Line(MT9V03X_W - 2, 0, MT9V03X_W - 2, Benzene_turn_point_right1);
            }
            else // 【直行模式】
            {
                // 强行封死右边缺口
                int start_col = right_line_list[MT9V03X_H - 5];
                if (start_col > MT9V03X_W - 1) start_col = MT9V03X_W - 1;

                Add_right_Line(start_col, MT9V03X_H - 5, Benzene_turn_point_right1, 0);
            }
        }
    }

    // ==========================================
    // 第三步：路口计数 (下降沿检测)
    // ==========================================
    // 逻辑：如果上一帧状态是“在路口里”，但这帧“没特征了”
    // 且 拐点位置已经很靠下了(说明车子开过去了)，或者完全消失了
    // 那么认为我们通过了这个路口

    else if (current_has_feature == 0 && is_in_h_junction == 1) // 刚才在路口，现在没特征了
    {
        // 这里简单处理：只要特征消失，就认为过完了

//        h_count++; // 切换到下一个路口序列
//        if (h_count >= total_h_count) h_count = 0;

        is_in_h_junction = 0; // 重置状态，等待下一个路口
    }
}

/**********************************************竖T字路口*************************************************************************************************************/
void T_corner ()
{
    int i, j, i1, j1;
    int point = 0;
    uint8_t valid_min = 25;
    uint8_t valid_max = 80;

    if (Benzene_turn_flag_left == 1 && Benzene_turn_flag_right == 1 && Benzene_turn_flag_up == 0 && cross_flag == 0
    && Benzene_turn_point_left1 > valid_min && Benzene_turn_point_right1 > valid_min && Benzene_turn_point_left1 < valid_max && Benzene_turn_point_right1 < valid_max)
    {
        T_flag = 1; //初步符合
    }

    // 进一步校验
    if (T_flag == 1)
    {
        // 校验1: 顶部是否存在开口
        j1 = MT9V03X_W / 2;
        for (i1 = 5; i1 < MT9V03X_H - 10; i1++)
        {
            if (image_two_value[i1][j1] == black_point && image_two_value[i1 + 1][j1] == white_point
                    && image_two_value[i1 + 2][j1] == white_point)
            {
                point = i1;
                break;
            }

        }
        if (point == 0)
        {
            T_flag = 0;
        }
        // 如果顶部白点位置偏离左右拐点连线太远，说明不是标准T字
        if (point > Benzene_turn_point_left1 + 10 || point < Benzene_turn_point_left1 - 10
                || point > Benzene_turn_point_right1 + 10 || point < Benzene_turn_point_right1 - 10)
        {
            T_flag = 0;
        }
    }


    if (T_flag == 1)
    {
        T_State = 1;      // 切换状态机到转弯模式
        ResetYawZero();   // 清零陀螺仪积分，准备开始计算转弯角度
        current_yaw = 0;  // 确保当前yaw为0
        T_turn_flag = T_turn_sequence[T_count];
    }

    // ============================================================
    // 第二步：执行转弯补线 (状态锁定)
    // ============================================================
    if (T_State == 1)
    {
        if (T_turn_flag == 0)   //左转
        {
            Add_Line(1, Benzene_turn_point_left1, mid_line_list[MT9V03X_H - 2], MT9V03X_H - 2);
            Add_Line(1, 0, 1, Benzene_turn_point_left1);
            if (current_yaw > 30)
            {
                T_State = 0;
                T_flag = 0;
                T_count++;
//                if(T_count >= total_T_count) T_count = 0;
                current_yaw = 0;
                ResetYawZero();

            }
        }
        else if (T_turn_flag == 1)   //右转
        {
            Add_Line(MT9V03X_W - 1, Benzene_turn_point_right1, mid_line_list[MT9V03X_H - 2], MT9V03X_H - 2);
            Add_Line(MT9V03X_W - 1, 0, MT9V03X_W - 1, Benzene_turn_point_right1);
            if (current_yaw < -30)
            {
                T_State = 0;
                T_flag = 0;
                T_count++;
//                if(T_count >= total_T_count) T_count = 0;
                current_yaw = 0;
                ResetYawZero();

            }
        }

    }
}

/***************************************************十字路口***********************************************************************************************/
void cross (void)
{
    if (left_stright == 0 && right_stright == 0)
    {
        if (Benzene_turn_flag_up >= 1 && Benzene_turn_flag_down >= 1 && Benzene_turn_flag_left == 1
                && Benzene_turn_flag_right == 1 && Benzene_turn_point_left1 > 40 && Benzene_turn_point_right1 > 40)
        {
            cross_flag = 1;
        }
        else
        {
            cross_flag = 0;
        }
    }

    if (cross_flag == 1)
    {
        Add_Line(Benzene_turn_point_up1, 0, mid_line_list[MT9V03X_H - 2], MT9V03X_H - 2);
    }

}

/***********************************锟斤拷锟竭猴拷锟斤拷*********************************************************************************************************************************************************/
void Add_Line (uint8 x1, uint8 y1, uint8 x2, uint8 y2)   //锟斤拷锟斤拷锟竭猴拷锟斤拷
{
    uint8 i, max, a1, a2;
    uint8 hx;
    if (x1 >= MT9V03X_W - 1)   //锟斤拷始锟斤拷位锟斤拷校锟斤拷锟斤拷锟脚筹拷锟斤拷锟斤拷越锟斤拷目锟斤拷锟�?
        x1 = MT9V03X_W - 1;
    else if (x1 <= 0)
        x1 = 0;
    if (y1 >= MT9V03X_H - 1)
        y1 = MT9V03X_H - 1;
    else if (y1 <= 0)
        y1 = 0;
    if (x2 >= MT9V03X_W - 1)
        x2 = MT9V03X_W - 1;
    else if (x2 <= 0)
        x2 = 0;
    if (y2 >= MT9V03X_H - 1)
        y2 = MT9V03X_H - 1;
    else if (y2 <= 0)
        y2 = 0;
    a1 = y1;
    a2 = y2;
    if (a1 > a2)   //锟斤拷锟疥互锟斤拷
    {
        max = a1;
        a1 = a2;
        a2 = max;
    }

    for (i = a1; i <= a2; i++)   //锟斤拷锟斤拷斜锟绞诧拷锟竭硷拷锟斤拷
    {
        hx = (i - y1) * (x2 - x1) / (y2 - y1) + x1;
        if (hx >= MT9V03X_W)
            hx = MT9V03X_W;
        else if (hx <= 0)
            hx = 0;
        mid_line_list[i] = hx;
    }
}

void Add_left_Line (uint8 x1, uint8 y1, uint8 x2, uint8 y2)   //锟斤拷锟斤拷锟竭猴拷锟斤拷
{
    uint8 i, max, a1, a2;
    uint8 hx;
    if (x1 >= MT9V03X_W - 1)   //锟斤拷始锟斤拷位锟斤拷校锟斤拷锟斤拷锟脚筹拷锟斤拷锟斤拷越锟斤拷目锟斤拷锟�?
        x1 = MT9V03X_W - 1;
    else if (x1 <= 0)
        x1 = 0;
    if (y1 >= MT9V03X_H - 1)
        y1 = MT9V03X_H - 1;
    else if (y1 <= 0)
        y1 = 0;
    if (x2 >= MT9V03X_W - 1)
        x2 = MT9V03X_W - 1;
    else if (x2 <= 0)
        x2 = 0;
    if (y2 >= MT9V03X_H - 1)
        y2 = MT9V03X_H - 1;
    else if (y2 <= 0)
        y2 = 0;
    a1 = y1;
    a2 = y2;
    if (a1 > a2)   //锟斤拷锟疥互锟斤拷
    {
        max = a1;
        a1 = a2;
        a2 = max;
    }

    for (i = a1; i <= a2; i++)   //锟斤拷锟斤拷斜锟绞诧拷锟竭硷拷锟斤拷
    {
        hx = (i - y1) * (x2 - x1) / (y2 - y1) + x1;
        if (hx >= MT9V03X_W)
            hx = MT9V03X_W;
        else if (hx <= 0)
            hx = 0;
        left_line_list[i] = hx;
    }
}

void Add_right_Line (uint8 x1, uint8 y1, uint8 x2, uint8 y2)   //锟斤拷锟斤拷锟竭猴拷锟斤拷
{
    uint8 i, max, a1, a2;
    uint8 hx;
    if (x1 >= MT9V03X_W - 1)   //锟斤拷始锟斤拷位锟斤拷校锟斤拷锟斤拷锟脚筹拷锟斤拷锟斤拷越锟斤拷目锟斤拷锟�?
        x1 = MT9V03X_W - 1;
    else if (x1 <= 0)
        x1 = 0;
    if (y1 >= MT9V03X_H - 1)
        y1 = MT9V03X_H - 1;
    else if (y1 <= 0)
        y1 = 0;
    if (x2 >= MT9V03X_W - 1)
        x2 = MT9V03X_W - 1;
    else if (x2 <= 0)
        x2 = 0;
    if (y2 >= MT9V03X_H - 1)
        y2 = MT9V03X_H - 1;
    else if (y2 <= 0)
        y2 = 0;
    a1 = y1;
    a2 = y2;
    if (a1 > a2)   //锟斤拷锟疥互锟斤拷
    {
        max = a1;
        a1 = a2;
        a2 = max;
    }

    for (i = a1; i <= a2; i++)   //锟斤拷锟斤拷斜锟绞诧拷锟竭硷拷锟斤拷
    {
        hx = (i - y1) * (x2 - x1) / (y2 - y1) + x1;
        if (hx >= MT9V03X_W)
            hx = MT9V03X_W;
        else if (hx <= 0)
            hx = 0;
        right_line_list[i] = hx;
    }
}

