#include "image.h"
#include "zf_device_ips200.h"
#include "zf_common_font.h"
#include "Gyro.h"
#include "enconder.h"
#include "ins.h"


//��ֵ����ر���?
uint8 threshold ;
uint8 limite_th=120;
uint8 image_two_value[MT9V03X_H][MT9V03X_W]; 

//ɨ����ر���?
int16 left_line_list[MT9V03X_H]; //左线数组
int16 right_line_list[MT9V03X_H];//右线数组
int16 mid_line_list[MT9V03X_H];  //中线数组
int16 rode_wide_list[MT9V03X_H]; //车道宽度数组
int16 left_lost_flag[MT9V03X_H]; //左线丢失标志位
int16 right_lost_flag[MT9V03X_H];//右线丢失标志位
int16 left_end_flag=0; //左线结束标志位
int16 right_end_flag=0;//右线结束标志位
uint8 left_end=0;//左线结束点
uint8 right_end=0;//右线结束点
uint8 end_line=0;//图像结束点

int16 cir_out_encoder=0;

//����Ȩ������
float mid_weight_list[MT9V03X_H]=
{
     12,12,11.5,11.5,11.5,11.5,11,11,11,11,                       //0-9行
     10.5,10.5,10.5,10.5,10.5,10.5,10,10,10,10,   //10-19行
     9.5,9.5,9.5,9.5,9.5,9,9,9,9,9,             //20-29行
     8.5,8.5,8.5,8.5,8.5,8,8,8,8,8,             //30-39行
     7.5,7.5,7.5,7.5,7.5,7,7,7,7,7,             //40-49行
     6.5,6.5,6.5,6.5,6.5,6,6,6,6,6,             //50-59行
     5.5,5.5,5.5,5.5,5.5,5,5,5,5,5,             //60-69行
     4.5,4.5,4.5,4.5,4.5,4,4,4,4,4,             //70-79行
     3.5,3.5,3.5,3.5,3,3,3,3,2.5,2.5,           //80-89行
     2,2,2,1.5,1.5,1.5,1,1,0.5,0.5              //90-99行
};




float mid_value = 0;
float image_error = 0;
int16 mid = 47;

//元素行列标志位
int16 Benzene_turn_flag_up=0;
int16 Benzene_turn_point_up1=0;
int16 Benzene_turn_point_up2=0;
int16 Benzene_turn_flag_down=0;
int16 Benzene_turn_point_down1=0;
int16 Benzene_turn_point_down2=0;
int16 Benzene_turn_flag_left=0;
int16 Benzene_turn_point_left1=0;
int16 Benzene_turn_point_left2=0;
int16 Benzene_turn_flag_right=0;
int16 Benzene_turn_point_right1=0;
int16 Benzene_turn_point_right2=0;

//直角标志位
int16 left_stright= 0;
int16 right_stright=0;
int16 left_strflag=0;
int16 right_strflag=0;
float stright_anglez=0;


//圆环标志位
int16 left_cir_flag=0;
int16 right_cir_flag=0;
int16 left_cir_point=0;
int16 right_cir_point=0;
int16 left_cir_height=0;
int16 last_left_cir_height=0;
int16 right_cir_height=0;
int16 last_right_cir_height=0;
int16 Benzene_height_cont=0;
int16 in_cir_left=0;
int16 in_cir_right=0;
int16 out_cir_left=0;
int16 out_cir_right=0;
int16 out_cir_encoder=0;


//十字标志位
int16 cross_flag=0;


//断路标志位
int16 check_row=0;
int16 start_check_row= MT9V03X_H-1;
int16 end_check_row= 10;
float percent_lost= 0.f;
int16 break_cont= 0;
int16 break_flag=0;

/**************************************大津法*************************************************/
uint8_t otsuThreshold(uint8_t *image, uint16_t width, uint16_t height)
{
    #define GrayScale 256

    int pixelCount[GrayScale] = {0};       // ͳ��ÿ���Ҷ�ֵ���ֵĴ���
    float pixelPro[GrayScale] = {0.0f};    // ÿ���Ҷ�ֵ�ĸ���
    int Sumpix = width * height;           // ��������
    uint8_t Threshold = 0;                 // ���շ��ص���ֵ
    uint8_t* data = image;                 // ͼ������ָ��

    float u = 0.0f;                        // ͼ��ȫ��ƽ���Ҷ�
    float maxVariance = 0.0f;              // �����䷽��
    float w0 = 0.0f;                       // ǰ���������?
    float avgValue = 0.0f;                 // ǰ������ļ�Ȩƽ���Ҷ�?

    // Step 1: ͳ��ÿ���Ҷ�ֵ���ֵĴ���
    for (int i = 0; i < Sumpix; ++i)
    {
        pixelCount[data[i]]++;
    }

    // Step 2: ����ÿ���Ҷ�ֵ�ĸ��ʣ���������ͼ���ƽ���Ҷ�? u
    for (int i = 0; i < GrayScale; ++i)
    {
        pixelPro[i] = (float)pixelCount[i] / Sumpix;
        u += i * pixelPro[i];
    }

    // Step 3: �������п��ܵĻҶ�ֵ��Ϊ��ֵ��������䷽��ҳ����ֵ��Ӧ����ֵ
    for (int i = 0; i < GrayScale; ++i)
    {
        w0 += pixelPro[i];         // ����ǰ������
        avgValue += i * pixelPro[i]; // ����ǰ����Ȩƽ���Ҷ�

        if (w0 == 0 || w0 == 1) continue; // ���������������ֹ�������

        // ������䷽��?
        float variance = powf((avgValue - u * w0), 2) / (w0 * (1 - w0));

        if (variance > maxVariance)
        {
            maxVariance = variance;
            Threshold = (uint8_t)i;
        }
    }
    return Threshold;
}

/******************************二值化***********************************/
void Image_Binarization(void)
{
  uint8 i,j;
  
  threshold = otsuThreshold(mt9v03x_image[0],MT9V03X_W,MT9V03X_H);
  if(threshold<limite_th) threshold=limite_th;
  for(i=0;i<MT9V03X_H-1;i++)
  {
    for(j=0;j<MT9V03X_W-1;j++)
    {
      if(j<10 || j>84)
      {
        image_two_value[i][j] = mt9v03x_image[i][j]>(threshold+10) ? 255:0; //图像两边阈值较小
      }
      else
      {
        image_two_value[i][j] = mt9v03x_image[i][j]>threshold ? 255:0;
      }
    }
  }
}


/******************************扫描线*********************************************/
void find_line(void)
{
  uint8 current_y = MT9V03X_H - 2;
  left_end_flag=0;
  //��ʼ���������� 
    memset(left_line_list, -1, sizeof(left_line_list));
    memset(right_line_list, -1, sizeof(right_line_list));
    memset(mid_line_list, 0, sizeof(mid_line_list));
    
    while(current_y > 5 && left_end_flag==0)
    {
      uint8 find_left_end = MT9V03X_W - 2;
      if(right_cir_flag==1 || out_cir_right==1)
      {
        find_left_end= mid+10;//10
      }
      if(in_cir_left==1 && Benzene_turn_flag_down==2)
      {
        find_left_end= mid+5;//10
      }
      for(uint8 i=2;i<find_left_end;i++)
      {
        if(image_two_value[current_y][i]==black_point&& image_two_value[current_y][i+1]==white_point && image_two_value[current_y][i+2]==white_point)
        {
          left_line_list[current_y] = i;
          left_end_flag= 1;
          left_end= current_y;
          break;
        }
      }
      if(current_y<60) break;
      if(left_end_flag==0) current_y-=5;
    }
    //补低线
    if(left_end < MT9V03X_H-10)
    {
      for(uint8 j = MT9V03X_H-1;j>left_end+1;j--)
      {
        left_line_list[j] = left_line_list[left_end];
      }
    }
    
    current_y = MT9V03X_H - 2;
    right_end_flag = 0;
    while(current_y>5 && right_end_flag==0)
    {
      uint8 find_right_end = 2;
      if(left_cir_flag==1 || out_cir_left==1 || in_cir_right==1)
      {
        find_right_end= mid-5;
      }
      for(uint8 i=MT9V03X_W-2;i>find_right_end;i--)
      {
        if(image_two_value[current_y][i]==black_point&& image_two_value[current_y][i-1]==white_point && image_two_value[current_y][i-2]==white_point)
        {
          right_line_list[current_y] = i;
          right_end_flag=1;
          right_end = current_y;
          break;
        }
      }
      if(current_y<60) break;
      if(right_end_flag==0) current_y-=5;
    }
    
    //补低线
    if(right_end < MT9V03X_H-10)
    {
      for(uint8 j=MT9V03X_H-1;j>right_end+1;j--)
      {
        right_line_list[j] = right_line_list[right_end];
      }
    }
    
    //确定结束点
    end_line= (left_end>right_end) ? right_end: left_end;
    
    //从结束点开始扫描
    if (right_end_flag == 1)//����ұ߽���ѵ��� ��ʼ�����ұ���
    {
        for (uint8 i = right_end - 1; i > 0; i--)
        {
            int start_col;
            int end_col;
            //�����߲���
            start_col = right_line_list[i + 1] + 10;
            end_col = right_line_list[i + 1] - 10;

            //防止越界
            if (end_col < 2)end_col = 2;
            if (start_col > MT9V03X_W - 2)start_col = MT9V03X_W - 2;

            for (uint8 j = start_col; j > end_col; j--)
            {
                if (image_two_value[i][j] == black_point && image_two_value[i][j - 1] == white_point && image_two_value[i][j-2]==white_point)//�ҵ���һ����ɫ     
                {
                    right_line_list[i] = j;
                    break;
                }
            }
            if (right_line_list[i] == -1 && right_line_list[i + 1] != -1 && i > 20)//���߲���
            {

                int max_jump = (i >= MT9V03X_H / 2) ? JUMP_STEP_MAX_xia : JUMP_STEP_max_shang;//���ж���ͼ����ĸ����?
                int step = (i >= MT9V03X_H / 2) ? JUMP_STEP : JUMP_STEP_shang;
                uint8 found = 0;
                int jump_end = i - max_jump;
                if (jump_end < 0)jump_end = 0;


                for (uint8 jump = i; jump > jump_end; jump -= step)
                {
                    int start_target = right_line_list[i + 1] + 15;
                    int end_target = right_line_list[i + 1] - 15;

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
                    
                    if(left_cir_flag==1 || out_cir_left==1 )
                    {
                      start_target= right_line_list[i + 1]+5;
                      end_target= right_line_list[i + 1]-5;
                    }
                    if(in_cir_right==1)
                    {
                      start_target= right_line_list[i + 1]+15;
                      end_target= right_line_list[i + 1]-15;
                    }
                    
                    //�޷�
                    if (start_target > MT9V03X_W - 2)start_target = MT9V03X_W - 2;
                    if (end_target < 2)end_target = 2;


                    int junp_found = -1;
                    for (uint8 j = start_target; j > end_target; j--)
                    {
                        
                        if (image_two_value[jump][j] == black_point && image_two_value[jump][j - 1] == white_point&& image_two_value[jump][j-2]==white_point)//�ҵ���һ����ɫ     
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
                        else  Add_right_Line(right_line_list[i + 1], i + 1, junp_found, jump);

                        i = jump;
                        found = 1;
                        break;
                    }
                    if(!found && i>40)
                    {
                      right_line_list[i] = right_line_list[i + 1];
                    }
                }
            }
            if (right_line_list[i] == -1) break;//全扫描完了都没扫到，退出
        }

    }
    
    //从结束点开始扫描
    if (left_end_flag == 1)
    {
        for (uint8 i = left_end - 1; i > 0; i--)
        {
            //�����߲���
            int start_col;
            int end_col;

            start_col = left_line_list[i + 1] - 10;
            end_col = left_line_list[i + 1] + 10;
            

            //防止越界
            if (end_col > MT9V03X_W - 2)end_col = MT9V03X_W - 2;
            if (start_col < 2)start_col = 2;

            for (uint8 j = start_col; j < end_col; j++)
            {
                if (image_two_value[i][j] == black_point && image_two_value[i][j + 1] == white_point && image_two_value[i][j+2]==white_point)//�ҵ���һ����ɫ     
                {
                    left_line_list[i] = j;
                    break;
                }
            }
            if (left_line_list[i] == -1 && left_line_list[i + 1] != -1 && i > 20)//���߲���
            {

                int max_jump = (i >= MT9V03X_H / 2) ? JUMP_STEP_MAX_xia : JUMP_STEP_max_shang;//���ж���ͼ����ĸ����?
                int step = (i >= MT9V03X_H / 2) ? JUMP_STEP : JUMP_STEP_shang;
                uint8 found = 0;
                int jump_end = i - max_jump;
                if (jump_end < 0)jump_end = 0;

                for (uint8 jump = i; jump > jump_end; jump -= step)
                {
                    int start_target = left_line_list[i + 1] - 15;
                    int end_target = left_line_list[i + 1] + 15;

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
                    
                    if(right_cir_flag==1 || out_cir_right==1 )
                    {
                        start_target= left_line_list[i+1]-5;
                        end_target= left_line_list[i+1]+5;
                    }
                    if(in_cir_left==1 && Benzene_turn_flag_down==2)
                    {
                      start_target= left_line_list[i+1]-15;
                      end_target= left_line_list[i+1]+15;
                    }

                    //�޷�
                    if (start_target < 2)start_target = 2;
                    if (end_target > MT9V03X_W - 2)end_target = MT9V03X_W - 2;


                    int junp_found = -1;
                    for (uint8 j = start_target; j < end_target; j++)
                    {
                        if (image_two_value[jump][j] == black_point && image_two_value[jump][j + 1] == white_point&& image_two_value[jump][j+2]==white_point)//�ҵ���һ����ɫ     
                        {
                            junp_found = j;
                            break;
                        }
                    }
                    if (junp_found != -1)
                    {
                        if (i + 5 < MT9V03X_H - 1)
                            Add_left_Line(left_line_list[i + 5], i + 1, junp_found, jump);
                        else Add_left_Line(left_line_list[i + 1], i + 1, junp_found, jump);

                        i = jump;// ����������
                        found = 1;
                        break;// �˳����ߴ���
                    }
                    if(!found && i>40)
                    {
                      left_line_list[i] = left_line_list[i + 1];
                    }
                }
            }
            //���߲���

            if (left_line_list[i] == -1) break;//ȫ�����껹��û�ѵ� ����
        }
    }
    for (int i = MT9V03X_H-1; i > 0; i--)
    {
        rode_wide_list[i] = 0;
        if (right_line_list[i] != -1 && left_line_list[i] != -1 && right_line_list[i] > left_line_list[i] && right_line_list[i] < MT9V03X_W - 2 && left_line_list[i]>2 )
        {
            end_line = i;
            rode_wide_list[i] = right_line_list[i] - left_line_list[i];
            mid_line_list[i] = (right_line_list[i] + left_line_list[i]) / 2;  
        }
        else mid_line_list[i] = 0;
    }
  
  
}





/******************************中线计算**************************************/
void get_error(void)
{
  int16 i=0;
  mid_value = 0;
  float weight_middle_sum=0;//��Ȩ�����ۼ�ֵ
  float weight_sum=0;//Ȩ���ۼ�ֵ

    for(i=99;i>0;i--)
    {
      if(mid_line_list[i] != 0)
      {
        weight_middle_sum += mid_line_list[i]*mid_weight_list[i];
        weight_sum +=  mid_weight_list[i];
      }
    }
  
  mid_value = weight_middle_sum/weight_sum;
  image_error = 60-mid_value;//һ֡ͼ���ƫ���?
}


/****************************************************画线******************************************************************/
void drawkline(void)
{
  for(uint8 i = MT9V03X_H-1 ; i > 1 ; i--)
  {
    if(left_line_list[i]>MT9V03X_W-1) left_line_list[i]= MT9V03X_W-1;
    else if(left_line_list[i]<0) left_line_list[i] = 0;
    ips200_draw_point((int16)left_line_list[i] , i , RGB565_YELLOW);
 
    if(right_line_list[i]>MT9V03X_W-1) right_line_list[i]= MT9V03X_W-1;
    else if(right_line_list[i]<0) right_line_list[i] = 0;
    ips200_draw_point((int16)right_line_list[i] , i , RGB565_RED);

    if(mid_line_list[i]>MT9V03X_W-1) mid_line_list[i]= MT9V03X_W-1;
    else if(mid_line_list[i]<0) mid_line_list[i] = 0;
    ips200_draw_point((int16)mid_line_list[i] , i , RGB565_GREEN);
  }
} 

/*****************直道判断******************/
float Straight_Judge(uint8 dir, uint8 start, uint8 end)     //返回值为1表示直道
{
    int i;
    float S = 0, Sum = 0, Err = 0, k = 0;
    switch (dir)
    {
    case 1:k = (float)(left_line_list[start] - left_line_list[end]) / (start - end);
        for (i = 0; i < end - start; i++)
        {
            Err = (left_line_list[start] + k * i -left_line_list[i + start]) * (left_line_list[start] + k * i - left_line_list[i + start]);
            Sum += Err;
        }
        S = Sum / (end - start);
        break;
    case 2:k = (float)(right_line_list[start] - right_line_list[end]) / (start - end);
        for (i = 0; i < end - start; i++)
        {
            Err = (right_line_list[start] + k * i - right_line_list[start + i]) * (right_line_list[start] + k * i - right_line_list[start + i]);
            Sum += Err;
        }
        S = Sum / (end - start);
        break;
    case 3:k = (float)(mid_line_list[start] - mid_line_list[end])/(start-end);
        for(i = 0;i<end-start;i++)
        {
            Err = (mid_line_list[start] + k * i - mid_line_list[start + i]) * (mid_line_list[start] + k * i - mid_line_list[start + i]);
            Sum += Err;
        }
        S = Sum / (end - start);
    }
    return S;
}

/*******************************元素行列判断******************************************/
void element_cow_col(void)
{
  int i,j;
  
  for(i=0;i<20;i++)     //����10����ɨ��
  {
    Benzene_turn_flag_up=0;
    for(j=0;j<MT9V03X_W-3;j++)
    {
      if(Benzene_turn_flag_up==0)
      {
        if(image_two_value[i][j]==black_point && image_two_value[i][j+1]==white_point && image_two_value[i][j+2]==white_point)
        {
          Benzene_turn_flag_up= 1;//��һ������
          Benzene_turn_point_up1 = j;
        }
      }
      else if(Benzene_turn_flag_up==1)
      {
        if(image_two_value[i][j]==black_point && image_two_value[i][j+1]==white_point && image_two_value[i][j+2]==white_point) 
        {
          Benzene_turn_flag_up = 2;//�ڶ�������
          Benzene_turn_point_up2 = j;
          break;
        }
      }
    }
    if(Benzene_turn_flag_up==2) break;    
  }
  
  for(i=MT9V03X_H-20;i<MT9V03X_H-1;i++)     //����10����ɨ��
  {
    Benzene_turn_flag_down=0;
    for(j=0;j<MT9V03X_W-3;j++)
    {
      if(Benzene_turn_flag_down==0)
      {
        if(image_two_value[i][j]==black_point && image_two_value[i][j+1]==white_point&& image_two_value[i][j+2]==white_point)
        {
          Benzene_turn_flag_down= 1; //��һ������
          Benzene_turn_point_down1 = j;
        }
      }
      else if(Benzene_turn_flag_down==1)
      {
        if(image_two_value[i][j]==black_point && image_two_value[i][j+1]==white_point && image_two_value[i][j+2]==white_point)
        {
          Benzene_turn_flag_down = 2;//�ڶ�������
          Benzene_turn_point_down2 = j;
          break;
        }
      }
    }
    if(Benzene_turn_flag_down==2) break;    
  }
  
  for(j=0;j<20;j++)     //������������
  {
    Benzene_turn_flag_left=0;
    for(i=MT9V03X_H-10;i>10;i--)
    {
      if(Benzene_turn_flag_left==0)
      {
         if(image_two_value[i][j]==black_point && image_two_value[i-1][j]==white_point && image_two_value[i-2][j]==white_point)
         {
            Benzene_turn_flag_left=1;
            Benzene_turn_point_left1=i;
            break;
         }
         else
         {
            Benzene_turn_flag_left=0;
         }
      }
    }
    if(Benzene_turn_flag_left==1) break;
  }
  
  for(j=MT9V03X_W-20;j<MT9V03X_W-1;j++) //�Ҳ����������?
  {
    Benzene_turn_flag_right=0;
    for(i=MT9V03X_H-10;i>10;i--)
    {
      if(Benzene_turn_flag_right==0)
      {
         if(image_two_value[i][j]==black_point && image_two_value[i-1][j]==white_point && image_two_value[i-2][j]==white_point)
         {
            Benzene_turn_flag_right=1;
            Benzene_turn_point_right1=i;
            break;
         }
         else
         {
            Benzene_turn_flag_right=0;
         }
      }
    }
    if(Benzene_turn_flag_right==1) break;
  }
}


/****************************************ֱ��*****************************************************************************************************************************************************/
void stright_angle(void)
{
  int i,j;
  left_stright=0;
  right_stright=0;
  int horizontal_white_count = 0; // 水平黑点计数
    
  if(Benzene_turn_flag_left==1 && Benzene_turn_flag_right==0 && Benzene_turn_flag_up==0 &&out_cir_left==0 && out_cir_right==0  && cross_flag==0)  //左直角
    {
      for(i=Benzene_turn_point_left1-10;i>15;i--)
      {
        for(j=0;j<MT9V03X_W-3;j++)
        {
          if(image_two_value[i][j]==black_point&& image_two_value[i][j+1]==white_point&& image_two_value[i][j+2]==white_point)
          {
              left_stright= 0;              
          }
          else
          {
              left_stright=1;            
              break;
          }
        }
        if(left_stright== 1) break;
      }
     
    }
    else if(Benzene_turn_flag_left== 0 && Benzene_turn_flag_right== 1 && Benzene_turn_flag_up==0 && out_cir_left==0 && out_cir_right==0  &&cross_flag==0) //右直角
    {
      
      for(i=Benzene_turn_point_right1-10;i>15;i--)
      {
        for(j=0;j<MT9V03X_W-3;j++)
        {
          if(image_two_value[i][j]==black_point&& image_two_value[i][j+1]==white_point&& image_two_value[i][j+2]==white_point)
          {
              right_stright= 0;                        
          }
          else
          {
              right_stright= 1;
            
            break;
          }
        }
        if(right_stright== 1) break;
      }
    }
  
    if(left_stright== 1)
    {
      Add_Line(1,Benzene_turn_point_left1,mid_line_list[MT9V03X_H-2],MT9V03X_H-2);
      Add_Line(1,0,1,Benzene_turn_point_left1);      
    }
    else if(right_stright==1)
    {
      Add_Line(MT9V03X_W-1,Benzene_turn_point_right1,mid_line_list[MT9V03X_H-2],MT9V03X_H-2);
      Add_Line(MT9V03X_W-1,0,MT9V03X_W-1,Benzene_turn_point_right1);
    }
    
        
}


/*****************************Բ��***********************************************************************************************************************************************************************/
void circular(void)
{
  int i,j;
  if(Benzene_turn_flag_up==2 && Benzene_turn_flag_down==2 && Benzene_turn_flag_left==1 && Benzene_turn_flag_right==0 && cross_flag==0 && out_cir_left==0 && right_cir_flag==0 && abs(Benzene_turn_point_down2-Benzene_turn_point_down1)>40)
  {
    left_cir_flag= 1;
  }
  else if(Benzene_turn_flag_up==2 && Benzene_turn_flag_down==2 && Benzene_turn_flag_left==0 && Benzene_turn_flag_right==1 && cross_flag==0 && out_cir_right==0 && left_cir_flag==0 && abs(Benzene_turn_point_down2-Benzene_turn_point_down1)>40)
  {
    right_cir_flag= 1;
  }
  
  /*****�ж��뻷******/
  if(left_cir_flag==1)
  {
    for(j=mid-5;j>40;j--)
    {
      for(i=MT9V03X_H-1;i>3;i--)
      {
        if(image_two_value[i][j]==black_point&& image_two_value[i+1][j]==white_point&& image_two_value[i+2][j]==white_point)
        {
          left_cir_point= i;
          left_cir_height= MT9V03X_H-left_cir_point;
          break;
        }
      }
    }
    if(abs(left_cir_height-last_left_cir_height)>=8)
    {
      Benzene_height_cont++;
      if(Benzene_height_cont== 2)
      {
        in_cir_left=1;
      }
    }
    last_left_cir_height= left_cir_height;
  }
  else if(right_cir_flag==1)
  {
    for(j=mid+10;j<100;j++)
    {
      for(i=MT9V03X_H-1;i>3;i--)
      {
        if(image_two_value[i][j]==black_point&& image_two_value[i+1][j]==white_point&& image_two_value[i+2][j]==white_point)
        {
          right_cir_point= i;
          right_cir_height= MT9V03X_H-right_cir_point;
          break;
        }
      }
    }
    if(abs(right_cir_height- last_right_cir_height)>=40)
    {
      Benzene_height_cont++;
      if(Benzene_height_cont==2)
      {
        in_cir_right= 1;
      }        
    }
    last_right_cir_height= right_cir_height;
  }
  
  if(in_cir_left==1 && current_yaw<-340)
  {
    current_yaw = 0;
    ResetYawZero();
    out_cir_left=1;
    
  }
  else if(in_cir_right==1 && current_yaw>340)
  {
    current_yaw = 0;
    ResetYawZero();
    out_cir_right=1;
   
  }
  
  /*ʶ��Բ�����Ѳ���߼��������ұ��߸�ֵ�����ߣ��Ա�����?*/
//  for(uint8 i=MT9V03X_H-1;i>40;i--)
//  {
    if(left_cir_flag==1)
    {
      for(uint8 i=MT9V03X_H-1;i>20;i--)
      {
        if(in_cir_left==1)
        {
          mid_line_list[i] = left_line_list[i]+2;
          if(mid_line_list[i]<=0)
          {
            mid_line_list[i] = 1;
          }
        }
        else
        {
          mid_line_list[i] = right_line_list[i]-2;
        }
      }      
    }
    else if(right_cir_flag==1)
    {
      for(uint8 i=MT9V03X_H-1;i>40;i--)
      {
        if(in_cir_right==1)
        {
          mid_line_list[i] = right_line_list[i]+3;
          if(mid_line_list[i]<=0 || mid_line_list[i]>=MT9V03X_W)
          {
            mid_line_list[i]=119;
          }
        }
        else
        {
          mid_line_list[i] = left_line_list[i];
        }
      }      
    }
    
     /*�������Ѳ���߼�?*/
  
    if(out_cir_left==1)
    {
      for(uint8 i=MT9V03X_H-1;i>30;i--)
      {
        mid_line_list[i] = right_line_list[i];
      }
      out_cir_encoder+= (left_encoder+right_encoder)/2;
    }
    else if(out_cir_right==1)
    {
      for(uint8 i=MT9V03X_H-1;i>50;i--)
      {
        mid_line_list[i] = left_line_list[i];
      }
      out_cir_encoder+= (left_encoder+right_encoder)/2;
    }
    
    if(out_cir_encoder>500 && Benzene_turn_flag_down==1)
    {
        left_cir_flag= 0;
        in_cir_left=0;
        out_cir_left=0;
        left_cir_height=0;
        last_left_cir_height=0;
        
        right_cir_flag= 0;
        in_cir_right=0;
        out_cir_right=0;
        right_cir_height=0;
        last_right_cir_height=0;
        Benzene_height_cont=0;
        out_cir_encoder=0;
    }
//  }
  
}


/*****************************************************************************ʮ��ʶ��************************************************************************************************/
void cross(void)
{
  if(left_stright== 0 && right_stright==0)
  {
    if(Benzene_turn_flag_up==1 && Benzene_turn_flag_down==1 && Benzene_turn_flag_left==1 && Benzene_turn_flag_right==1 && Benzene_turn_point_left1>40 && Benzene_turn_point_right1>40)
    {
      cross_flag= 1;
    }
    else
    {
      cross_flag= 0;
    }
  }
  
  if(cross_flag==1)
  {
    Add_Line(Benzene_turn_point_up1,0,mid_line_list[MT9V03X_H-2],MT9V03X_H-2);
  }
  
}


/*************************************************************************��·ʶ��**************************************************************************************************/
void break_rode(void)
{
  uint8 lost_cont=0;
  int i;
  for(i=start_check_row;i>end_check_row;i--)
  {
    if(left_line_list[i]==-1 && right_line_list[i]==-1)
    {
      lost_cont++;//��ⶪ������?
    }
  }
  check_row= start_check_row- end_check_row; //�ܼ����?
  percent_lost= (lost_cont*100/check_row); //���߰ٷֱ�
  
  if(percent_lost > 99 && (left_cir_flag==0 || right_cir_flag==0))
  {
    break_flag= 1;
    replay_mode=1;
  }
}



/***********************************���ߺ���*********************************************************************************************************************************************************/
void Add_Line(uint8 x1,uint8 y1,uint8 x2,uint8 y2)//�����ߺ���
{
    uint8 i,max,a1,a2;
    uint8 hx;
    if(x1>=MT9V03X_W-1)//��ʼ��λ��У�����ų�����Խ��Ŀ���?
       x1=MT9V03X_W-1;
    else if(x1<=0)
        x1=0;
     if(y1>=MT9V03X_H-1)
        y1=MT9V03X_H-1;
     else if(y1<=0)
        y1=0;
     if(x2>=MT9V03X_W-1)
        x2=MT9V03X_W-1;
     else if(x2<=0)
             x2=0;
     if(y2>=MT9V03X_H-1)
        y2=MT9V03X_H-1;
     else if(y2<=0)
             y2=0;
    a1=y1;
    a2=y2;
    if(a1>a2)//���껥��
    {
        max=a1;
        a1=a2;
        a2=max;
    }

    for(i=a1;i<=a2;i++)//����б�ʲ��߼���
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;
        if(hx>=MT9V03X_W)
            hx=MT9V03X_W;
        else if(hx<=0)
            hx=0;
       mid_line_list[i]=hx;
    }
}

void Add_left_Line(uint8 x1,uint8 y1,uint8 x2,uint8 y2)//�����ߺ���
{
    uint8 i,max,a1,a2;
    uint8 hx;
    if(x1>=MT9V03X_W-1)//��ʼ��λ��У�����ų�����Խ��Ŀ���?
       x1=MT9V03X_W-1;
    else if(x1<=0)
        x1=0;
     if(y1>=MT9V03X_H-1)
        y1=MT9V03X_H-1;
     else if(y1<=0)
        y1=0;
     if(x2>=MT9V03X_W-1)
        x2=MT9V03X_W-1;
     else if(x2<=0)
             x2=0;
     if(y2>=MT9V03X_H-1)
        y2=MT9V03X_H-1;
     else if(y2<=0)
             y2=0;
    a1=y1;
    a2=y2;
    if(a1>a2)//���껥��
    {
        max=a1;
        a1=a2;
        a2=max;
    }

    for(i=a1;i<=a2;i++)//����б�ʲ��߼���
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;
        if(hx>=MT9V03X_W)
            hx=MT9V03X_W;
        else if(hx<=0)
            hx=0;
       left_line_list[i]=hx;
    }
}

void Add_right_Line(uint8 x1,uint8 y1,uint8 x2,uint8 y2)//�����ߺ���
{
    uint8 i,max,a1,a2;
    uint8 hx;
    if(x1>=MT9V03X_W-1)//��ʼ��λ��У�����ų�����Խ��Ŀ���?
       x1=MT9V03X_W-1;
    else if(x1<=0)
        x1=0;
     if(y1>=MT9V03X_H-1)
        y1=MT9V03X_H-1;
     else if(y1<=0)
        y1=0;
     if(x2>=MT9V03X_W-1)
        x2=MT9V03X_W-1;
     else if(x2<=0)
             x2=0;
     if(y2>=MT9V03X_H-1)
        y2=MT9V03X_H-1;
     else if(y2<=0)
             y2=0;
    a1=y1;
    a2=y2;
    if(a1>a2)//���껥��
    {
        max=a1;
        a1=a2;
        a2=max;
    }

    for(i=a1;i<=a2;i++)//����б�ʲ��߼���
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;
        if(hx>=MT9V03X_W)
            hx=MT9V03X_W;
        else if(hx<=0)
            hx=0;
       right_line_list[i]=hx;
    }
}

