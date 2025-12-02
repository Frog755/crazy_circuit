#include "key.h"
#include "encoder.h"
#include "pid.h"
#include "ins.h"
#include "motor.h"
#include "image.h"
#include "Gyro.h"

uint16 KEY_NUKM = 0;
uint8 keyOld = 0;
uint8 break_page = BREAK_PAGE;



void KEY_INIT(void)
{
   gpio_init(P20_6, GPI, 1, GPO_PUSH_PULL);
   gpio_init(P20_7, GPI, 1, GPO_PUSH_PULL);
   gpio_init(P11_2, GPI, 1, GPO_PUSH_PULL);
   gpio_init(P11_3, GPI, 1, GPO_PUSH_PULL);
}

uint8_t Key_Get(void)
{
  uint8_t  KEY_VALUE = 0;
  if(gpio_get_level(P20_6) == 0)
  {
      system_delay_ms(40);
      if(gpio_get_level(P20_6) == 0)
      {
        KEY_VALUE = 1;
      }
      
  }
  else if(gpio_get_level(P20_7) == 0)
  {
   system_delay_ms(40);
      if(gpio_get_level(P20_7) == 0)
      {
        KEY_VALUE = 2;
      }
  }
  else if(gpio_get_level(P11_2) == 0)
  {
    system_delay_ms(40);
      if(gpio_get_level(P11_2) == 0)
      {
        KEY_VALUE = 3;
      }
  }
  else if(gpio_get_level(P11_3) == 0)
  {
      system_delay_ms(40);
      if(gpio_get_level(P11_3) == 0)
      {
        KEY_VALUE = 4;
      }
  }
  
  return KEY_VALUE;
}


int16 para_encoder=0;
int16 down_encoder=0;
int select=0;
int last_select=0;
int para_mode=0;
int menu_page= 0;
int write_flash=0;
int read_flash=0;


//锟斤拷锟斤拷锟斤拷锟斤拷
void PID(void)
{
  
  
  ips200_show_string(0,0,"pid_L.Target:");                              ips200_show_float(120,0,pid_L.Target,3,1);
  ips200_show_string(0,20,"pid_R.Target:");                             ips200_show_float(120,20,pid_R.Target,3,1);
  ips200_show_string(0,40,"angle_pid.Kp:");                             ips200_show_float(120,40,angle_pid.Kp,2,1);
  ips200_show_string(0,60,"angle_pid.Kp2:");                             ips200_show_float(120,60,angle_pid.Kp2,2,2);
  ips200_show_string(0,80,"angle_pid.Kd:");                             ips200_show_float(120,80,angle_pid.Kd,2,1);
  ips200_show_string(0,100,"Angular_pid.Kp:");                            ips200_show_float(120,100,Angular_pid.Kp,2,2);
  ips200_show_string(0,120,"Angular_pid.Kd:");                            ips200_show_float(120,120,Angular_pid.Kd,2,2);
  ips200_show_string(0,140,"write_flash");
  ips200_show_string(0,160,"back");

  down_encoder += right_encoder;
  para_encoder += left_encoder;
      
   if(down_encoder>100)
   {
      down_encoder=0;
      select++;
      if(select>8) select=0;
   }
   else if(down_encoder<-100)
   {
      down_encoder=0;
      select--;
      if(select<0) select=8;
   }
   ips200_show_char(180,last_select*20,' ');//锟斤拷锟斤拷洗锟斤拷锟斤拷锟�
   ips200_show_char(180,select*20,'*');//锟斤拷示锟斤拷锟斤拷锟斤拷锟�
   
   last_select=select;
   
   if(para_encoder>50)
   {
     para_encoder=0;
      switch(select)
      {
        case 0: pid_L.Target+=1; break; 
        case 1: pid_R.Target+=1; break;
        case 2: angle_pid.Kp +=0.1; break;
        case 3: angle_pid.Kp2+=0.01; break; 
        case 4: angle_pid.Kd+=0.1;break;
        case 5: Angular_pid.Kp+=0.1;break;
        case 6: Angular_pid.Kd+=0.1;break;
      default: break;
      }
   }
   
   else if(para_encoder<-50)
   {
     para_encoder=0;
     switch(select)
     {
       case 0: pid_L.Target-=1; break;
       case 1: pid_R.Target-=1; break;
       case 2: angle_pid.Kp -=0.1; break;
       case 3: angle_pid.Kp2-=0.01; break; 
       case 4: angle_pid.Kd-=0.1;break;
       case 5: Angular_pid.Kp-=0.1;break; 
       case 6: Angular_pid.Kd-=0.1;break;
       default: break;
     }
   }
   
   uint8_t key = Key_Get();
   if(key == 1)
   {
      
      if(select == 7)
      {
        Write_Flash();
      }
      else if(select == 8)
      {
        menu_page = 0;
        select = 0;     // 杩斿洖涓昏彍鍗曟椂璁剧疆鍒癙ID閫夐」
        last_select = 0;
        down_encoder=0;
        para_encoder=0;
      }
   }  
}

//锟斤拷路锟斤拷锟斤拷
void Navigate(void)
{
  ips200_show_string(0,0,"reply_mode:");          ips200_show_int(120,0,replay_mode,1);
  ips200_show_string(0,20,"record_mode:");        ips200_show_int(120,20,record_mode,1);
  ips200_show_string(0,40,"record_stop:");        ips200_show_int(120,40,record_stop,1);
  ips200_show_string(0,60,"flash_page:");         ips200_show_int(120,60,flash_page,1);
  ips200_show_string(0,80,"current_replay_page:");        ips200_show_int(160,80,current_replay_page,1);
  ips200_show_string(0,100,"targe_yaw:");          ips200_show_float(120,100,target_yaw,3,1);
  ips200_show_string(0,120,"BREAK_PAGE:");          ips200_show_int(120,120,BREAK_PAGE,1);
  ips200_show_string(0,140,"write flash");
  ips200_show_string(0,160,"back");
  
  down_encoder += right_encoder;
  para_encoder += left_encoder;
  
  if(down_encoder>100 )
   {
      down_encoder=0;
      select++;
      if(select>8) select=0;
   }
   else if(down_encoder<-100 )
   {
      down_encoder=0;
      select--;
      if(select<0) select=8;
   }
   ips200_show_char(180,last_select*20,' ');//锟斤拷锟斤拷洗锟斤拷锟斤拷锟�
   ips200_show_char(180,select*20,'*');//锟斤拷示锟斤拷锟斤拷锟斤拷锟�
   
   last_select=select;
   
   if(para_encoder>50 && record_mode==0)
   {
     para_encoder=0;
      if(select==6)
      {
        break_page++;
      }
   }
   else if(para_encoder<-50 && record_mode==0)
   {
      para_encoder=0;
      if(select==6)
      {
        break_page--;
      }
   }
  
//   if(select==7)
//   {
//      if(Key_Get()==4)
//      {
//        menu_page = 0;
//        select = 1;     // 杩斿洖涓昏彍鍗曟椂璁剧疆鍒癷ns閫夐」
//        last_select = 1;
//      }
//   }
   
  if(Key_Get() == 2)
  {
    record_mode =1;
  }
  else if(Key_Get() == 3)
  {
    record_stop = 1;
  }
  
  uint8_t key = Key_Get();
  if(key == 1)
  {
    if(select==7)
    {
      Write_Flash();  // 娣诲姞鍐欏叆flash鍔熻兘
    }
    else if(select==8)
    {
      menu_page = 0;
      select = 1;
      last_select = 1;
      down_encoder=0;
      para_encoder=0;
    }
  }
}

//
void deaprt()
{
  ips200_show_string(0,0,"YES");
  ips200_show_string(0,20,"NO");
  ips200_show_string(0,40,"back");
  
  down_encoder += right_encoder;
  para_encoder += left_encoder;
  
  if(down_encoder>100)
   {
      down_encoder=0;
      select++;
      if(select>2) select=0;
   }
   else if(down_encoder<-100)
   {
      down_encoder=0;
      select--;
      if(select<0) select=2;
   }
   ips200_show_char(180,last_select*20,' ');//锟斤拷锟斤拷洗锟斤拷锟斤拷锟�
   ips200_show_char(180,select*20,'*');//锟斤拷示锟斤拷锟斤拷锟斤拷锟�
   
   last_select=select;
   
   if(Key_Get()==1)
   {
      switch(select)
      {
        case 0:
          system_delay_ms(2000);
          run_flag = 1;
          break;
        case 1: run_flag = 0;break;
        case 2: 
          menu_page = 0;
          select = 3;     // 杩斿洖涓昏彍鍗曟椂璁剧疆鍒癲epart閫夐」
          last_select = 3;
          down_encoder=0;
          para_encoder=0;
          break;
        default: break;
      }
   }
   
   
}

void iamge()
{
//  if(mt9v03x_finish_flag==1)
//  {
    ips200_displayimage03x((const uint8 *)image_two_value,120,100);
    drawkline();
//  }  
  ips200_show_int(0,120,left_stright,1);
  ips200_show_int(20,120,right_stright,1);
  ips200_show_float(60,120,image_error,2,1);
  
  ips200_show_int(0,140,left_cir_flag,1);
  ips200_show_int(20,140,in_cir_left,1);
  ips200_show_int(40,140,out_cir_left,1);
  ips200_show_int(0,160,right_cir_flag,1);
  ips200_show_int(20,160,in_cir_right,1);
  ips200_show_int(40,160,out_cir_right,1);
  ips200_show_int(60,160,current_yaw,3);
  
  ips200_show_int(0,180,cross_flag,1);
  
  ips200_show_string(0,200,"threshold:");     ips200_show_int(120,200,threshold,3);
  ips200_show_string(0,220,"limite_th:");     ips200_show_int(120,220,limite_th,3);
  
  ips200_show_string(0,240,"write_flash");    // 娣诲姞鍐欏叆flash閫夐」
  ips200_show_string(0,260,"back");
  
  down_encoder += right_encoder;
  para_encoder += left_encoder;
  
  if(down_encoder>100)
  {
    down_encoder=0;
    select++;
    if(select>13) select=0;  // 澧炲姞閫夐」鏁伴噺
  }
  else if(down_encoder<-100)
  {
    down_encoder=0;
    select--;
    if(select<0) select=13;  // 澧炲姞閫夐」鏁伴噺
  }
  ips200_show_char(180,last_select*20,' ');
  ips200_show_char(180,select*20,'*');
  
  last_select=select;
  
  if(para_encoder>50)
  {
    para_encoder=0;
    if(select==10)
    {
      threshold+=5;
    }
    else if(select==11)
    {
      limite_th+=5;
    }
  }
  else if(para_encoder<-50)
  {
    para_encoder=0;
    if(select==10)
    {
      threshold-=5;
    }
    else if(select==11)
    {
      limite_th-=5;
    }
  }
  
  uint8_t key = Key_Get();
  if(key == 1)
  {
    if(select==12)  // write_flash閫夐」
    {
      Write_Flash();
    }
    else if(select==13)  // back閫夐」
    {
      menu_page = 0;
      select = 2;     // 杩斿洖涓昏彍鍗曟椂璁剧疆鍒癷mage閫夐」
      last_select = 2;
      down_encoder=0;
      para_encoder=0;
    }
  }
}

void menu()
{
  static uint8_t last_menu_page = 0;  // 璁板綍涓婁竴娆＄殑鑿滃崟椤甸潰
  
  // 鍙湪鑿滃崟椤甸潰鍒囨崲鏃舵竻灞�
  if(last_menu_page != menu_page)
  {
    ips200_clear();
    last_menu_page = menu_page;
  }
  
  if(menu_page == 0)
  {
    ips200_show_string(0,0,"pid");
    ips200_show_string(0,20,"ins");
    ips200_show_string(0,40,"image");
    ips200_show_string(0,60,"depart");
    
    down_encoder += right_encoder;
    para_encoder += left_encoder;
    
    if(down_encoder>100)
    {
      down_encoder=0;
      select++;
      if(select>3) select=0;
    }
    else if(down_encoder<-100)
    {
      down_encoder=0;
      select--;
      if(select<0) select=3;
    }
    ips200_show_char(180,last_select*20,' ');//娓呴櫎涓婃绱㈠紩
    ips200_show_char(180,select*20,'*');//鏄剧ず杩欐绱㈠紩
    
    last_select=select;
    
    uint8_t key = Key_Get();
    if(key == 1)  // 鎸夐敭1杩涘叆瀛愯彍鍗�
    {
      switch(select)
      {
        case 0: 
          menu_page = 1;
          select = 0;    // 閲嶇疆閫夋嫨绱㈠紩
          last_select = 0;
          down_encoder=0;
          para_encoder=0;
          break;
        case 1: 
          menu_page = 2;
          select = 0;
          last_select = 0;
          down_encoder=0;
          para_encoder=0;
          break;
        case 2: 
          menu_page = 3;
          select = 0;
          last_select = 0;
          down_encoder=0;
          para_encoder=0;
          break;
        case 3: 
          menu_page = 4;
          select = 0;
          last_select = 0;
          down_encoder=0;
          para_encoder=0;
          break;
        default: break;
      }
    }
  }
  else
  {
    switch(menu_page)
    {
      case 1: PID(); break;
      case 2: Navigate(); break;
      case 3: iamge(); break;
      case 4: deaprt(); break;
      default: break;
    }
  }
}


void Write_Flash(void)
{
  flash_buffer_clear();
  // PID鍙傛暟
  flash_union_buffer[0].float_type = pid_L.Target;
  flash_union_buffer[1].float_type = pid_R.Target;
  flash_union_buffer[2].float_type = angle_pid.Kp;
  flash_union_buffer[3].float_type = angle_pid.Kp2;
  flash_union_buffer[4].float_type = angle_pid.Kd;
  flash_union_buffer[5].float_type = Angular_pid.Kp;
  flash_union_buffer[6].float_type = Angular_pid.Kd;
  
  // 鍥惧儚澶勭悊鍙傛暟
  flash_union_buffer[7].int32_type = threshold;
  flash_union_buffer[8].int32_type = limite_th;
  
  // 瀵艰埅鍙傛暟
  flash_union_buffer[9].int32_type = BREAK_PAGE;
  
  flash_write_page_from_buffer(0, write_flash);
}

void Read_flash(void)
{
  flash_read_page_to_buffer(0, read_flash);
  
  // PID鍙傛暟
  pid_L.Target = flash_union_buffer[0].float_type;
  pid_R.Target = flash_union_buffer[1].float_type;
  angle_pid.Kp = flash_union_buffer[2].float_type;
  angle_pid.Kp2 = flash_union_buffer[3].float_type;
  angle_pid.Kd = flash_union_buffer[4].float_type;
  Angular_pid.Kp = flash_union_buffer[5].float_type;
  Angular_pid.Kd = flash_union_buffer[6].float_type;
  
  // 鍥惧儚澶勭悊鍙傛暟
  threshold = flash_union_buffer[7].int32_type;
  limite_th = flash_union_buffer[8].int32_type;
  
  // 瀵艰埅鍙傛暟
  break_page = flash_union_buffer[9].int32_type;
}



















