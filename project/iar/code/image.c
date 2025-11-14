void stright_angle(void)
{
  int i,j;
  int vertical_black_count = 0;  // 垂直黑点计数
  int horizontal_black_count = 0; // 水平黑点计数
  left_stright=0;
  right_stright=0;
  
  // 左直角判断
  if(Benzene_turn_flag_left==1 && Benzene_turn_flag_right==0 && Benzene_turn_flag_up==0 
     && in_cir_right==0 && in_cir_left==0 && cross_flag==0)
  {
    // 检查垂直边界的连续性
    for(i=Benzene_turn_point_left1; i>Benzene_turn_point_left1-20 && i>5; i--)
    {
      vertical_black_count = 0;
      for(j=0; j<15; j++) // 检查每一行左侧的黑点数
      {
        if(image_two_value[i][j]==black_point)
        {
          vertical_black_count++;
        }
      }
      // 要求垂直边界有足够的黑点
      if(vertical_black_count >= 8)
      {
        left_stright = 1;
      }
      else
      {
        left_stright = 0;
        break;
      }
    }
    
    // 如果垂直边界满足条件，再检查水平边界
    if(left_stright == 1)
    {
      horizontal_black_count = 0;
      for(j=0; j<MT9V03X_W/2; j++)
      {
        if(image_two_value[Benzene_turn_point_left1][j]==black_point)
        {
          horizontal_black_count++;
        }
      }
      // 要求水平边界也有足够的黑点
      if(horizontal_black_count < 5)
      {
        left_stright = 0;
      }
    }
  }
  
  // 右直角判断
  if(Benzene_turn_flag_left==0 && Benzene_turn_flag_right==1 && Benzene_turn_flag_up==0 
     && in_cir_right==0 && in_cir_left==0 && cross_flag==0)
  {
    // 检查垂直边界的连续性
    for(i=Benzene_turn_point_right1; i>Benzene_turn_point_right1-20 && i>5; i--)
    {
      vertical_black_count = 0;
      for(j=MT9V03X_W-15; j<MT9V03X_W; j++) // 检查每一行右侧的黑点数
      {
        if(image_two_value[i][j]==black_point)
        {
          vertical_black_count++;
        }
      }
      // 要求垂直边界有足够的黑点
      if(vertical_black_count >= 8)
      {
        right_stright = 1;
      }
      else
      {
        right_stright = 0;
        break;
      }
    }
    
    // 如果垂直边界满足条件，再检查水平边界
    if(right_stright == 1)
    {
      horizontal_black_count = 0;
      for(j=MT9V03X_W/2; j<MT9V03X_W; j++)
      {
        if(image_two_value[Benzene_turn_point_right1][j]==black_point)
        {
          horizontal_black_count++;
        }
      }
      // 要求水平边界也有足够的黑点
      if(horizontal_black_count < 5)
      {
        right_stright = 0;
      }
    }
  }
    
  // 根据直角标志位进行中线补偿
  if(left_stright == 1)
  {
    Add_Line(2, Benzene_turn_point_left1, mid_line_list[MT9V03X_H-2], MT9V03X_H-2);
    Add_Line(2, 0, 2, Benzene_turn_point_left1);
  }
  else if(right_stright == 1)
  {
    Add_Line(MT9V03X_W-1, Benzene_turn_point_right1, mid_line_list[MT9V03X_H-2], MT9V03X_H-2);
    Add_Line(MT9V03X_W-1, 0, MT9V03X_W-1, Benzene_turn_point_right1);
  }
} 