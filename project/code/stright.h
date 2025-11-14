#ifndef __STRIGHT_H_
#define __STRIGHT_H_

#include "zf_common_headfile.h"

// 直角检测相关参数定义
#define RIGHT_ANGLE_THRESHOLD  80   // 判定为直角的角度阈值(度)：在 90° ± threshold 范围内认为是直角
#define MIN_EDGE_LENGTH       15    // 最小边长要求，防止噪点
#define VERIFY_POINTS         5     // 验证点数量
#define MAX_DEVIATION         2     // 允许的最大偏差(像素)

// 直角检测结构体
typedef struct {
    int corner_x;    // 直角顶点x坐标
    int corner_y;    // 直角顶点y坐标
    int start1_x;    // 第一条边起点x
    int start1_y;    // 第一条边起点y
    int start2_x;    // 第二条边起点x
    int start2_y;    // 第二条边起点y
    float angle;     // 角度
} RightAngle;

float calculate_angle(int x1, int y1, int x2, int y2, int x3, int y3);
bool detect_right_angle(int x, int y, RightAngle* result);
bool check_edge_continuity(int start_x, int start_y, int end_x, int end_y, bool is_horizontal);
void detect_right_angles(void);


#endif



