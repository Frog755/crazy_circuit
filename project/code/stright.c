
/*最终未采用 */


#include "stright.h"
#include "math.h"
#include "image.h"

// �������������ļн�(��)
float calculate_angle(int x1, int y1, int x2, int y2, int x3, int y3)
{
    // ������������
    float vector1_x = x1 - x2;
    float vector1_y = y1 - y2;
    float vector2_x = x3 - x2;
    float vector2_y = y3 - y2;
    
    // ������
    float dot_product = vector1_x * vector2_x + vector1_y * vector2_y;
    
    // ������������
    float length1 = sqrt(vector1_x * vector1_x + vector1_y * vector1_y);
    float length2 = sqrt(vector2_x * vector2_x + vector2_y * vector2_y);
    
    // ����нǣ����ȣ�
    float angle_rad = acos(dot_product / (length1 * length2));
    
    // ת��Ϊ�Ƕ�
    return angle_rad * 180.0f / 3.14159f;
}

// ����Ե��������
bool check_edge_continuity(int start_x, int start_y, int end_x, int end_y, bool is_horizontal)
{
    int valid_points = 0;
    int total_points = VERIFY_POINTS;
    
    if (is_horizontal) {
        int step = (end_x - start_x) / (total_points + 1);
        int expected_y = start_y;
        
        for (int x = start_x + step; x < end_x; x += step) {
            // ��xλ������MAX_DEVIATION�����ط�Χ�ڲ��ұ�Ե��
            for (int dy = -MAX_DEVIATION; dy <= MAX_DEVIATION; dy++) {
                int check_y = expected_y + dy;
                if (check_y < 0 || check_y >= MT9V03X_H) continue;
                
                if (image_two_value[check_y][x] == black_point && 
                    x + 1 < MT9V03X_W && image_two_value[check_y][x+1] == white_point) {
                    valid_points++;
                    break;
                }
            }
        }
    } else {
        int step = (end_y - start_y) / (total_points + 1);
        int expected_x = start_x;
        
        for (int y = start_y + step; y < end_y; y += step) {
            // ��yλ������MAX_DEVIATION�����ط�Χ�ڲ��ұ�Ե��
            for (int dx = -MAX_DEVIATION; dx <= MAX_DEVIATION; dx++) {
                int check_x = expected_x + dx;
                if (check_x < 0 || check_x >= MT9V03X_W) continue;
                
                if (image_two_value[y][check_x] == black_point && 
                    check_x + 1 < MT9V03X_W && image_two_value[y][check_x+1] == white_point) {
                    valid_points++;
                    break;
                }
            }
        }
    }
    
    return valid_points >= (total_points * 3 / 4); // ����75%�ĵ�Ҫ����Ҫ��
}


// ���ֱ��
bool detect_right_angle(int x, int y, RightAngle* result)
{
    if (!result || x <= 0 || y <= 0 || x >= MT9V03X_W-1 || y >= MT9V03X_H-1) return false;
    
    // ����ȷ�ϵ�ǰ���Ƿ��Ǳ�Ե��
    if (!(image_two_value[y][x] == black_point && image_two_value[y][x+1] == white_point)) {
        return false;
    }
    
    // �ڵ�ǰ����Χ�������ܵı�Ե��
    int horizontal_edge_x = -1;
    int vertical_edge_y = -1;
    
    // ��������ˮƽ��Ե
    for (int search_x = x - MIN_EDGE_LENGTH; search_x < x; search_x++) {
        if (search_x < 0) continue;
        if (image_two_value[y][search_x] == black_point && 
            search_x + 1 < MT9V03X_W && image_two_value[y][search_x+1] == white_point) {
            horizontal_edge_x = search_x;
            break;
        }
    }
    
    // ����������ֱ��Ե
    for (int search_y = y - MIN_EDGE_LENGTH; search_y < y; search_y++) {
        if (search_y < 0) continue;
        if (image_two_value[search_y][x] == black_point && 
            x + 1 < MT9V03X_W && image_two_value[search_y][x+1] == white_point) {
            vertical_edge_y = search_y;
            break;
        }
    }
    
    // ����ҵ���������
    if (horizontal_edge_x != -1 && vertical_edge_y != -1) {
        // ���ߵ�������
        if (!check_edge_continuity(horizontal_edge_x, y, x, y, true) ||
            !check_edge_continuity(x, vertical_edge_y, x, y, false)) {
            return false;
        }
        
        // ����Ƕ�
        float angle = calculate_angle(horizontal_edge_x, y, x, y, x, vertical_edge_y);
        
        // �ж��Ƿ�ӽ�90��
        if (fabs(angle - 90.0f) <= RIGHT_ANGLE_THRESHOLD) {
            result->corner_x = x;
            result->corner_y = y;
            result->start1_x = horizontal_edge_x;
            result->start1_y = y;
            result->start2_x = x;
            result->start2_y = vertical_edge_y;
            result->angle = angle;
            return true;
        }
    }
    
    return false;
}

// ֱ�Ǽ��������
void detect_right_angles(void)
{
    static RightAngle angles[4];  // ���洢4��ֱ��
    int angle_count = 0;
    
    // ���֮ǰ�ı�־
    left_stright = 0;
    right_stright = 0;
    
    // ��ͼ���м�������ɨ��
    for (int y = MT9V03X_H/4; y < MT9V03X_H*3/4 && angle_count < 4; y += 2) {
        for (int x = MT9V03X_W/4; x < MT9V03X_W*3/4 && angle_count < 4; x += 2) {
            RightAngle temp;
            if (detect_right_angle(x, y, &temp)) {
                // ����Ƿ����Ѽ�⵽��ֱ�Ǿ���̫��
                bool too_close = false;
                for (int i = 0; i < angle_count; i++) {
                    int dx = temp.corner_x - angles[i].corner_x;
                    int dy = temp.corner_y - angles[i].corner_y;
                    if (dx*dx + dy*dy < MIN_EDGE_LENGTH*MIN_EDGE_LENGTH) {
                        too_close = true;
                        break;
                    }
                }
                
                if (!too_close) {
                    angles[angle_count++] = temp;
                    
                    // �ж�����ֱ�ǻ�����ֱ��
                    if (temp.corner_x < MT9V03X_W/2) {
                        left_stright = 1;
                        Add_Line(temp.start1_x, temp.start1_y, temp.corner_x, temp.corner_y);
                    } else {
                        right_stright = 1;
                        Add_Line(temp.corner_x, temp.corner_y, temp.start2_x, temp.start2_y);
                    }
                    
                    // ���ӻ�ֱ�ǣ������Ҫ��
//                    Add_Line(temp.start1_x, temp.start1_y, temp.corner_x, temp.corner_y);
//                    Add_Line(temp.corner_x, temp.corner_y, temp.start2_x, temp.start2_y);
                }
            }
        }
    }
}




