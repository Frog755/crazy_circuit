#ifndef FUZZY_PID_H
#define FUZZY_PID_H

#include <stdint.h>

/* 模糊集合定义 */
typedef enum {
    NB = 0,  // 负大
    NM = 1,  // 负中
    NS = 2,  // 负小
    ZO = 3,  // 零
    PS = 4,  // 正小
    PM = 5,  // 正中
    PB = 6   // 正大
} FuzzyLevel;

/* 模糊PID控制器结构体 */
typedef struct {
    // 系统状态
    float position_error;     // 位置偏差
    float position_error_rate; // 位置偏差变化率
    float speed_error;        // 速度偏差
    float speed_error_rate;   // 速度偏差变化率
    
    // 控制参数
    float set_speed;          // 预设速度
    float kp;                 // 比例系数
    float ki;                 // 积分系数
    float pwm_output;         // PWM输出值
    
    // PID历史数据
    float last_position_error; // 上一次位置偏差
    float last_speed_error;    // 上一次速度偏差
    float last_pwm_output;     // 上一次PWM输出
    
    // 模糊控制表
    const float (*speed_rule)[7];  // 预设速度模糊规则表
    const float (*kp_rule)[7];     // Kp模糊规则表
    const float (*ki_rule)[7];     // Ki模糊规则表
    
    // 模糊化参数
    float pos_error_range[2];      // 位置偏差范围 [min, max]
    float pos_error_rate_range[2]; // 位置偏差变化率范围
    float speed_error_range[2];    // 速度偏差范围
    float speed_error_rate_range[2]; // 速度偏差变化率范围
} FuzzyPIDController;

/* 函数声明 */
// 初始化模糊PID控制器
void fuzzy_pid_init(FuzzyPIDController *controller);

// 更新控制器状态并计算输出
void fuzzy_pid_update(FuzzyPIDController *controller, float current_position, float current_speed);

// 获取当前PWM输出
float fuzzy_pid_get_output(FuzzyPIDController *controller);

// 设置目标位置
void fuzzy_pid_set_target_position(FuzzyPIDController *controller, float target_position);

#endif /* FUZZY_PID_H */