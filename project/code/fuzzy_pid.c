
/*最终未用*/
#include "fuzzy_pid.h"
#include <math.h>
#include <stdlib.h>

/* ģ�����ƹ�������� */
static const float speed_rule_table[7][7] = {
    /*          NB      NM      NS      ZO      PS      PM      PB    */
    /* NB */ { 10.0,   10.0,   10.0,   10.0,   20.0,   40.0,   20.0 },
    /* NM */ { 10.0,   20.0,   10.0,   10.0,   40.0,   40.0,   20.0 },
    /* NS */ { 10.0,   20.0,   40.0,   40.0,   30.0,   40.0,   20.0 },
    /* ZO */ { 10.0,   20.0,   30.0,   30.0,   30.0,   20.0,   10.0 },
    /* PS */ { 20.0,   40.0,   30.0,   40.0,   40.0,   20.0,   10.0 },
    /* PM */ { 20.0,   40.0,   40.0,   10.0,   10.0,   20.0,   10.0 },
    /* PB */ { 20.0,   40.0,   20.0,   10.0,   10.0,   10.0,   10.0 }
};

//static const float kp_rule_table[7][7] = {
   
//};

//static const float ki_rule_table[7][7] = {
    
//
//�����������Ⱥ��� 
static float triangular_membership(float x, float a, float b, float c) {
    if (x <= a || x >= c) {
        return 0.0;
    } else if (x >= a && x <= b) {
        return (x - a) / (b - a);
    } else {
        return (c - x) / (c - b);
    }
}

/* ��ȡ����ֵ��ģ�������� */
static void get_fuzzy_membership(float x, float min, float max, float membership[7]) {
    float range = max - min;
    float step = range / 6.0;
    
    // ����ÿ��ģ�����ϵ�������
    membership[NB] = triangular_membership(x, min - step, min, min + step);
    membership[NM] = triangular_membership(x, min, min + step, min + 2 * step);
    membership[NS] = triangular_membership(x, min + step, min + 2 * step, min + 3 * step);
    membership[ZO] = triangular_membership(x, min + 2 * step, min + 3 * step, min + 4 * step);
    membership[PS] = triangular_membership(x, min + 3 * step, min + 4 * step, min + 5 * step);
    membership[PM] = triangular_membership(x, min + 4 * step, min + 5 * step, max);
    membership[PB] = triangular_membership(x, min + 5 * step, max, max + step);
    
    // ��һ������
    float sum = 0.0;
    for (int i = 0; i < 7; i++) {
        sum += membership[i];
    }
    
    if (sum > 0) {
        for (int i = 0; i < 7; i++) {
            membership[i] /= sum;
        }
    }
}

/* ģ����������������͹����������� */
static float fuzzy_inference(const float input1_membership[7], const float input2_membership[7], 
                             const float rule_table[7][7]) {
    float output = 0.0;
    float weight_sum = 0.0;
    
    // �������й���
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            // ȡ��С��������Ϊ����ǿ��
            float rule_strength = (input1_membership[i] < input2_membership[j]) ? 
                                  input1_membership[i] : input2_membership[j];
            
            // ��Ȩ���
            output += rule_strength * rule_table[i][j];
            weight_sum += rule_strength;
        }
    }
    
    // ��һ�����
    return (weight_sum > 0) ? output / weight_sum : 0.0;
}

/* ����ʽPID���� */
static float incremental_pid_calculate(float error, float last_error, float kp, float ki, float last_output) {
    // ����ʽPID��ʽ: ��u(k) = Kp[e(k) - e(k-1)] + Ki*e(k)
    float delta_output = kp * (error - last_error) + ki * error;
    
    // �����µ����
    float new_output = last_output + delta_output;
    
    // �޷�����
    if (new_output > 100.0) new_output = 100.0;
    if (new_output < 0.0) new_output = 0.0;
    
    return new_output;
}

/* ��ʼ��ģ��PID������ */
void fuzzy_pid_init(FuzzyPIDController *controller) {
    // ��ʼ������������
    controller->position_error = 0.0;
    controller->position_error_rate = 0.0;
    controller->speed_error = 0.0;
    controller->speed_error_rate = 0.0;
    
    controller->set_speed = 0.0;
    controller->kp = 0.5;  // Ĭ��ֵ������ģ����������
    controller->ki = 0.1;  // Ĭ��ֵ������ģ����������
    controller->pwm_output = 0.0;
    
    controller->last_position_error = 0.0;
    controller->last_speed_error = 0.0;
    controller->last_pwm_output = 0.0;
    
    // ����ģ�������
    controller->speed_rule = speed_rule_table;
   // controller->kp_rule = kp_rule_table;
   // controller->ki_rule = ki_rule_table;
    
    // ����ģ������Χ
    controller->pos_error_range[0] = -10.0;
    controller->pos_error_range[1] = 10.0;
    controller->pos_error_rate_range[0] = -5.0;
    controller->pos_error_rate_range[1] = 5.0;
    controller->speed_error_range[0] = -20.0;
    controller->speed_error_range[1] = 20.0;
    controller->speed_error_rate_range[0] = -10.0;
    controller->speed_error_rate_range[1] = 10.0;
}

/* ���¿�����״̬��������� */
void fuzzy_pid_update(FuzzyPIDController *controller, float current_position, float current_speed) {
    float target_position = 0.0;  // ����Ŀ��λ��Ϊ0�����У�
    
    // ����λ��ƫ��
    controller->last_position_error = controller->position_error;
    controller->position_error = target_position - current_position;
    controller->position_error_rate = controller->position_error - controller->last_position_error;
    
    // ģ��������
    float pos_error_membership[7];
    float pos_error_rate_membership[7];
    
    get_fuzzy_membership(controller->position_error, 
                         controller->pos_error_range[0], 
                         controller->pos_error_range[1], 
                         pos_error_membership);
                         
    get_fuzzy_membership(controller->position_error_rate, 
                         controller->pos_error_rate_range[0], 
                         controller->pos_error_rate_range[1], 
                         pos_error_rate_membership);
    
    // ģ����������Ԥ���ٶ�
    controller->set_speed = fuzzy_inference(pos_error_membership, 
                                           pos_error_rate_membership, 
                                           controller->speed_rule);
    
    // �����ٶ�ƫ��
    controller->last_speed_error = controller->speed_error;
    controller->speed_error = controller->set_speed - current_speed;
    controller->speed_error_rate = controller->speed_error - controller->last_speed_error;
    
    // ģ�����ٶ�ƫ���ƫ��仯��
    float speed_error_membership[7];
    float speed_error_rate_membership[7];
    
    get_fuzzy_membership(controller->speed_error, 
                         controller->speed_error_range[0], 
                         controller->speed_error_range[1], 
                         speed_error_membership);
                         
    get_fuzzy_membership(controller->speed_error_rate, 
                         controller->speed_error_rate_range[0], 
                         controller->speed_error_rate_range[1], 
                         speed_error_rate_membership);
    
    // ģ����������PID����
    controller->kp = fuzzy_inference(speed_error_membership, 
                                    speed_error_rate_membership, 
                                    controller->kp_rule);
                                    
    controller->ki = fuzzy_inference(speed_error_membership, 
                                    speed_error_rate_membership, 
                                    controller->ki_rule);
    
    // ����ʽPID����
    controller->last_pwm_output = controller->pwm_output;
    controller->pwm_output = incremental_pid_calculate(controller->speed_error, 
                                                       controller->last_speed_error, 
                                                       controller->kp, 
                                                       controller->ki, 
                                                       controller->last_pwm_output);
}

/* ��ȡ��ǰPWM��� */
float fuzzy_pid_get_output(FuzzyPIDController *controller) {
    return controller->pwm_output;
}

/* ����Ŀ��λ�� */
void fuzzy_pid_set_target_position(FuzzyPIDController *controller, float target_position) {
    controller->position_error = target_position;
}