#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H

// PID控制器结构
typedef struct
{
    float kp;               // 比例系数
    float ki;               // 积分系数
    float kd;               // 微分系数
    
    float integral;         // 积分项
    float prev_error;       // 上次误差
    float prev_error2;      // 上上次误差
    float output;           // 输出
    
    float out_max;          // 输出上限
    float out_min;          // 输出下限
    float integral_max;     // 积分限幅
    
    float dead_zone;        // 死区
    float filter_coeff;     // 滤波器系数
    float filtered_error;   // 滤波后的误差
    
    uint8_t enabled;        // 使能标志
} PID_Controller;

// 函数声明
void PID_Init(PID_Controller* pid, float kp, float ki, float kd, 
              float out_max, float out_min, float integral_max);
float PID_Calculate(PID_Controller* pid, float setpoint, float measurement, float dt);
float PID_Calculate_Incremental(PID_Controller* pid, float setpoint, float measurement, float dt);
void PID_Reset(PID_Controller* pid);
void PID_SetParameters(PID_Controller* pid, float kp, float ki, float kd);
void PID_GetParameters(PID_Controller* pid, float* kp, float* ki, float* kd);
void PID_AutoTune(PID_Controller* pid, float setpoint, float (*measurement_func)(void), 
                  float dt, uint16_t cycles);
void PID_Diagnostic(PID_Controller* pid);

#endif
