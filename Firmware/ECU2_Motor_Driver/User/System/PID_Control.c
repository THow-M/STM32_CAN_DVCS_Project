#include "stm32f10x.h"                  // Device header
#include "PID_Control.h"
#include "Serial.h"

/** 函  数：PID控制器初始化
  * 参  数：pid PID控制器结构体
  *	参  数：kp 比例项常数，ki 积分项常数，kd 微分项常数
  *	参  数：out_max 输出上限，out_min 输出下限
  *	参  数：intergal_max 积分限幅
  * 返回值：无
  */
void PID_Init(PID_Controller* pid, float kp, float ki, float kd, 
              float out_max, float out_min, float integral_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_error2 = 0.0f;
    pid->output = 0.0f;
    pid->out_max = out_max;
    pid->out_min = out_min;
    pid->integral_max = integral_max;
    pid->dead_zone = 0.0f;
    pid->filter_coeff = 0.0f;
    pid->filtered_error = 0.0f;
    pid->enabled = 1;
    
    printf("PID initialized: Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n", kp, ki, kd);
}
