#include "stm32f10x.h"                  // Device header
#include "PID_Control.h"
#include "Serial.h"
#include <math.h>

/** 函  数：PID控制器初始化
  * 参  数：pid PID控制器结构体
  *	参  数：kp 比例项常数，ki 积分项常数，kd 微分项常数
  *	参  数：out_max 输出上限，out_min 输出下限
  *	参  数：intergal_max 积分限幅
  * 返回值：pid->output 输出值
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

/** 函  数：位置式PID计算
  * 参  数：pid PID控制器结构体
  *	参  数：setpoint 目标值
  *	参  数：measurement 测量值
  *	参  数：dt 
  * 返回值：pid->output 
  */
float PID_Calculate(PID_Controller* pid, float setpoint, float measurement, float dt)
{
    if(!pid->enabled)
	{
        return 0.0f;
    }
    
    float error = setpoint - measurement;
    float proportional, integral, derivative;
    
    // 死区处理
    if(fabs(error) < pid->dead_zone)
	{
        error = 0.0f;
    }
    
    // 低通滤波
    pid->filtered_error = pid->filter_coeff * pid->filtered_error + (1.0f - pid->filter_coeff) * error;
    
    // 比例项
    proportional = pid->kp * pid->filtered_error;
    
    // 积分项（带抗饱和）
    pid->integral += error * dt;
    
    // 积分限幅
    if(pid->integral > pid->integral_max)
	{
        pid->integral = pid->integral_max;
    }
	else if(pid->integral < -pid->integral_max)
	{
        pid->integral = -pid->integral_max;
    }
    
    // 积分分离（当误差较大时，减小积分作用）
    float integral_gain = pid->ki;
    if(fabs(error) > pid->integral_max * 0.5f)
	{
        integral_gain *= 0.5f;  // 减小积分增益
    }
    
    integral = integral_gain * pid->integral;
    
    // 微分项（不完全微分）
    derivative = pid->kd * (pid->filtered_error - pid->prev_error) / dt;
    pid->prev_error = pid->filtered_error;
    
    // 计算输出
    pid->output = proportional + integral + derivative;
    
    // 输出限幅
    if(pid->output > pid->out_max)
	{
        pid->output = pid->out_max;
    }
	else if(pid->output < pid->out_min)
	{
        pid->output = pid->out_min;
    }
    
    return pid->output;
}

/** 函  数：增量式PID计算
  * 参  数：pid PID控制器结构体
  *	参  数：setpoint 目标值
  *	参  数：measurement 测量值
  *	参  数：dt
  * 返回值：pid->output 输出值
  */
float PID_Calculate_Incremental(PID_Controller* pid, float setpoint, float measurement, float dt)
{
    if(!pid->enabled)
	{
        return 0.0f;
    }
    
    float error = setpoint - measurement;
    float delta_output;
    
    // 死区处理
    if(fabs(error) < pid->dead_zone)
	{
        error = 0.0f;
    }
    
    // 低通滤波
    pid->filtered_error = pid->filter_coeff * pid->filtered_error + (1.0f - pid->filter_coeff) * error;
    
    // 计算增量
    delta_output = pid->kp * (pid->filtered_error - pid->prev_error) +
                  pid->ki * pid->filtered_error * dt +
                  pid->kd * (pid->filtered_error - 2 * pid->prev_error + pid->prev_error2) / dt;
    
    // 更新历史误差
    pid->prev_error2 = pid->prev_error;
    pid->prev_error = pid->filtered_error;
    
    // 累加输出
    pid->output += delta_output;
    
    // 输出限幅
    if(pid->output > pid->out_max)
	{
        pid->output = pid->out_max;
    }
	else if(pid->output < pid->out_min)
	{
        pid->output = pid->out_min;
    }
    
    return pid->output;
}

/** 函  数：重置PID控制器
  * 参  数：pid PID控制器结构体
  * 返回值：无
  */
void PID_Reset(PID_Controller* pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_error2 = 0.0f;
    pid->output = 0.0f;
    pid->filtered_error = 0.0f;
}

/** 函  数：设置PID参数
  * 参  数：pid PID控制器结构体
  * 参  数：kp 比例项常数，ki 积分项常数，kd 微分项常数
  * 返回值：无
  */
void PID_SetParameters(PID_Controller* pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
