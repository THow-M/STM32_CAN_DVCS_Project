#include "stm32f10x.h"                  // Device header
#include "PID_Control.h"
#include "Serial.h"
#include "Motor.h"
#include "Delay.h"
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
	if (pid == NULL)
    {
        return;
    }
	
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
	if (pid == NULL)
    {
        return 0.0f;
    }
	
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
	if (pid == NULL)
    {
        return 0.0f;
    }
	
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
	if (pid == NULL)
    {
        return;
    }
	
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
	if (pid == NULL)
    {
        return;
    }
	
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/** 函  数：获取PID参数
  * 参  数：pid PID控制器结构体
  * 参  数：*kp 获取比例项常数的指针，*ki 获取积分项常数的指针，*kd 获取微分项常数的指针
  * 返回值：无
  */
void PID_GetParameters(PID_Controller* pid, float* kp, float* ki, float* kd)
{
	if (pid == NULL || kp == NULL || ki == NULL || kd == NULL)
    {
        return;
    }
	
    *kp = pid->kp;
    *ki = pid->ki;
    *kd = pid->kd;
}

/**
  * 函  数：PID自整定（简易版 - 齐格勒-尼科尔斯方法）
  * 参  数：pid               PID控制器结构体指针
  * 参  数：setpoint          目标设定值
  * 参  数：measurement_func  测量值回调函数指针
  * 参  数：dt                采样周期(秒)
  * 参  数：cycles            自整定最大循环次数
  * 返回值：无
  * 注  释：使用继电器反馈法（类似Z-N法）寻找临界增益和振荡周期
  *         注意：此函数会阻塞CPU执行，建议仅在调试阶段使用
  */
void PID_AutoTune(PID_Controller* pid, float setpoint, float (*measurement_func)(void), 
                  float dt, uint16_t cycles)
{
	if (pid == NULL || measurement_func == NULL)
    {
        return;
    }
	
    printf("Starting PID auto-tuning...\r\n");
    
    float ku, tu;  // 临界增益和周期
    float max_output = pid->out_max;
    float min_output = pid->out_min;
    float test_output = 0.0f;
    float measurement = 0.0f;
    uint8_t oscillating = 0;
    float last_measurement = 0.0f;
    uint32_t oscillation_start = 0;
    float max_amplitude = 0.0f;
    uint32_t half_period_count = 0;
    
    // 步骤1：逐渐增加输出直到系统振荡
    for(uint16_t i = 0; i < cycles; i++)
	{
        test_output = (max_output * (i + 1)) / cycles;
        Motor_SetTargetSpeed((int16_t)test_output, MOTOR_FORWARD);
        
        measurement = measurement_func();
        
        // 检测振荡
        if(i > 10)
		{
            float amplitude = fabs(measurement - last_measurement);
            if(amplitude > max_amplitude)
			{
                max_amplitude = amplitude;
            }
            
            // 检测到明显的振荡
            if(amplitude > setpoint * 0.1f && !oscillating)
			{
                oscillating = 1;
                oscillation_start = HAL_GetTick();
                ku = test_output / amplitude;  // 估算临界增益
                printf("Oscillation detected at output=%.1f\r\n", test_output);
            }
            
            // 测量振荡周期
            if(oscillating)
			{
                float last_peak = 0.0f;
                static uint8_t peak_count = 0;
                
                if(measurement > last_measurement && measurement > setpoint)
				{
                    if(peak_count == 0)
					{
                        last_peak = measurement;
                        peak_count = 1;
                    }
					else
					{
                        tu = (HAL_GetTick() - oscillation_start) / 1000.0f;  // 周期(秒)
                        printf("Oscillation period: %.3f s\r\n", tu);
                        
                        // 使用齐格勒-尼科尔斯方法
                        float kp_new = 0.6f * ku;
                        float ki_new = 2.0f * kp_new / tu;
                        float kd_new = kp_new * tu / 8.0f;
                        
                        PID_SetParameters(pid, kp_new, ki_new, kd_new);
                        Motor_SetTargetSpeed(0, MOTOR_STOP);
                        
                        printf("Auto-tuning completed.\r\n");
                        printf("New parameters: Kp=%.3f, Ki=%.3f, Kd=%.3f\r\n", 
                               kp_new, ki_new, kd_new);
                        return;
                    }
                }
            }
        }
        
        last_measurement = measurement;
        Delay_ms((uint32_t)(dt * 1000));
    }
    
    Motor_SetTargetSpeed(0, MOTOR_STOP);
    printf("Auto-tuning failed. Using default parameters.\r\n");
}

// PID诊断
void PID_Diagnostic(PID_Controller* pid)
{
    printf("=== PID Diagnostic ===\r\n");
    printf("Parameters: Kp=%.3f, Ki=%.3f, Kd=%.3f\r\n", 
           pid->kp, pid->ki, pid->kd);
    printf("Integral: %.3f\r\n", pid->integral);
    printf("Output: %.3f\r\n", pid->output);
    printf("Enabled: %s\r\n", pid->enabled ? "Yes" : "No");
    printf("Dead Zone: %.3f\r\n", pid->dead_zone);
    printf("Integral Max: %.3f\r\n", pid->integral_max);
    printf("Output Range: [%.1f, %.1f]\r\n", pid->out_min, pid->out_max);
    printf("=====================\r\n");
}
