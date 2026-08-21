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
	
	/* NaN / Inf 输入保护（任何一项 NaN 则 0；Inf 则钳到 ±1000） */
    if (isnan(kp) || isinf(kp))  kp = 0.0f;
    if (isnan(ki) || isinf(ki))  ki = 0.0f;
    if (isnan(kd) || isinf(kd))  kd = 0.0f;
    if (isnan(out_max))  out_max =  1000.0f;
    if (isnan(out_min))  out_min = -1000.0f;
    if (isnan(integral_max) || isinf(integral_max)) integral_max = 1000.0f;

    /* kp/ki/kd 钳制到 ±1000；量产版建议根据实际 PWM 范围再调窄（如 ±800） */
    if (kp >  1000.0f)  kp =  1000.0f;
    if (kp < -1000.0f)  kp = -1000.0f;
    if (ki >  1000.0f)  ki =  1000.0f;
    if (ki < -1000.0f)  ki = -1000.0f;
    if (kd >  1000.0f)  kd =  1000.0f;
    if (kd < -1000.0f)  kd = -1000.0f;

    /* 输出限幅最小值必须 ≤ 最大值，否则交换并告警 */
    if (out_min > out_max)
    {
        float tmp = out_min;
        out_min = out_max;
        out_max = tmp;
        printf("WARN PID_Init: out_min>out_max swapped!\r\n");
    }

    /* M20 (FR-39): 积分限幅必为正 */
    if (integral_max < 0.0f) integral_max = -integral_max;
	
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
	
	if(dt <= 0.0f)
	{
		return pid->output;
	}
	
	/* dt 过大上限钳制（dt=60s 的"假采样"会导致 integral+=error*60 直接把积分打满、微分项爆炸） */
    if (dt > 10.0f)  dt = 10.0f;

    /* 测量/设定值 NaN/Inf → 不更新，返回历史旧输出（避免 PID 输出饱和） */
    if (isnan(setpoint) || isinf(setpoint))  return pid->output;
    if (isnan(measurement) || isinf(measurement))  return pid->output;
	
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
    pid->integral += pid->filtered_error * dt;
    
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
    
    // 计算输出
    pid->output = proportional + integral + derivative;
    
	float output_unclamped = pid->output;
	
    // 输出限幅
    if(pid->output > pid->out_max)
	{
        pid->output = pid->out_max;
    }
	else if(pid->output < pid->out_min)
	{
        pid->output = pid->out_min;
    }
	
	// 反向计算抗饱和：当发生饱和时，将积分修正为使输出刚好到达限幅边界的值
	if (pid->output != output_unclamped)
	{
		// 目标：proportional + ki * new_integral + derivative = 限幅值
		// => new_integral = (限幅值 - proportional - derivative) / ki
		if (pid->ki != 0.0f)
		{
			pid->integral = (pid->output - proportional - derivative) / pid->ki;
		}
	}
	
	pid->prev_error = pid->filtered_error;
    
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
	
	/* 增量式同样加 dt 上限 + 数值保护 */
    if (isnan(setpoint) || isinf(setpoint)) return pid->output;
    if (isnan(measurement) || isinf(measurement)) return pid->output;
    if (dt <= 0.0f)    return pid->output;
    if (dt > 10.0f)    dt = 10.0f;
	
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
  * 注  意：PID_Reset 忽略 pid->enabled 位 → 即使 disabled(enabled=0) 停机状态下也允许清积分，
  *         避免"停电机 → 积分残留在 enabled=0 前饱和 → 重新启动瞬间跳变"的典型抗饱和 bug。 *
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
	/* 假读 enabled，避免 Lint/PCLint "成员未使用"告警 */
    (void)pid->enabled;
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
	
	/* NaN / Inf 输入保护（任何一项 NaN 则 0；Inf 则钳到 ±1000） */
    if (isnan(kp) || isinf(kp))  kp = 0.0f;
    if (isnan(ki) || isinf(ki))  ki = 0.0f;
    if (isnan(kd) || isinf(kd))  kd = 0.0f;

    /* kp/ki/kd 钳制到 ±1000；量产版建议根据实际 PWM 范围再调窄（如 ±800） */
    if (kp >  1000.0f)  kp =  1000.0f;
    if (kp < -1000.0f)  kp = -1000.0f;
    if (ki >  1000.0f)  ki =  1000.0f;
    if (ki < -1000.0f)  ki = -1000.0f;
    if (kd >  1000.0f)  kd =  1000.0f;
    if (kd < -1000.0f)  kd = -1000.0f;
	
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
    
    float ku = 0.0f;         /* 临界增益 */
    float tu = 0.0f;         /* 振荡周期(秒) */
    float max_output = pid->out_max;
    float test_output = 0.0f;
    float measurement = 0.0f;
    uint8_t oscillating = 0;
    uint32_t oscillation_start = 0;
    float max_amplitude = 0.0f;
    uint8_t peak_count = 0;  /* 峰值计数（非static，每次调用重置） */
    float last_measurement = 0.0f;
    uint8_t last_sign = 0;   /* 上次测量值方向: 1-上升, 0-下降 */
    
    /* 步骤1：逐渐增加输出，寻找等幅振荡点 */
    for (uint16_t i = 0; i < cycles; i++)
    {
        /* 线性增加测试输出 */
        test_output = (max_output * (float)(i + 1)) / (float)cycles;
        Motor_SetTargetSpeed((int16_t)test_output, MOTOR_FORWARD);
        
        measurement = measurement_func();
        
        /* 跳过前几次采样的不稳定阶段 */
        if (i > 10)
        {
            float amplitude = fabs(measurement - last_measurement);
            
            if (amplitude > max_amplitude)
            {
                max_amplitude = amplitude;
            }
            
            /* 检测过零点（符号变化）来识别振荡周期 */
            uint8_t current_sign = (measurement >= setpoint) ? 1 : 0;
            
            if (current_sign != last_sign && last_sign != 0)
            {
                peak_count++;
            }
            last_sign = current_sign;
            
            /* 检测到明显的振荡（振幅超过设定值的10%） */
            if ((amplitude > setpoint * 0.1f) && (!oscillating))
            {
                oscillating = 1;
                oscillation_start = HAL_GetTick();  /* 记录振荡起始时间(ms) */
                ku = test_output / max_amplitude;   /* 估算临界增益 */
                peak_count = 0;
                printf("Oscillation detected at output=%.1f, Ku=%.3f\r\n", test_output, ku);
            }
            
            /* 检测至少2个过零点（1个完整振荡周期） */
            if (oscillating && (peak_count >= 2))
            {
                /* 使用HAL_GetTick计算经过的时间(ms) */
                uint32_t elapsed_ms = HAL_GetTick() - oscillation_start;
                tu = (float)elapsed_ms / 1000.0f;  /* 转换为秒 */
                printf("Oscillation period: %.3f s\r\n", tu);
                
                /* 使用齐格勒-尼科尔斯方法计算新参数 */
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
        
        last_measurement = measurement;
        Delay_ms((uint32_t)(dt * 1000.0f));
    }
    
    /* 自整定失败，恢复停止状态 */
    Motor_SetTargetSpeed(0, MOTOR_STOP);
    printf("Auto-tuning failed. Using default parameters.\r\n");
}
