#include "stm32f10x.h"                  // Device header
#include "PWM.h"
#include "Delay.h"
#include "Serial.h"
#include <stdlib.h>

// PWM定时器配置
#define PWM_TIM                TIM1
#define PWM_TIM_CLK           RCC_APB2Periph_TIM1
#define PWM_GPIO_CLK          RCC_APB2Periph_GPIOA
#define PWM_GPIO              GPIOA
#define PWM_PIN_CH1           GPIO_Pin_8   // PA8 - TIM1_CH1
#define PWM_PIN_CH2           GPIO_Pin_9   // PA9 - TIM1_CH2
#define PWM_PIN_CH3           GPIO_Pin_10  // PA10 - TIM1_CH3
#define PWM_PIN_CH4           GPIO_Pin_11  // PA11 - TIM1_CH4

// 电机控制结构体
Motor_Control motor_control = {0};
volatile uint16_t pwm_period = 1000;  // PWM周期值

/** 函  数：电机初始化
  * 参  数：arr 定时器自动重装载值，psc 定时器预分频器
  * 返回值：无
  */
void Motor_Init(uint16_t arr, uint16_t psc)
{
    // 1. 开启时钟
    RCC_APB2PeriphClockCmd(PWM_TIM_CLK | PWM_GPIO_CLK, ENABLE);
    
    // 2. 配置GPIO为复用推挽输出
	GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = PWM_PIN_CH1 | PWM_PIN_CH2 | PWM_PIN_CH3 | PWM_PIN_CH4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PWM_GPIO, &GPIO_InitStructure);
    
    // 3. 定时器基础配置
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    pwm_period = arr;
    TIM_TimeBaseStructure.TIM_Period = arr;        // 自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler = psc;     // 预分频器
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(PWM_TIM, &TIM_TimeBaseStructure);
    
    // 4. PWM模式配置
	TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                  // PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 0;                                 // CCR的值，初始占空比0
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;          // 高极性
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    
    // 配置4个通道
    TIM_OC1Init(PWM_TIM, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);
    
    TIM_OC2Init(PWM_TIM, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);
    
    TIM_OC3Init(PWM_TIM, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);
    
    TIM_OC4Init(PWM_TIM, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);
    
    // 5. 使能预装载寄存器
    TIM_ARRPreloadConfig(PWM_TIM, ENABLE);
    
    // 6. 使能定时器
    TIM_CtrlPWMOutputs(PWM_TIM, ENABLE);
    TIM_Cmd(PWM_TIM, ENABLE);
    
    // 7. 初始化电机控制结构
    motor_control.current_speed = 0;
    motor_control.target_speed = 0;
    motor_control.direction = MOTOR_STOP;
    motor_control.state = MOTOR_STATE_IDLE;
    motor_control.protection.over_current = 0;
    motor_control.protection.over_temp = 0;
    motor_control.protection.stall = 0;
    motor_control.protection.over_voltage = 0;
    motor_control.protection.under_voltage = 0;
    
    printf("Motor initialized. PWM Freq: %ld Hz\r\n", 
           SystemCoreClock / ((psc + 1) * (arr + 1)));
}

/** 函  数：设置电机速度和方向
  * 参  数：speed: 0-1000，direction: 0=停止，1=正转，2=反转
  * 返回值：无
  */
void Motor_SetSpeed(int16_t speed, uint8_t direction)
{
    uint16_t pwm_value = 0;
    
    // 限幅处理
    if(speed > 1000) speed = 1000;
    if(speed < 0) speed = 0;
    
    // 更新电机控制结构
    motor_control.target_speed = speed;
    motor_control.direction = direction;
    
    switch(direction)
	{
        case MOTOR_STOP:
            // 电机停止
            TIM_SetCompare1(PWM_TIM, 0);
            TIM_SetCompare2(PWM_TIM, 0);
            TIM_SetCompare3(PWM_TIM, 0);
            TIM_SetCompare4(PWM_TIM, 0);
            motor_control.state = MOTOR_STATE_STOP;
            break;
            
        case MOTOR_FORWARD:
            // 正转：CH1输出PWM，CH2低电平
            pwm_value = (speed * pwm_period) / 1000;
            TIM_SetCompare1(PWM_TIM, pwm_value);
            TIM_SetCompare2(PWM_TIM, 0);
            TIM_SetCompare3(PWM_TIM, 0);
            TIM_SetCompare4(PWM_TIM, 0);
            motor_control.state = MOTOR_STATE_RUN;
            break;
            
        case MOTOR_REVERSE:
            // 反转：CH2输出PWM，CH1低电平
            pwm_value = (speed * pwm_period) / 1000;
            TIM_SetCompare1(PWM_TIM, 0);
            TIM_SetCompare2(PWM_TIM, pwm_value);
            TIM_SetCompare3(PWM_TIM, 0);
            TIM_SetCompare4(PWM_TIM, 0);
            motor_control.state = MOTOR_STATE_RUN;
            break;
            
        case MOTOR_BRAKE:
            // 刹车：两个引脚都输出高电平
            TIM_SetCompare1(PWM_TIM, pwm_period);
            TIM_SetCompare2(PWM_TIM, pwm_period);
            TIM_SetCompare3(PWM_TIM, 0);
            TIM_SetCompare4(PWM_TIM, 0);
            motor_control.state = MOTOR_STATE_BRAKE;
            break;
    }
    
    // 更新实际速度（开环时等于目标速度）
    motor_control.current_speed = speed;
}

/** 函  数：平滑加速
  * 参  数：target_speed: 0-1000，direction: 0=停止，1=正转，2=反转，acc_time_ms：加速时间
  * 返回值：无
  */
void Motor_SmoothAcceleration(int16_t target_speed, uint8_t direction, uint16_t acc_time_ms)
{
    int16_t current_speed = motor_control.current_speed;
    int16_t step = 0;
    uint16_t delay_time = 0;
    
    if(acc_time_ms == 0)
	{
        Motor_SetSpeed(target_speed, direction);
        return;
    }
    
    // 计算步进和延时
    if(target_speed > current_speed)
	{
        step = 1;   // 加速
    }
	else
	{
        step = -1;  // 减速
    }
    
    delay_time = acc_time_ms / abs(target_speed - current_speed);
    
    // 平滑调整速度
    while(current_speed != target_speed)
	{
        current_speed += step;
        Motor_SetSpeed(current_speed, direction);
        Delay_ms(delay_time);
    }
}

/** 函  数：电机保护检测
  * 参  数：无
  * 返回值：error_flags：错误标志位，详见错误代码定义
  */
uint8_t Motor_Protection_Check(void)
{
    uint8_t error_flags = 0;
    
    // 检测过流（这里需要连接电流检测电路）
    // 通过ADC读取电流值
    /*uint16_t current = Motor_GetCurrent();  // 需要实现此函数
    if(current > MOTOR_MAX_CURRENT)
	{
        motor_control.protection.over_current = 1;
        error_flags |= ERROR_OVER_CURRENT;
    } else {
        motor_control.protection.over_current = 0;
    }*/
    
    // 检测温度（需要温度传感器）
    /*uint8_t temperature = Motor_GetTemperature();  // 需要实现此函数
    if(temperature > MOTOR_MAX_TEMP)
	{
        motor_control.protection.over_temp = 1;
        error_flags |= ERROR_OVER_TEMP;
    }
	else
	{
        motor_control.protection.over_temp = 0;
    }*/
    
    // 检测堵转
    static uint32_t last_speed = 0;
    static uint32_t stall_counter = 0;
    
    if(motor_control.target_speed > 100 && motor_control.current_speed < 10)
	{
        stall_counter++;
        if(stall_counter > 50)
		{  // 连续50次检测到堵转
            motor_control.protection.stall = 1;
            error_flags |= ERROR_STALL;
        }
    }
	else
	{
        stall_counter = 0;
        motor_control.protection.stall = 0;
    }
    
    last_speed = motor_control.current_speed;
    
    return error_flags;
}

// 错误处理
void Motor_Error_Handler(uint8_t error_code)
{
    // 立即停止电机
    Motor_SetSpeed(0, MOTOR_STOP);
    motor_control.state = MOTOR_STATE_ERROR;
    
    // 根据错误代码进行相应处理
    switch(error_code)
	{
        case ERROR_OVER_CURRENT:
            printf("Motor Error: Over Current!\r\n");
            break;
        case ERROR_OVER_TEMP:
            printf("Motor Error: Over Temperature!\r\n");
            break;
        case ERROR_STALL:
            printf("Motor Error: Stall!\r\n");
            break;
        case ERROR_OVER_VOLTAGE:
            printf("Motor Error: Over Voltage!\r\n");
            break;
        case ERROR_UNDER_VOLTAGE:
            printf("Motor Error: Under Voltage!\r\n");
            break;
    }
    
    // 记录错误
    motor_control.error_code = error_code;
    motor_control.error_count++;
	
    // 可以添加错误恢复机制
    // 例如：延时后尝试重启
    static uint32_t error_time = 0;
    if(error_time == 0)
	{
        error_time = HAL_GetTick();
    }
    
    if(HAL_GetTick() - error_time > 5000)
	{  // 5秒后尝试恢复
        printf("Attempting to recover from error...\r\n");
        motor_control.state = MOTOR_STATE_IDLE;
        error_time = 0;
    }
}
