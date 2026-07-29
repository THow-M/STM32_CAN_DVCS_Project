#include "stm32f10x.h"                  // Device header
#include "System_Config.h"
#include "ecu2_main.h"
#include "MyCAN.h"
#include "Motor.h"
#include "PWM.h"
#include "Encoder.h"
#include "PID_Control.h"
#include "Delay.h"
#include "Serial.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*
 * 硬件连接：
 *   PA8  -> PWM输出（TIM1_CH1）
 *   PB0  -> H桥 IN1（方向控制）
 *   PB1  -> H桥 IN2（方向控制）
 *
 * 方向逻辑：
 *   IN1=1, IN2=0 -> 正转
 *   IN1=0, IN2=1 -> 反转
 *   IN1=1, IN2=1 -> 刹车
 *   IN1=0, IN2=0 -> 停止（滑行）
 */
 
#define DIR_PORT        GPIOB
#define DIR_PIN_IN1     GPIO_Pin_0
#define DIR_PIN_IN2     GPIO_Pin_1
#define DIR_GPIO_CLK    RCC_APB2Periph_GPIOB

Motor_Control motor_control = {0};


/**函  数：电机初始化
  * 参  数：arr 定时器自动重装载值，psc 定时器预分频器
  * 返回值：无
  */
void Motor_Init(uint16_t arr, uint16_t psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 初始化PWM
    PWM_Init(arr, psc);

    // 初始化方向GPIO
    RCC_APB2PeriphClockCmd(DIR_GPIO_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Pin = DIR_PIN_IN1 | DIR_PIN_IN2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DIR_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(DIR_PORT, DIR_PIN_IN1 | DIR_PIN_IN2);

    // 初始化PID
    PID_Init(&speed_pid, 0.8f, 0.05f, 0.01f, 1000.0f, -1000.0f, 1000.0f);
    speed_pid.dead_zone = 5.0f;
    speed_pid.filter_coeff = 0.7f;

    // 初始化控制结构
    motor_control.current_speed = 0;
    motor_control.target_speed = 0;
    motor_control.direction = MOTOR_STOP;
    motor_control.state = MOTOR_STATE_IDLE;
    motor_control.protection.over_current = 0;
    motor_control.protection.over_temp = 0;
    motor_control.protection.stall = 0;
    motor_control.protection.over_speed = 0;
    motor_control.protection.under_voltage = 0;
    motor_control.error_code = 0;
    motor_control.error_count = 0;

    printf("Motor driver initialized.\n");
}

/** 函  数：底层设置速度和方向（内部调用）
  * 参  数：speed 要设置的速度，direction 要设置的方向
  * 返回值：无
  */
void Motor_SetSpeed(int16_t speed, uint8_t direction)
{
    if (speed > 1000) speed = 1000;
    if (speed < 0) speed = 0;
    motor_control.current_speed = speed;
    motor_control.direction = direction;
    switch (direction)
	{
        case MOTOR_STOP:
            GPIO_ResetBits(DIR_PORT, DIR_PIN_IN1 | DIR_PIN_IN2);
            PWM_SetCompare1(0);
            motor_control.state = MOTOR_STATE_STOP;
            break;
        case MOTOR_FORWARD:
            GPIO_SetBits(DIR_PORT, DIR_PIN_IN1);
            GPIO_ResetBits(DIR_PORT, DIR_PIN_IN2);
            PWM_SetCompare1(speed);
            motor_control.state = MOTOR_STATE_RUN;
            break;
        case MOTOR_REVERSE:
            GPIO_ResetBits(DIR_PORT, DIR_PIN_IN1);
            GPIO_SetBits(DIR_PORT, DIR_PIN_IN2);
            PWM_SetCompare1(speed);
            motor_control.state = MOTOR_STATE_RUN;
            break;
        case MOTOR_BRAKE:
            GPIO_SetBits(DIR_PORT, DIR_PIN_IN1 | DIR_PIN_IN2);
            PWM_SetCompare1(0);
            motor_control.state = MOTOR_STATE_BRAKE;
            break;
    }
}

/** 函  数：电机模式设置
  * 参  数：mode 电机模式
  * 返回值：无
  */
void Motor_SetControlMode(uint8_t mode)
{
    if (mode != CONTROL_MODE_MANUAL && mode != CONTROL_MODE_AUTO)
		return;
    control_mode = mode;
    if (mode == CONTROL_MODE_MANUAL)
	{
        speed_pid.enabled = 0;
        PID_Reset(&speed_pid);
    }
	else
	{
        speed_pid.enabled = 1;
    }
	
    printf("Control mode: %s\n", mode == CONTROL_MODE_MANUAL ? "Manual" : "Auto");
}

/** 函  数：设置电机目标速度
  * 参  数：speed_rpm 目标速度，direction 目标方向
  * 返回值：无
  */
void Motor_SetTargetSpeed(float speed_rpm, uint8_t direction)
{
    if (speed_rpm > MOTOR_MAX_SPEED)
		speed_rpm = MOTOR_MAX_SPEED;
    if (speed_rpm < 0)
		speed_rpm = 0;

    motor_control.target_speed = (int16_t)speed_rpm;
    motor_control.direction = direction;

    if (control_mode == CONTROL_MODE_MANUAL)
	{
        int16_t pwm = (int16_t)(speed_rpm * 1000.0f / MOTOR_MAX_SPEED);
        if (pwm > 1000)
			pwm = 1000;
        if (pwm < 0)
			pwm = 0;
        Motor_SetSpeed(pwm, direction);  // 直接调用底层
        if (speed_rpm > 0)
			motor_control.state = MOTOR_STATE_RUN;
        else motor_control.state = MOTOR_STATE_STOP;
    }
	else
	{
        // 自动模式，PID控制
        /* 仅更新目标值，实际 PWM 由 Motor_RunPIDControl() 计算 */
		if (speed_rpm > 0.0f)
		{
			motor_control.state = MOTOR_STATE_RUN;
		} else
		{
			motor_control.state = MOTOR_STATE_STOP;
		}
    }
	
    printf("Target speed: %.1f RPM, Dir: %d\n", speed_rpm, direction);
}

/** 函  数：电机紧急停止
  * 参  数：无
  * 返回值：无
  */
void Motor_EmergencyStop(void)
{
    // 刹车
    GPIO_SetBits(DIR_PORT, DIR_PIN_IN1 | DIR_PIN_IN2);
    PWM_SetCompare1(0);
    Delay_ms(100);
	
	// 释放刹车，进入滑行停止状态
    GPIO_ResetBits(DIR_PORT, DIR_PIN_IN1 | DIR_PIN_IN2);
    
    PID_Reset(&speed_pid);
    motor_control.target_speed = 0;
    motor_control.current_speed = 0;
    motor_control.state = MOTOR_STATE_STOP;
    printf("Emergency stop.\n");
}

/** 函  数：电机初始化
  * 参  数：运行PID控制
  * 返回值：无
  */
void Motor_RunPIDControl(void)
{
    if (control_mode != CONTROL_MODE_AUTO) return;
    if (motor_control.state != MOTOR_STATE_RUN) return;
    if (!speed_pid.enabled) return;

    static uint32_t last_time = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_time < 10) return;
    last_time = now;

    Encoder_Data enc = Encoder_GetData();
    motor_control.current_speed = (int16_t)enc.speed_rpm;

    float corrected_target = motor_control.target_speed;
    if (motor_control.direction == MOTOR_REVERSE)
		corrected_target = -corrected_target;

    float output = PID_Calculate(&speed_pid,
                              fabsf(corrected_target),
                              fabsf(enc.speed_rpm),
                              0.01f);
	int16_t pwm = (int16_t)fabsf(output);
	if (pwm > 1000) pwm = 1000;
	if (pwm < 0) pwm = 0;
	
	// 方向始终跟随目标方向，不随 PID 输出符号改变
	uint8_t dir = motor_control.direction;
	Motor_SetSpeed(pwm, dir);

    static uint32_t debug = 0;
    if (now - debug > 500)
	{
        debug = now;
        printf("PID: T=%.1f A=%.1f O=%d I=%.1f\n",
               corrected_target, enc.speed_rpm, pwm, speed_pid.integral);
    }
}

/** 函  数：电机保护检测
  * 参  数：无
  * 返回值：错误类型
  */
uint8_t Motor_ProtectionCheck(void)
{
    uint8_t errors = 0;
    uint16_t current = Motor_GetCurrent();
    if (current > MOTOR_MAX_CURRENT)
	{
		motor_control.protection.over_current = 1;
        errors |= ERROR_OVER_CURRENT;
        motor_control.error_code = ERROR_OVER_CURRENT;
    }
	else
	{
		motor_control.protection.over_current = 0;
	}
	
    // 堵转
    static uint32_t stall_start = 0;
	float actual_speed = Motor_GetSpeed();
    if (motor_control.target_speed > 100 && fabs(actual_speed) < 10)
	{
		motor_control.protection.stall = 1;
        if (stall_start == 0)
			stall_start = HAL_GetTick();
        else if (HAL_GetTick() - stall_start > 2000)
		{
            errors |= ERROR_STALL;
            motor_control.error_code = ERROR_STALL;
        }
    }
	else
	{
        stall_start = 0;
		motor_control.protection.stall = 0;
    }
	
    // 过速
    if (abs(motor_control.current_speed) > MOTOR_MAX_SPEED * 1.1f)
	{
        errors |= ERROR_OVER_SPEED;
        motor_control.error_code = ERROR_OVER_SPEED;
    }
	else
	{
		motor_control.protection.over_speed = 0;
	}
	
    if (errors)
	{
        motor_control.error_count++;
        motor_control.state = MOTOR_STATE_ERROR;
        if (motor_control.error_count > 5) motor_control.state = MOTOR_STATE_LOCKED;
    }
	else
	{
		if (motor_control.error_count > 0)
		{
			motor_control.error_count--;
		}
		if (motor_control.error_count == 0)
		{
			motor_control.error_code = 0;
			if (motor_control.target_speed > 0)
			{
				motor_control.state = MOTOR_STATE_RUN;
			}
			else
			{
				motor_control.state = MOTOR_STATE_IDLE;
			}
		}
	}
    return errors;
}

/** 函  数：电机错误处理
  * 参  数：error_code 错误代码
  * 返回值：无
  */
void Motor_ErrorHandler(uint8_t error_code)
{
    printf("Motor error: 0x%02X\n", error_code);
    switch (error_code)
	{
        case ERROR_OVER_CURRENT:
            Motor_EmergencyStop();
            break;
        case ERROR_OVER_TEMP:
            // 缓慢减速
            for (int16_t i = 1000; i >= 0; i -= 100)
			{
                Motor_SetSpeed(i, motor_control.direction);
                Delay_ms(100);
            }
            Motor_SetSpeed(0, MOTOR_STOP);
            break;
        case ERROR_STALL:
            Motor_EmergencyStop();
			motor_control.state = MOTOR_STATE_ERROR;
			printf("Motor stall detected. Manual reset required.\r\n");
			// 不自动恢复，等待 CAN 命令复位
            break;
        default:
            Motor_EmergencyStop();
            break;
    }
    MyCAN_Send_ErrorReport(NODE_ID, error_code, motor_control.error_count);
}

/** 函  数：获取电机状态
  * 参  数：无
  * 返回值：电机状态结构体
  */
Motor_Status Motor_GetStatus(void)
{
    Motor_Status st;
    st.speed = motor_control.current_speed;
    st.target_speed = motor_control.target_speed;
    st.direction = motor_control.direction;
    st.state = motor_control.state;
    st.error_code = motor_control.error_code;
    st.protection_status = 0;
    if (motor_control.protection.over_current) st.protection_status |= 0x01;
    if (motor_control.protection.over_temp) st.protection_status |= 0x02;
    if (motor_control.protection.stall) st.protection_status |= 0x04;
    if (motor_control.protection.over_speed) st.protection_status |= 0x08;
    if (motor_control.protection.under_voltage) st.protection_status |= 0x10;
    return st;
}

/** 函  数：获取电机速度
  * 参  数：无
  * 返回值：电机速度
  */
float Motor_GetSpeed(void)
{
    Encoder_Data enc = Encoder_GetData();
    return enc.speed_rpm;
}

/** 函  数：自动调参
  * 参  数：无
  * 返回值：无
  */
void Motor_AutoTune(void)
{
    printf("Auto-tuning started...\n");
    motor_control.state = MOTOR_STATE_CALIBRATING;

    // Step 1: 测试最大正向速度
    Motor_SetSpeed(1000, MOTOR_FORWARD);
    Delay_ms(2000);
    Encoder_Data enc = Encoder_GetData();
    float max_fwd = enc.speed_rpm;
    printf("Max forward speed: %.1f RPM\n", max_fwd);
    Motor_SetSpeed(0, MOTOR_STOP);
    Delay_ms(1000);

    // Step 2: 测试最大反向速度
    Motor_SetSpeed(1000, MOTOR_REVERSE);
    Delay_ms(2000);
    enc = Encoder_GetData();
    float max_rev = -enc.speed_rpm;
    printf("Max reverse speed: %.1f RPM\n", max_rev);
    Motor_SetSpeed(0, MOTOR_STOP);
    Delay_ms(1000);

    // Step 3: PID自整定
    float test_speed = (max_fwd + max_rev) / 2.0f;
    if (test_speed > 100)
	{
        PID_AutoTune(&speed_pid, test_speed, Motor_GetSpeed, 0.1f, 20);
    }
	else
	{
        printf("Speed too low for auto-tune.\n");
    }
    motor_control.state = MOTOR_STATE_IDLE;
    printf("Auto-tuning done.\n");
}

/** 函  数：电机诊断报告
  * 参  数：arr 定时器自动重装载值，psc 定时器预分频器
  * 返回值：无
  */
void Motor_Diagnostic(void)
{
    printf("=== Motor Driver Diagnostic ===\n");
    Motor_Status st = Motor_GetStatus();
    printf("Control mode: %s\n", control_mode == CONTROL_MODE_MANUAL ? "Manual" : "Auto");
    printf("State: %d\n", st.state);
    printf("Target speed: %d RPM\n", st.target_speed);
    printf("Actual speed: %d RPM\n", st.speed);
    printf("Direction: %d\n", st.direction);
    printf("Error code: 0x%02X\n", st.error_code);
    printf("Fault count: %d\n", motor_control.error_count);
    printf("PID integral: %.2f\n", speed_pid.integral);
    printf("PID output: %.2f\n", speed_pid.output);
    printf("Current: %dmA\n", Motor_GetCurrent());
    printf("==============================\n");
}

/** 函  数：获取电机电流
  * 参  数：无
  * 返回值：fake 电机电流
  * 注  意：此函数还未实现，返回电流为伪造值
  */
uint16_t Motor_GetCurrent(void)
{
    static uint16_t fake = 0;
    if (motor_control.state == MOTOR_STATE_RUN)
        fake = motor_control.current_speed * 10;
    else
        fake = 0;
    return fake;
}
