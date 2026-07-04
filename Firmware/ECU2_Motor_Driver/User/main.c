#include "stm32f10x.h"                  // Device header
#include "System_Config.h"
#include "ecu2_main.h"
#include "OLED.h"
#include "Delay.h"
#include "LED.h"
#include "Timer.h"
#include "MyCAN.h"
#include "Serial.h"
#include "PWM.h"
#include "PID_Control.h"
#include "Encoder.h"
#include "Motor.h"
#include <stdio.h>

// 全局变量
System_State system_state = SYS_IDLE;
uint32_t system_uptime = 0;
uint8_t error_code = ERROR_NONE;
uint8_t can_connected = 0;

// 控制变量
PID_Controller speed_pid;
float target_speed_rpm = 0.0f;
float actual_speed_rpm = 0.0f;
float pid_output = 0.0f;
uint8_t motor_direction = MOTOR_STOP;
uint8_t control_mode = CONTROL_MODE_MANUAL;

// 定时器
uint32_t last_pid_time = 0;
uint32_t last_can_send_time = 0;
uint32_t last_heartbeat_time = 0;
uint32_t last_status_time = 0;
uint32_t last_protection_check = 0;
uint32_t last_debug_time = 0;

// CAN接收数据
SpeedCmd_Data speed_cmd = {0};
SystemCtrl_Data system_ctrl = {0};
uint8_t heartbeat_status[NODE_NUM] = {0};
uint32_t heartbeat_time[NODE_NUM] = {0};

void System_Init(void)
{
	Serial_Init(DEBUG_BAUDRATE);
	
	printf("\r\n\r\n");
    printf("================================\r\n");
    printf("  ECU2 - Motor Drive Unit\r\n");
    printf("  Version: 1.0.0\r\n");
    printf("  Build Date: %s %s\r\n", __DATE__, __TIME__);
    printf("================================\r\n\r\n");
	
    LED_Init();
    
    MyCAN_Init(CAN_BAUDRATE);
    
    Encoder_Init();
    
    // 初始化电机
    // PWM频率 = 72MHz / (71+1) / (999+1) = 1kHz
    Motor_Init(999, 71);
    
    // 初始化PID控制器
    // 参数需要根据实际电机调整
    PID_Init(&speed_pid, 0.8f, 0.05f, 0.01f, 1000.0f, -1000.0f, 1000.0f);
    speed_pid.dead_zone = 5.0f;  // 5RPM死区
    speed_pid.filter_coeff = 0.7f;  // 低通滤波系数
    
    // 系统启动指示
    printf("System initializing...\r\n");
	
	LED_ON();
	Delay_ms(200);
	LED_OFF();
	Delay_ms(200);
	LED_ON();
	Delay_ms(200);
	LED_OFF();
	Delay_ms(200);
	LED_ON();
	Delay_ms(200);
	LED_OFF();
	Delay_ms(200);
	
	// 编码器校准
    //printf("Calibrating encoder...\r\n");
    //Encoder_Calibrate();
    
    // 启动系统
    system_state = SYS_READY;
    system_uptime = 0;
    
    printf("System initialized successfully!\r\n");
    printf("Control Mode: %s\r\n", 
           control_mode == CONTROL_MODE_MANUAL ? "Manual" : "Auto");
    printf("Waiting for commands...\r\n");
    
	//启动成功指示
    LED_ON();
}

int main(void) 
{
	LED_ON();
	
	
	
	while (1)
	{
		

	}
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		Tick_ms++;
		
		
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
