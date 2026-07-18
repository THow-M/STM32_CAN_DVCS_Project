#include "stm32f10x.h"                  // Device header
#include "System_Config.h"
#include "ecu3_main.h"
#include "Timer.h"
#include "Delay.h"
#include "Key.h"
#include "LED.h"
#include "Serial.h"
#include "MyCAN.h"
#include "MPU6050.h"
#include "Ultrasonic.h"
#include "Voltage.h"
#include <stdio.h>
#include <math.h>

// 全局变量
System_State system_state = SYS_IDLE;
uint32_t system_uptime = 0;
uint8_t error_code = ERROR_NONE;
uint8_t can_connected = 0;

// 传感器数据
Sensor_Fusion sensor_fusion = {0};
uint8_t sensor_ready = 0;
uint8_t calibration_complete = 0;

// 定时器
uint32_t last_sensor_update = 0;
uint32_t last_can_send = 0;
uint32_t last_heartbeat = 0;
uint32_t last_status_update = 0;
uint32_t last_diagnostic = 0;
uint32_t last_calibration_check = 0;

// CAN接收数据
SystemCtrl_Data system_ctrl = {0};
uint8_t heartbeat_status[NODE_NUM] = {0};
uint32_t heartbeat_time[NODE_NUM] = {0};

// 系统初始化
void System_Init(void)
{
	// 初始化定时器2
	Timer_Init();
	
    // 初始化串口
    Serial_Init(DEBUG_BAUDRATE);
    
    printf("\r\n\r\n");
    printf("================================\r\n");
    printf("  ECU3 - Sensor Fusion Unit\r\n");
    printf("  Version: 1.0.0\r\n");
    printf("  Build Date: %s %s\r\n", __DATE__, __TIME__);
    printf("================================\r\n\r\n");
    
    // 初始化LED
    LED_Init();
    
    // 系统启动指示
    printf("System initializing...\r\n");
    
    LED1_ON();
    Delay_ms(200);
    LED2_ON();
    Delay_ms(200);
    LED3_ON();
    Delay_ms(200);
    
    LED1_OFF();
    LED2_OFF();
    LED3_OFF();
    
    // 初始化CAN
    MyCAN_Init(CAN_BAUDRATE);
    
    // 初始化传感器
    printf("Initializing sensors...\r\n");
    
    // 1. 初始化MPU6050
    if(MPU6050_Init())
	{
        printf("MPU6050 initialized successfully\r\n");
        LED1_ON();
    }
	else
	{
        printf("MPU6050 initialization failed\r\n");
        error_code |= ERROR_MPU6050_FAIL;
    }
    
    // 2. 初始化超声波
    Ultrasonic_Init();
    printf("Ultrasonic initialized\r\n");
    LED2_ON();
    
    // 3. 初始化电压检测
    Voltage_Init();
    printf("Voltage detection initialized\r\n");
    LED3_ON();
    
    // 等待传感器稳定
    Delay_ms(1000);
    
    LED1_OFF();
    LED2_OFF();
    LED3_OFF();
    
    // 传感器自检
    printf("Running sensor self-tests...\r\n");
    //Sensor_Self_Test();
    
    // 初始化传感器融合
    //Sensor_Fusion_Init();
    
    // 启动系统
    system_state = SYS_READY;
    system_uptime = 0;
    sensor_ready = 1;
    
    printf("System initialized successfully!\r\n");
    
    // 启动成功指示
    for(int i = 0; i < 3; i++)
	{
        LED1_ON();
		LED2_ON();
		LED3_ON();
        Delay_ms(100);
        LED1_OFF();
		LED2_OFF();
		LED3_OFF();
        Delay_ms(100);
    }
}

int main(void) 
{
    System_Init();
	
    while(1) 
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
