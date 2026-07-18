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


int main(void) 
{
    Timer_Init();
	Serial_Init(115200);
	Voltage_Init();
	
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
