#include "stm32f10x.h"                  // Device header
#include "System_Config.h"
#include "Timer.h"
#include "OLED.h"
#include "Delay.h"
#include "Key.h"
#include "LED.h"
#include "Serial.h"
#include "MyCAN.h"
#include "MPU6050.h"

int main(void) 
{
    Timer_Init();
	Serial_Init(115200);
	MPU6050_Init();
	MPU6050_Self_Test();

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
