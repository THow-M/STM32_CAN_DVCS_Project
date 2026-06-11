#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "Delay.h"
#include "Key.h"
#include "LED.h"
#include "Serial.h"
#include "MyCAN.h"
#include "Timer.h"

int main(void) 
{
	OLED_Init();
	Key_Init();
	LED_Init();
	Timer_Init();
	Serial_Init(115200);
	MyCAN_Init(CAN_BAUD_500K);
	
    while(1) 
	{
        
    }
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		Key_Tick();
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
