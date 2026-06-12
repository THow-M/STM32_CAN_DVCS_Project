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
	
	uint16_t Num1 = 0;
    while(1) 
	{
        if(Key_Check(3,KEY_SINGLE) || Key_Check(3,KEY_REPEAT))
		{
			Num1++;
		}
		
		OLED_ShowNum(1,1,Num1,5);
		OLED_ShowNum(2,1,Num1,5);
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
