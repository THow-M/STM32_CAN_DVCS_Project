#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "Delay.h"
#include "Key.h"
#include "LED.h"
#include "Timer.h"
#include "Serial.h"
#include "PWM.h"
#include "Encoder.h"
#include "Motor.h"

int16_t PWM = 0;
float Speed;

int main(void) 
{
	Timer_Init();
	OLED_Init();
	LED_Init();
	Encoder_Init();
    Motor_Init(999,71);		//电机初始化
	
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
		
		
		
		Speed = Encoder_CalculateSpeed(40);
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
