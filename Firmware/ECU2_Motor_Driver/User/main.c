#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "Delay.h"
#include "Key.h"
#include "LED.h"
#include "Serial.h"
#include "Motor.h"
#include "PWM.h"

int16_t PWM = 0;

int main(void) 
{
	LED_Init();
    Motor_Init(999,71);		//电机初始化
	//GPIO_SetBits(GPIOB, GPIO_Pin_0);
    //GPIO_ResetBits(GPIOB, GPIO_Pin_1);
	//Motor_SetSpeed(300,1);
	LED_ON();
	while (1)
	{
		

	}
}
