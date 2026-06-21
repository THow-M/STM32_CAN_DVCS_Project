#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "Delay.h"
#include "Key.h"
#include "LED.h"
#include "Serial.h"
#include "PWM.h"

int main(void) 
{
    LED_Init();
	PWM_Init(999, 71);

    while(1) 
	{
        LED_ON();
		Delay_ms(500);
		LED_OFF();
		Delay_ms(500);
		LED_Turn();
		Delay_ms(500);
		LED_Turn();
		Delay_ms(500);
    }
}
