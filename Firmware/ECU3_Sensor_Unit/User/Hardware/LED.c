#include "stm32f10x.h"                  // Device header

/**
  * 函    数：LED初始化
  * 参    数：无
  * 返 回 值：无
  */
void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 开启GPIOB时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    // 配置PB12-PB15为推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 初始状态：全部熄灭
    GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
}

/**
  * 函    数：LED控制
  * 参    数：无
  * 返 回 值：无
  */
void LED_Control(uint8_t led, uint8_t state)
{
    uint16_t pin;
    
    switch(led)
	{
        case 1: pin = GPIO_Pin_12; break;
        case 2: pin = GPIO_Pin_13; break;
        case 3: pin = GPIO_Pin_14; break;
        case 4: pin = GPIO_Pin_15; break;
        default: return;
    }
    
    if(state)
	{
        GPIO_SetBits(GPIOB, pin);
    }
	else
	{
        GPIO_ResetBits(GPIOB, pin);
    }
}
