#include "stm32f10x.h"                  // Device header
#include "Delay.h"

/**
  * 函    数：LED初始化
  * 参    数：无
  * 返 回 值：无
  */
void LED_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//开启GPIOA的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);						//将PA1、PA2和PA3引脚初始化为推挽输出
	
	/*设置GPIO初始化后的默认电平*/
	GPIO_SetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);				//设置PA1、PA2和PA3引脚为高电平
}

/**
  * 函    数：LED1开启
  * 参    数：无
  * 返 回 值：无
  */
void LED1_ON(void)
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_1);		//设置PA1引脚为低电平
}

/**
  * 函    数：LED1关闭
  * 参    数：无
  * 返 回 值：无
  */
void LED1_OFF(void)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_1);		//设置PA1引脚为高电平
}

/**
  * 函    数：LED1状态翻转
  * 参    数：无
  * 返 回 值：无
  */
void LED1_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1) == 0)		//获取输出寄存器的状态，如果当前引脚输出低电平
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_1);					//则设置PA1引脚为高电平
	}
	else													//否则，即当前引脚输出高电平
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_1);					//则设置PA1引脚为低电平
	}
}

/**
  * 函    数：LED2开启
  * 参    数：无
  * 返 回 值：无
  */
void LED2_ON(void)
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);		//设置PA2引脚为低电平
}

/**
  * 函    数：LED2关闭
  * 参    数：无
  * 返 回 值：无
  */
void LED2_OFF(void)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_2);		//设置PA2引脚为高电平
}

/**
  * 函    数：LED2状态翻转
  * 参    数：无
  * 返 回 值：无
  */
void LED2_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_2) == 0)		//获取输出寄存器的状态，如果当前引脚输出低电平
	{                                                  
		GPIO_SetBits(GPIOA, GPIO_Pin_2);               		//则设置PA2引脚为高电平
	}                                                  
	else                                               		//否则，即当前引脚输出高电平
	{                                                  
		GPIO_ResetBits(GPIOA, GPIO_Pin_2);             		//则设置PA2引脚为低电平
	}
}

/**
  * 函    数：LED3开启
  * 参    数：无
  * 返 回 值：无
  */
void LED3_ON(void)
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_3);		//设置PA3引脚为低电平
}

/**
  * 函    数：LED3关闭
  * 参    数：无
  * 返 回 值：无
  */
void LED3_OFF(void)
{
	GPIO_SetBits(GPIOA, GPIO_Pin_3);		//设置PA3引脚为高电平
}

/**
  * 函    数：LED2状态翻转
  * 参    数：无
  * 返 回 值：无
  */
void LED3_Turn(void)
{
	if (GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_3) == 0)		//获取输出寄存器的状态，如果当前引脚输出低电平
	{                                                  
		GPIO_SetBits(GPIOA, GPIO_Pin_3);               		//则设置PA3引脚为高电平
	}                                                  
	else                                               		//否则，即当前引脚输出高电平
	{                                                  
		GPIO_ResetBits(GPIOA, GPIO_Pin_3);             		//则设置PA3引脚为低电平
	}
}

/**
  * 函    数：LED流水灯
  * 参    数：direction 流水灯方向，1为正向，0为反向
  * 参    数：delay 闪烁间隔（ms）
  * 返 回 值：无
  */
void LED_RunningWater(uint8_t direction, uint16_t delay)
{
	static uint8_t current_led = 0;
	
	GPIO_SetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
	
	switch(current_led)
	{
        case 0: GPIO_ResetBits(GPIOA, GPIO_Pin_1); break;
        case 1: GPIO_ResetBits(GPIOA, GPIO_Pin_2); break;
        case 2: GPIO_ResetBits(GPIOA, GPIO_Pin_3); break;
		default: break;
    }
	
	if(direction)
	{
        // 正向
        current_led = (current_led + 1) % 3;
    }
	else
	{
        // 反向
        current_led = (current_led == 0) ? 2 : (current_led - 1);
    }
    
    Delay_ms(delay);
}
