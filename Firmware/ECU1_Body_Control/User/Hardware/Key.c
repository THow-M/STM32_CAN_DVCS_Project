#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Key.h"

// 按键参数
#define KEY_TIME_DOUBLE     200      // 双击判定时间（ms）
#define KEY_TIME_LONG       2000     // 长按判定时间（ms）
#define KEY_TIME_REPEAT     100      // 重复触发间隔（ms）

volatile uint8_t Key_Flag[KEY_COUNT];         //标志位


/**
  * 函    数：按键初始化
  * 参    数：无
  * 返 回 值：无
  */
void Key_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = KEY1_PIN | KEY2_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Pin = KEY3_PIN | KEY4_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
  * 函    数：获取按键状态
  * 参    数：n 按键编号
  * 返 回 值：按键电平
  */
uint8_t Key_GetState(uint8_t n)
{
	if(n == KEY_NUM_1)
	{
		if(KEY1 == 0)
		{
			return KEY_PRESS;
		}
	}
	else if(n == KEY_NUM_2)
	{
		if(KEY2 == 0)
		{
			return KEY_PRESS;
		}
	}
	else if(n == KEY_NUM_3)
	{
		if(KEY3 == 1)
		{
			return KEY_PRESS;
		}
	}
	else if(n == KEY_NUM_4)
	{
		if(KEY4 == 1)
		{
			return KEY_PRESS;
		}
	}
	
	return KEY_RELEASE;
}

/** 函  数：检查指定按键标志位
  * 参  数：n 要检查的按键编号
  * 参  数：Flag 要检查的标志位类型
  * 返回值：1，标志事件发生 0，标志事件未发生
  */
/*uint8_t Key_Check(uint8_t n, uint8_t Flag)
{
	if(Key_Flag[n] & Flag)
	{
		if(Flag != KEY_HOLD)
		{
			Key_Flag[n] &= ~Flag;
		}
		
		return 1;
	}
	return 0;
}*/

/** 函  数：检查指定按键标志位
  * 参  数：Flag 要检查的标志位类型
  * 返回值：KEYx_PRESS 发生检查事件的按键
  */
uint8_t Key_Check(uint8_t Flag)
{
	for(uint8_t i = 0; i < KEY_COUNT; i++)
	{
		__disable_irq();  /* 临界区保护 */
		if(Key_Flag[i] & Flag)
		{
			if(Flag != KEY_HOLD)
			{
				Key_Flag[i] &= ~Flag;
			}
			__enable_irq();
			
			switch(i)
			{
				case 0:
					return KEY1_PRESS;
				case 1:
					return KEY2_PRESS;
				case 2:
					return KEY3_PRESS;
				case 3:
					return KEY4_PRESS;
				default:
					break;
			}
			
		}
		__enable_irq();
	}

	return 0;
}

/**
  * 函    数：按键扫描（基于 HAL_GetTick 时间基准，扫描周期独立于调用频率）
  * 参    数：无
  * 返 回 值：无
  * 说    明：用 HAL_GetTick() 时间过滤，无论调用频率高低都按 20ms 周期扫描。
  *           Time[i] 按 elapsed_ms 真实经过时间递减，KEY_TIME_xxx 仍以 ms 为单位不变。
  */
void Key_Scan(void)
{
	static uint32_t last_scan_ms = 0U;
	static uint8_t i;
	static uint8_t CurrState[KEY_COUNT], PrevState[KEY_COUNT];
	static uint8_t S[KEY_COUNT];
	static uint16_t Time[KEY_COUNT];
	
	uint32_t now = HAL_GetTick();
	uint16_t elapsed_ms;
	
	/* 时间过滤：未到 20ms 扫描周期直接返回，避免高频空转 */
	if (now == last_scan_ms)
	{
		return;
	}
	elapsed_ms = (uint16_t)(now - last_scan_ms);
	if (elapsed_ms < 20U)
	{
		return;
	}
	last_scan_ms = now;
	
	/* Time[i] 按真实经过时间递减（保持 ms 单位，KEY_TIME_xxx 不动） */
	for(i = 0; i < KEY_COUNT; i++)
	{
		if(Time[i] > 0U)
		{
			if(Time[i] >= elapsed_ms)
			{
				Time[i] -= elapsed_ms;
			}
			else
			{
				Time[i] = 0U;
			}
		}
	}
	
	/* 扫描主体 */
	for(i = 0; i < KEY_COUNT; i++)
	{
		PrevState[i] = CurrState[i];
		CurrState[i] = Key_GetState(i);
		
		if(CurrState[i] == KEY_PRESS)
		{
			Key_Flag[i] |= KEY_HOLD;
		}
		else
		{
			Key_Flag[i] &= ~KEY_HOLD;
		}
		
		if(PrevState[i] == KEY_RELEASE && CurrState[i] == KEY_PRESS)
		{
			Key_Flag[i] |= KEY_DOWN;
		}
		
		if(PrevState[i] == KEY_PRESS && CurrState[i] == KEY_RELEASE)
		{
			Key_Flag[i] |= KEY_UP;
		}
		
		/* 状态转移 */
		if(S[i] == 0)        /* 空闲 */
		{
			if(CurrState[i] == KEY_PRESS)
			{
				Time[i] = KEY_TIME_LONG;
				S[i] = 1;
			}
		}
		else if(S[i] == 1)   /* 按键已按下 */
		{
			if(CurrState[i] == KEY_RELEASE)
			{
				Time[i] = KEY_TIME_DOUBLE;
				S[i] = 2;
			}
			else if(Time[i] == 0)
			{
				Time[i] = KEY_TIME_REPEAT;
				Key_Flag[i] |= KEY_LONG;
				S[i] = 4;
			}
		}
		else if(S[i] == 2)   /* 按键已松开 */
		{
			if(CurrState[i] == KEY_PRESS)
			{
				Key_Flag[i] |= KEY_DOUBLE;
				S[i] = 3;
			}
			else if(Time[i] == 0)
			{
				Key_Flag[i] |= KEY_SINGLE;
				S[i] = 0;
			}
		}
		else if(S[i] == 3)   /* 按键已双击 */
		{
			if(CurrState[i] == KEY_RELEASE)
			{
				S[i] = 0;
			}
		}
		else if(S[i] == 4)   /* 按键已长按 */
		{
			if(CurrState[i] == KEY_RELEASE)
			{
				S[i] = 0;
			}
			else if(Time[i] == 0)
			{
				Time[i] = KEY_TIME_REPEAT;
				Key_Flag[i] |= KEY_REPEAT;
				S[i] = 4;
			}
		}
	}
}
