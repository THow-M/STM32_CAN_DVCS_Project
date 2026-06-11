#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Key.h"

uint8_t Key_Flag;

// 按键参数
#define DEBOUNCE_MS     20      // 消抖时间（ms）
#define LONG_PRESS_MS   800     // 长按判定时间（ms）
#define HOLD_INTERVAL_MS 200    // 连按触发间隔（ms）

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
	GPIO_InitStructure.GPIO_Pin = KEY1_PIN | KEY2_PIN | KEY3_PIN | KEY4_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/** 函  数：按键扫描
  * 参  数：mode 扫描模式，1为连按，0为点按
  * 返回值：按下按键的键码
  */
uint8_t Key_Scan(uint8_t mode)
{
    static uint8_t key_up = 1;   // 按键松开标志
    
    if (mode) key_up = 1;        // 支持连按
    
    if (key_up && (KEY1 == KEY_PRESS || KEY2 == KEY_PRESS || KEY3 == KEY_PRESS || KEY4 == KEY_PRESS))
	{
        Delay_ms(10);
        key_up = 0;
        
        if (KEY1 == KEY_PRESS)  return KEY1_PRES;
        if (KEY2 == KEY_PRESS)  return KEY2_PRES;
        if (KEY3 == KEY_PRESS)  return KEY3_PRES;
        if (KEY4 == KEY_PRESS)  return KEY4_PRES;
    }
	else if (KEY1 == 1 && KEY2 == 1 && KEY3 == 1 && KEY4 == 1)
	{
        key_up = 1;
    }
    
    return 0;   // 无按键按下
}

/**
  * 函    数：按键获取键码
  * 参    数：无
  * 返 回 值：
  * 注意事项：
  */
uint8_t Key_GetState(void)
{
	if(KEY1 == 0)
	{
		return KEY_PRESS;
	}
	return KEY_RELEASE;
}

// 按键长按检测
uint8_t Key_LongPress(Key_Type key, uint16_t long_press_time)
{
    static uint32_t press_start_time[4] = {0};
    static uint8_t long_press_flag[4] = {0};
    
    uint8_t index = key - 1;   // KEY_1 -> 0
    
    if (Key_GetState() == KEY_PRESS)
	{
        if (press_start_time[index] == 0)
		{
            press_start_time[index] = HAL_GetTick();
            long_press_flag[index] = 0;
        }
		else if (!long_press_flag[index] && 
                   (HAL_GetTick() - press_start_time[index] > long_press_time))
		{
            long_press_flag[index] = 1;
            return 1;   // 长按事件
        }
    } else {
        press_start_time[index] = 0;
        long_press_flag[index] = 0;
    }
    
    return 0;
}

/** 函  数：检查指定指定标志位
  * 参  数：
  * 返回值：
  */
uint8_t Key_Check(uint8_t Flag)
{
	if(Key_Flag & Flag)
	{
		if(Flag != KEY_HOLD)
		{
			Key_Flag &= ~Flag;
		}
		
		return 1;
	}
	return 0;
}

void Key_Tick(void)
{
	static uint8_t Count;
	static uint8_t CurrState, PrevState;
	
	Count ++;
	if(Count >= 20)
	{
		Count = 0;
		PrevState = Key_GetState();
		CurrState = Key_GetState();
		
		if(CurrState == KEY_PRESS)
		{
			Key_Flag |= KEY_HOLD;
		}
		else
		{
			Key_Flag &= ~KEY_HOLD;
		}
		
		if(PrevState == KEY_RELEASE && CurrState == KEY_PRESS)
		{
			Key_Flag |= KEY_DOWN;
		}
		
		if(PrevState == KEY_PRESS && CurrState == KEY_RELEASE)
		{
			Key_Flag |= KEY_UP;
		}
	}
}
