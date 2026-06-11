#ifndef __KEY_H
#define __KEY_H

//按键引脚定义
#define KEY1_PIN        GPIO_Pin_11
#define KEY2_PIN        GPIO_Pin_1
#define KEY3_PIN        GPIO_Pin_13
#define KEY4_PIN        GPIO_Pin_15
#define KEY_PORT        GPIOB

// 按键按下电平（低电平有效）
#define KEY_PRESS       0
#define KEY_RELEASE     1

// 读取按键电平
#define KEY1            GPIO_ReadInputDataBit(KEY_PORT, KEY1_PIN)
#define KEY2            GPIO_ReadInputDataBit(KEY_PORT, KEY2_PIN)
#define KEY3            GPIO_ReadInputDataBit(KEY_PORT, KEY3_PIN)
#define KEY4            GPIO_ReadInputDataBit(KEY_PORT, KEY4_PIN)

// 按键返回值
#define KEY_NONE        0
#define KEY1_PRES       1
#define KEY2_PRES       2
#define KEY3_PRES       3
#define KEY4_PRES       4

#define KEY_HOLD        0x01
#define KEY_DOWN        0x02
#define KEY_UP          0x04

// 按键类型枚举
typedef enum
{
    KEY_1 = 1,
    KEY_2 = 2,
    KEY_3 = 3,
    KEY_4 = 4
} Key_Type;

// 按键事件类型
typedef enum
{
    KEY_EVENT_NONE = 0,
    KEY_EVENT_SHORT,    // 短按（单击）
    KEY_EVENT_LONG,     // 长按（首次触发）
    KEY_EVENT_HOLD      // 连按（按住时周期性触发）
} KeyEvent_t;

// 按键状态机结构体（每个按键一个）
typedef struct
{
    uint8_t  state;             // 当前状态：0=释放，1=按下消抖中，2=按下确认
    uint32_t press_time;        // 按下时刻（ms）
    uint8_t  long_triggered;    // 长按是否已触发
    uint32_t hold_timer;        // 连按定时器
} KeyStateMachine;

void Key_Init(void);
uint8_t Key_Scan(uint8_t mode);
uint8_t Key_GetState(void);
uint8_t Key_Check(uint8_t Flag);
void Key_Tick(void);

#endif
