#ifndef __KEY_H
#define __KEY_H

//按键数量
#define KEY_COUNT        4

//按键编号
#define KEY_NUM_1        0
#define KEY_NUM_2        1
#define KEY_NUM_3        2
#define KEY_NUM_4        3

//按键引脚定义
#define KEY1_PIN        GPIO_Pin_11
#define KEY2_PIN        GPIO_Pin_1
#define KEY3_PIN        GPIO_Pin_13
#define KEY4_PIN        GPIO_Pin_15
#define KEY_PORT        GPIOB

// 按键电平（低电平有效）
#define KEY_PRESS       0
#define KEY_RELEASE     1

// 读取按键电平
#define KEY1            GPIO_ReadInputDataBit(KEY_PORT, KEY1_PIN)
#define KEY2            GPIO_ReadInputDataBit(KEY_PORT, KEY2_PIN)
#define KEY3            GPIO_ReadInputDataBit(KEY_PORT, KEY3_PIN)
#define KEY4            GPIO_ReadInputDataBit(KEY_PORT, KEY4_PIN)

//按键按下
#define KEY1_PRESS      1
#define KEY2_PRESS      2
#define KEY3_PRESS      3
#define KEY4_PRESS      4


//仿状态机 标志位位掩码
#define KEY_HOLD        0x01
#define KEY_DOWN        0x02
#define KEY_UP          0x04
#define KEY_SINGLE      0x08
#define KEY_DOUBLE      0x10
#define KEY_LONG        0x20
#define KEY_REPEAT      0x40

void Key_Init(void);
uint8_t Key_GetState(uint8_t n);
//uint8_t Key_Check(uint8_t n, uint8_t Flag);
uint8_t Key_Check(uint8_t Flag);
void Key_Scan(void);

#endif
