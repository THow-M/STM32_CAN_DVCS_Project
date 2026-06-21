#ifndef __PWM_H
#define __PWM_H

//函数声明
void PWM_Init(uint16_t arr, uint16_t psc);
void PWM_SetCompare1(uint16_t Compare);
uint16_t PWM_GetPeriod(void);

#endif
