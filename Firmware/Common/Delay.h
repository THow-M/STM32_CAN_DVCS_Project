#ifndef __DELAY_H
#define __DELAY_H

extern volatile uint32_t Tick_ms;

void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);
uint32_t HAL_GetTick(void);

#endif
