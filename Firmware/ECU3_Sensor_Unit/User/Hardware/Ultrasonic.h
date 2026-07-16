#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

// 超声波数据结构
typedef struct
{
    uint16_t distance_mm;      // 距离 (mm)
    float distance_cm;         // 距离 (cm)
    uint8_t valid;             // 数据是否有效
    uint8_t signal_strength;   // 信号强度 (%)
} Ultrasonic_Data;

//函数声明
void Ultrasonic_Trigger(void);

#endif
