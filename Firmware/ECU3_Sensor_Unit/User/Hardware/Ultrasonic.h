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
void Ultrasonic_Init(void);
void Ultrasonic_Trigger(void);
uint16_t Ultrasonic_GetDistance(void);
void Ultrasonic_Update(void);
void Ultrasonic_Calculate_Distance(void);
void Ultrasonic_Calibrate(void);
void Ultrasonic_Diagnostic(void);
void Ultrasonic_Test_All(void);
// 外部变量声明
extern Ultrasonic_Data ultrasonic_data;
extern volatile uint8_t ultrasonic_data_ready;

#endif
