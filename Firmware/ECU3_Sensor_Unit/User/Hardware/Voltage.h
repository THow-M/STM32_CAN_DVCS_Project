#ifndef __VOLTAGE_H
#define __VOLTAGE_H

// 电压状态
typedef enum
{
    VOLTAGE_LOW = 0,
    VOLTAGE_NORMAL,
    VOLTAGE_HIGH
} Voltage_Status;

// 电压数据结构
typedef struct
{
    float voltage_v;           // 电压 (V)
    uint16_t voltage_mv;       // 电压 (mV)
    float filtered_v;          // 滤波后的电压
    uint8_t battery_percent;   // 电量百分比
    Voltage_Status status;     // 电压状态
} Voltage_Data;

// 电压范围
#define VOLTAGE_MIN     10.0f  // 最低电压
#define VOLTAGE_MAX     13.0f  // 最高电压
#define ADC_SAMPLE_COUNT 10    // ADC采样次数

// 函数声明
void Voltage_Init(void);
uint16_t Voltage_Read_ADC(void);


// 外部变量声明
extern Voltage_Data voltage_data;

#endif
