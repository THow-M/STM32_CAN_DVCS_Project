#ifndef __VOLTAGE_H
#define __VOLTAGE_H

/*
 * ============================================================
 *  电压检测模块 - 硬件接线说明
 * ============================================================
 *
 *  引脚分配:
 *    PA1 = ADC1_IN1 (模拟输入)
 *
 *  分压电路 (适用于 12V 电池系统, 满量程 19.8V):
 *
 *      Vbat (0~15V)
 *        │
 *        R1 = 10kΩ (1% 金属膜电阻)
 *        │
 *        ├─────── PA1 (ADC1_IN1)
 *        │       │
 *        │       └── 100nF 陶瓷电容 (到 GND, 滤波)
 *        │              │
 *        R2 = 2kΩ       GND
 *        │ (1% 金属膜)
 *        GND
 *
 *  关键参数:
 *    分压比           = (R1+R2)/R2 = 6.0
 *    满量程电压       = 3.3V × 6 = 19.8V
 *    电压分辨率       = 0.806mV × 6 = 4.83 mV/LSB
 *    ADC 时钟         = 72MHz / 6 = 12MHz (≤14MHz 规格)
 *    单次转换时间     = 252 / 12MHz = 21μs
 *
 *  接线要求:
 *    1. R1/R2 必须使用 1% 精度金属膜电阻
 *    2. 分压电阻靠近 MCU 放置, PA1 走线 < 10mm
 *    3. PA1 到 GND 加 100nF 陶瓷电容抑制高频噪声
 *    4. 被测电源 GND 必须与 STM32 GND 共地
 *    5. 被测为电池/感性负载时, Vbat 入口加 TVS (如 SMBJ15A)
 * ============================================================
 */

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
void Voltage_Update(void);
Voltage_Data Voltage_GetData(void);
void Voltage_Calibrate(float actual_voltage);
void Voltage_Diagnostic(void);

// 外部变量声明
extern Voltage_Data voltage_data;

#endif
