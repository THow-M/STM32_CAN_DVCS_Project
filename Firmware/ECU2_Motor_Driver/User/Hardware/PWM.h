#ifndef __PWM_H
#define __PWM_H

// 电机方向定义
#define MOTOR_STOP                   0
#define MOTOR_FORWARD                1
#define MOTOR_REVERSE                2
#define MOTOR_BRAKE                  3

// 电机状态定义
typedef enum
{
    MOTOR_STATE_IDLE = 0,            // 空闲
    MOTOR_STATE_RUN,                 // 运行
    MOTOR_STATE_STOP,                // 停止
    MOTOR_STATE_BRAKE,               // 刹车
    MOTOR_STATE_ERROR,               // 错误
    MOTOR_STATE_CALIBRATING          // 校准
} Motor_State;

// 电机保护结构
typedef struct
{
    uint8_t over_current:1;    // 过流
    uint8_t over_temp:1;       // 过热
    uint8_t stall:1;           // 堵转
    uint8_t over_voltage:1;    // 过压
    uint8_t under_voltage:1;   // 欠压
} Motor_Protection;

// 电机控制结构
typedef struct
{
    int16_t current_speed;     // 当前速度
    int16_t target_speed;      // 目标速度
    uint8_t direction;         // 方向
    Motor_State state;         // 状态
    Motor_Protection protection; // 保护状态
    uint8_t error_code;        // 错误代码
    uint16_t error_count;      // 错误计数
} Motor_Control;

// 电机状态结构
typedef struct
{
    int16_t speed;             // 速度
    int16_t target_speed;      // 目标速度
    uint8_t direction;         // 方向
    Motor_State state;         // 状态
    uint8_t error_code;        // 错误代码
    uint8_t protection_status; // 保护状态
} Motor_Status;

// 错误代码定义
#define ERROR_OVER_CURRENT     0x01
#define ERROR_OVER_TEMP        0x02
#define ERROR_STALL            0x04
#define ERROR_OVER_VOLTAGE     0x08
#define ERROR_UNDER_VOLTAGE    0x10

// 电机参数
#define MOTOR_MAX_CURRENT     2000  // 最大电流2A
#define MOTOR_MAX_TEMP        80    // 最高温度80℃
#define MOTOR_MAX_SPEED       1000  // 最大速度

//函数声明
void Motor_Init(uint16_t arr, uint16_t psc);
void Motor_SetSpeed(int16_t speed, uint8_t direction);
void Motor_SmoothAcceleration(int16_t target_speed, uint8_t direction, uint16_t acc_time_ms);
uint8_t Motor_Protection_Check(void);
void Motor_Error_Handler(uint8_t error_code);
Motor_Status Motor_GetStatus(void);

#endif
