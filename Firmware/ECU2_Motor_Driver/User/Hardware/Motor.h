#ifndef __MOTOR_H
#define __MOTOR_H

//电机方向定义
#define MOTOR_STOP     0
#define MOTOR_FORWARD  1
#define MOTOR_REVERSE  2
#define MOTOR_BRAKE    3

//电机状态枚举
typedef enum {
    MOTOR_STATE_IDLE = 0,
    MOTOR_STATE_RUN,
    MOTOR_STATE_STOP,
    MOTOR_STATE_BRAKE,
    MOTOR_STATE_ERROR,
    MOTOR_STATE_LOCKED,
    MOTOR_STATE_CALIBRATING
} Motor_State;

//电机保护结构
typedef struct {
    uint8_t over_current:1;
    uint8_t over_temp:1;
    uint8_t stall:1;
    uint8_t over_speed:1;
    uint8_t under_voltage:1;
} Motor_Protection;

//电机控制结构
typedef struct {
    int16_t current_speed;
    int16_t target_speed;
    uint8_t direction;
    Motor_State state;
    Motor_Protection protection;
    uint8_t error_code;
    uint16_t error_count;
} Motor_Control;

//电机状态结构
typedef struct {
    int16_t speed;
    int16_t target_speed;
    uint8_t direction;
    Motor_State state;
    uint8_t error_code;
    uint8_t protection_status;
} Motor_Status;

//错误代码
#define ERROR_OVER_CURRENT     0x01
#define ERROR_OVER_TEMP        0x02
#define ERROR_STALL            0x04
#define ERROR_OVER_SPEED     0x08
#define ERROR_UNDER_VOLTAGE    0x10

#define MOTOR_MAX_CURRENT     2000
#define MOTOR_MAX_TEMP        80
#define MOTOR_MAX_SPEED       1000

// 控制模式
#define CONTROL_MODE_MANUAL    0
#define CONTROL_MODE_AUTO      1

//函数声明
void Motor_Init(uint16_t arr, uint16_t psc);
void Motor_SetControlMode(uint8_t mode);
void Motor_SetTargetSpeed(float speed_rpm, uint8_t direction);
void Motor_EmergencyStop(void);
void Motor_RunPIDControl(void);
uint8_t Motor_ProtectionCheck(void);
void Motor_ErrorHandler(uint8_t error_code);
Motor_Status Motor_GetStatus(void);
float Motor_GetSpeed(void);
void Motor_AutoTune(void);
void Motor_Diagnostic(void);

uint16_t Motor_GetCurrent(void);

extern Motor_Control motor_control;

#endif
