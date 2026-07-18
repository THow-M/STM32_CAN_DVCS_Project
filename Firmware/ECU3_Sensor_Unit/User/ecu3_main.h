#ifndef __ECU3_MAIN_H
#define __ECU3_MAIN_H

//--------------------------系统常量-----------------------------
#define NODE_NUM                  3         //ECU节点总数
#define HEARTBEAT_TIMEOUT         3000      //心跳超时时间（ms）
#define CAN_TIMEOUT               5000      // 超时定义5秒

// 系统状态定义
typedef enum
{
    SYS_IDLE = 0,       // 空闲
    SYS_READY,          // 准备就绪
    SYS_RUN,            // 运行
    SYS_ERROR,          // 错误
    SYS_CALIBRATING     // 校准
} System_State;

// 传感器融合数据结构
typedef struct
{
    uint16_t distance_mm;       // 距离 (mm)
    float distance_cm;          // 距离 (cm)
    float filtered_distance_cm; // 滤波后的距离
    float roll;                 // 横滚角 (度)
    float pitch;                // 俯仰角 (度)
    float yaw;                  // 航向角 (度)
    float filtered_roll;        // 滤波后的横滚角
    float filtered_pitch;       // 滤波后的俯仰角
    float filtered_yaw;         // 滤波后的航向角
    float voltage_v;            // 电压 (V)
    float temperature_c;        // 温度 (°C)
    uint8_t valid;              // 数据是否有效
    uint32_t timestamp;         // 时间戳 (ms)
} Sensor_Fusion;

/*
 * 错误代码定义 (位掩码, 可用 | 组合, 用 & 判断)
 * 修复: 用 #undef 覆盖 System_Config.h 中的顺序值定义
 *       统一为位掩码风格, 便于 ECU3 做多错误组合判断
 */
#undef ERROR_NONE
#define ERROR_NONE              0x00    // 无错误

#define ERROR_MPU6050_FAIL      0x01    // ECU3 特有: MPU6050 通信失败
#define ERROR_MPU6050_SELFTEST  0x02    // ECU3 特有: MPU6050 自检失败
#define ERROR_ULTRASONIC_FAIL   0x04    // ECU3 特有: 超声波模块故障
#define ERROR_VOLTAGE_FAIL      0x08    // ECU3 特有: 电压采集故障

#undef ERROR_VOLTAGE_LOW
#define ERROR_VOLTAGE_LOW       0x10    // 电压过低 (覆盖 System_Config.h 的 0x04)

#undef ERROR_CAN_COMM
#define ERROR_CAN_COMM          0x20    // CAN 通信故障 (覆盖 System_Config.h 的 0x01)

#define ERROR_SENSOR_FUSION     0x40    // ECU3 特有: 传感器融合异常

// 系统命令定义
#define SYS_CMD_START        1
#define SYS_CMD_STOP         2
#define SYS_CMD_RESET        3
#define SYS_CMD_CALIBRATE    4
#define SYS_CMD_DIAGNOSTIC   5
#define SYS_CMD_SELF_TEST    6

// 函数声明
void System_Init(void);


// 外部变量声明
extern System_State system_state;
extern uint32_t system_uptime;
extern uint8_t error_code;
extern uint8_t can_connected;
extern Sensor_Fusion sensor_fusion;
extern uint8_t sensor_ready;
extern uint8_t calibration_complete;

#endif
