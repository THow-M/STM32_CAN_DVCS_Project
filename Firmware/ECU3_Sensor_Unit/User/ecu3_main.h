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


// 系统命令定义
#define SYS_CMD_START        1
#define SYS_CMD_STOP         2
#define SYS_CMD_RESET        3
#define SYS_CMD_CALIBRATE    4
#define SYS_CMD_DIAGNOSTIC   5
#define SYS_CMD_SELF_TEST    6

// 函数声明
void System_Init(void);
void Sensor_Self_Test(void);
void Sensor_Update_All(void);
void Sensor_Fusion_Init(void);
void Sensor_Fusion_Process(void);
void Sensor_Anomaly_Detection(void);
void Sensor_Data_Filtering(void);
void Auto_Calibration(void);
void Control_Loop(void);
void Communication_Handler(void);
void CAN_Data_Handler(uint32_t id, uint8_t len, uint8_t* data);
void System_Diagnostic(void);
void Error_Handler(void);

// 外部变量声明
extern System_State system_state;
extern volatile uint32_t system_uptime;
extern uint8_t error_code;
extern uint8_t can_connected;
extern Sensor_Fusion sensor_fusion;
extern uint8_t sensor_ready;
extern uint8_t calibration_complete;

#endif
