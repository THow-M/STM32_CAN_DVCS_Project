#ifndef __ECU2_MAIN_H
#define __ECU2_MAIN_H

//--------------------------系统常量-----------------------------
#define NODE_NUM                  3         //ECU节点总数
#define HEARTBEAT_TIMEOUT         3000      //心跳超时时间（ms）
#define HEARTBEAT_PERIOD          1000      //心跳发送周期（ms）

// 系统状态定义
typedef enum
{
    SYS_IDLE = 0,       // 空闲
    SYS_READY,         // 准备就绪
    SYS_RUN,           // 运行
    SYS_ERROR,         // 错误
    SYS_CALIBRATING    // 校准
} System_State;

// 控制模式定义
#define CONTROL_MODE_MANUAL  0
#define CONTROL_MODE_AUTO    1

// 系统命令定义
#define SYS_CMD_START        1
#define SYS_CMD_STOP         2
#define SYS_CMD_RESET        3
#define SYS_CMD_MANUAL       4
#define SYS_CMD_AUTO         5
#define SYS_CMD_CALIBRATE    6
#define SYS_CMD_DIAGNOSTIC   7

// 超时定义
#define HEARTBEAT_TIMEOUT    3000  // 3秒
#define CAN_TIMEOUT          5000  // 5秒


#endif
