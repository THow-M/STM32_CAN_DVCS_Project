#ifndef __ECU1_MAIN_H
#define __ECU1_MAIN_H

//系统状态枚举
typedef enum
{
	SYSTEM_IDLE = 0,                        //空闲
	SYSTEM_READY,                           //就绪
	REMOTE_CONTROL,                         //远程控制模式
	SENSOR_DISPLAY,                         //传感器显示模式
	SYSTEM_MONITOR,                         //系统监控模式
	CAN_TEST,                               //CAN测试模式
	PARAM_SETTING,                          //参数设置模式
	SYSTEM_ERROR                            //错误
} System_State;

//系统常量
#define NODE_NUM                  3         //ECU节点总数
#define HEARTBEAT_TIMEOUT         3000      //心跳超时时间（ms）
#define HEARTBEAT_PERIOD          1000      //心跳发送周期（ms）

//外部全局变量声明
extern System_State system_state;
extern uint8_t selected_menu;               //当前选中的菜单索引
extern uint32_t last_heartbeat_time;
extern uint32_t last_display_time;
extern uint32_t last_can_send_time;

#endif
