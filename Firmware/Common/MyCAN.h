#ifndef __MYCAN_H
#define __MYCAN_H

extern CanRxMsg RxMessage;

//CAN波特率定义
typedef enum
{
	CAN_BAUD_125K = 0,
	CAN_BAUD_250K,
	CAN_BAUD_500K,
	CAN_BAUD_1M
} CAN_BaudRate;

// ECU节点ID
#define NODE_ID_ECU1              0x01    // 车身控制
#define NODE_ID_ECU2              0x02    // 电机驱动
#define NODE_ID_ECU3              0x03    // 传感器单元

//报文ID定义
#define MSG_ID_HEARTBEAT          0x100   //心跳包
#define MSG_ID_SPEED_CMD          0x101   //速度指令
#define MSG_ID_MOTOR_STATUS       0x102   //电机状态
#define MSG_ID_SENSOR_DATA        0x103   //传感器数据
#define MSG_ID_SYSTEM_CTRL        0x104   //系统控制
#define MSG_ID_ERROR_REPORT       0x105   //错误报告

//数据结构定义
#pragma pack(push,1)   //1字节对齐

//心跳包数据结构（8字节）
typedef struct
{
	uint8_t node_id;              //节点ID
	uint8_t status;               //状态：0=正常，1=警告，2=错误
	uint8_t error_code;           //错误代码
	uint32_t uptime;              //运行时间（秒）
} HeartBeat_Data;

//速度指令数据结构（8字节）
typedef struct
{
	int16_t target_speed;         //目标速度（RPM）
	uint8_t direction;            //方向：0=停止，1=正转，2=反转
	uint8_t acceleration;         //加速度（0-100%）
	uint8_t reserved[4];          //保留字节
} SpeedCmd_Data;

//电机状态数据结构（8字节）
typedef struct
{
	int16_t actual_speed;         //实际速度（RPM）
	uint16_t current;             //电流（mA）
	//uint8_t temperature;          //温度（）
	uint8_t status;               //状态位
	uint16_t resevered;           //保留
} MotorStatus_Data;

//传感器数据结构（8字节）
typedef struct
{
	uint16_t distance;            //距离（mm）
	int16_t pitch;                //俯仰角（0.1度）
	int16_t roll;                 //横滚角（0.1度）
	int16_t yaw;                  //航向角（0.1度）
	uint16_t voltage;             //电压（mV）
} Sensor_Data;

//系统控制数据结构（8字节）
typedef struct
{
	uint8_t command;              //命令：1=启动，2=停止，3=复位
	uint8_t param1;               //参数1
	uint8_t param2;               //参数2
	uint8_t reserved[5];          //保留
} SystemCtrl_Data;

//错误报告数据结构（8字节）
typedef struct
{
	uint8_t node_id;              //节点ID
	uint8_t error_type;           //错误类型
	uint16_t error_code;          //错误代码
	uint32_t timestamp;           //时间戳（ms）
} ErrorReport_Data;

#pragma pack(pop)   //恢复默认对齐

//函数声明
void MyCAN_Init(CAN_BaudRate baudrate);
uint8_t MyCAN_Send_Message(uint32_t ID,uint8_t Len,uint8_t* Data);
uint8_t MyCAN_Receive_Message(uint32_t* ID, uint8_t* Len, uint8_t* Data);
void MyCAN_Send_Heartbeat(uint8_t node_id, uint8_t status, uint8_t error_code, uint32_t uptime);
void MyCAN_Send_SpeedCmd(int16_t speed, uint8_t direction, uint8_t acceleration);
void MyCAN_Send_MotorStatus(int16_t speed, uint16_t current, /*uint8_t temp,*/ uint8_t status);
void MyCAN_Send_SensorData(uint16_t distance, int16_t pitch, int16_t roll, int16_t yaw, uint16_t voltage);
void MyCAN_Send_SystemCtrl(uint8_t command, uint8_t param1, uint8_t param2);
void MyCAN_Send_ErrorReport(uint8_t node_id, uint8_t error_type, uint16_t error_code);


#endif
