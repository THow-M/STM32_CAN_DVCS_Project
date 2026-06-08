#ifndef __SYSTEM_CONFIG_H
#define __SYSTEM_CONFIG_H

//系统时钟配置
#define SYSTEM_CLOCK_FREQ 72000000;

//ECU1节点配置
#ifdef ECU1
	#define NODE_ID                   0x01
	#define NODE_NAME                 "ECU1_Body_Control"
	#define CAN_BAUDRATE              CAN_BAUD_500K
	#define HEARTBEAT_PERIOD          1000
#endif

//ECU2节点配置
#ifdef ECU2
	#define NODE_ID                   0x02
	#define NODE_NAME                 "ECU2_Motor_Drive"
	#define CAN_BAUDRATE              CAN_BAUD_500K
	#define HEARTBEAT_PERIOD          1000
#endif

//ECU1节点配置
#ifdef ECU3
	#define NODE_ID                   0x03
	#define NODE_NAME                 "ECU3_Sensor_Unit"
	#define CAN_BAUDRATE              CAN_BAUD_500K
	#define HEARTBEAT_PERIOD          1000
#endif

//调试开关
#define DEBUG_ENABLE                  1
#define DEBUG_USART                   USART1
#define DEBUG_BAUDRATE                115200

//错误代码定义
#define ERROR_NONE                    0x00
#define ERROR_CAN_COMM                0x01
#define ERROR_MOTOR_OVERHEAT          0x02
#define ERROR_SENSOR_FAIL             0x03
#define ERROR_VOLTAGE_LOW             0x04
#define ERROR_ENCODER_FAIL            0x05

//系统状态定义
#define STATUS_NORMAL                 0x00
#define STATUS_WARNING                0x01
#define STATUS_ERROR                  0x02
#define STATUS_CRITICAL               0x03

#endif
