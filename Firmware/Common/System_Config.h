#ifndef __SYSTEM_CONFIG_H
#define __SYSTEM_CONFIG_H

//系统时钟配置
#define SYSTEM_CLOCK_FREQ 72000000

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

//ECU3节点配置
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
#define ERROR_NONE                    0x00u   /* 无错误 */
#define ERROR_CAN_COMM                0x01u   /* CAN 通信故障 */
#define ERROR_VOLTAGE_LOW             0x02u   /* 电压过低 */
#define ERROR_MPU6050_FAIL            0x04u   /* MPU6050 故障 (ECU3 专用) */
#define ERROR_ULTRASONIC_FAIL         0x08u   /* 超声波故障 (ECU3 专用) */
#define ERROR_MOTOR_FAULT             0x10u   /* 电机故障 (ECU2 专用，原 ERROR_MOTOR_OVERHEAT) */
#define ERROR_SENSOR_FUSION           0x20u   /* 融合异常 (ECU3 专用) */
#define ERROR_OVERCURRENT             0x40u   /* 过流 (ECU2 专用) */
#define ERROR_WATCHDOG                0x80u   /* 看门狗复位 */

//系统状态定义
#define STATUS_NORMAL                 0x00
#define STATUS_WARNING                0x01
#define STATUS_ERROR                  0x02
#define STATUS_CRITICAL               0x03

#endif
