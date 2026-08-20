#ifndef __SYSTEM_CONFIG_H
#define __SYSTEM_CONFIG_H

//系统时钟配置
#define SYSTEM_CLOCK_FREQ 72000000UL

//ECU1节点配置
#ifdef ECU1
	#define NODE_ID                   0x01U
	#define NODE_NAME                 "ECU1_Body_Control"
	#define CAN_BAUDRATE              CAN_BAUD_500K
	#define HEARTBEAT_PERIOD          1000U
#endif

//ECU2节点配置
#ifdef ECU2
	#define NODE_ID                   0x02U
	#define NODE_NAME                 "ECU2_Motor_Drive"
	#define CAN_BAUDRATE              CAN_BAUD_500K
	#define HEARTBEAT_PERIOD          1000U
#endif

//ECU3节点配置
#ifdef ECU3
	#define NODE_ID                   0x03U
	#define NODE_NAME                 "ECU3_Sensor_Unit"
	#define CAN_BAUDRATE              CAN_BAUD_500K
	#define HEARTBEAT_PERIOD          1000U
#endif

//调试开关
#define DEBUG_ENABLE                  1U
#define DEBUG_USART                   USART1
#define DEBUG_BAUDRATE                115200UL

//错误代码定义
#define ERROR_NONE                    0x00U   /* 无错误 */
#define ERROR_CAN_COMM                0x01U   /* CAN 通信故障 */
#define ERROR_VOLTAGE_LOW             0x02U   /* 电压过低 */
#define ERROR_MPU6050_FAIL            0x04U   /* MPU6050 故障 (ECU3 专用) */
#define ERROR_ULTRASONIC_FAIL         0x08U   /* 超声波故障 (ECU3 专用) */
#define ERROR_MOTOR_FAULT             0x10U   /* 电机故障 (ECU2 专用，原 ERROR_MOTOR_OVERHEAT) */
#define ERROR_SENSOR_FUSION           0x20U   /* 融合异常 (ECU3 专用) */
#define ERROR_OVERCURRENT             0x40U   /* 过流 (ECU2 专用) */
#define ERROR_WATCHDOG                0x80U   /* 看门狗复位 */

//系统状态定义
#define STATUS_NORMAL                 0x00U
#define STATUS_WARNING                0x01U
#define STATUS_ERROR                  0x02U
#define STATUS_CRITICAL               0x03U

/* 系统控制命令：所有 ECU 必须使用同一组值 */
#define SYS_CMD_START       1U
#define SYS_CMD_STOP        2U
#define SYS_CMD_RESET       3U
#define SYS_CMD_CALIBRATE   4U
#define SYS_CMD_DIAGNOSTIC  5U
#define SYS_CMD_SELF_TEST   6U
#define SYS_CMD_MANUAL      7U
#define SYS_CMD_AUTO        8U

#define CAN_TARGET_BROADCAST 0U

#endif
