#include "stm32f10x.h"                  // Device header
#include "MyCAN.h"
#include "Delay.h"
#include <stddef.h>

#define CAN_STD_ID_MAX      0x7FFU
#define CAN_EXT_ID_MAX      0x1FFFFFFFU
#define CAN_MAX_DLC         8U

CanTxMsg TxMessage;
static CAN_RxBuffer_t can_rx_buf = {0};
volatile uint8_t MyCAN_RxFlag = 0;

void MyCAN_Init(CAN_BaudRate baudrate)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	//配置Rx引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//配置Tx引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//使能CAN接收中断
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//CAN参数配置
	CAN_InitTypeDef CAN_InitStructure;
	
	//设置波特率
	switch(baudrate)
	{
		case CAN_BAUD_125K:
			CAN_InitStructure.CAN_Prescaler = 24;		//波特率 = 36M / 24 / (1 + 9 + 2) = 125K
			CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
			CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
			CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;
			break;
		case CAN_BAUD_250K:
			CAN_InitStructure.CAN_Prescaler = 12;		//波特率 = 36M / 12 / (1 + 9 + 2) = 250K
			CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
			CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
			CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;
			break;
		case CAN_BAUD_500K:
			CAN_InitStructure.CAN_Prescaler = 6;		//波特率 = 36M / 6 / (1 + 9 + 2) = 500K
			CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
			CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
			CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;
			break;
		case CAN_BAUD_1M:
			CAN_InitStructure.CAN_Prescaler = 3;		//波特率 = 36M / 3 / (1 + 9 + 2) = 1M
			CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
			CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
			CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;
			break;
		default:
			CAN_InitStructure.CAN_Prescaler = 6;
			CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
			CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
			CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;
			break;
	}
	
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_NART = DISABLE;           //非自动重传
	CAN_InitStructure.CAN_TXFP = DISABLE;           //发送优先级由ID决定
	CAN_InitStructure.CAN_RFLM = DISABLE;           //不锁定FIFO
	CAN_InitStructure.CAN_AWUM = ENABLE;            //自动唤醒
	CAN_InitStructure.CAN_TTCM = DISABLE;           //关闭时间触发通信功能
	CAN_InitStructure.CAN_ABOM = ENABLE;            //自动离线管理
	CAN_Init(CAN1, &CAN_InitStructure);
	
	//配置过滤器
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
}

/**
  * 函  数：发送CAN报文
  * 参  数：ID 向此ID设备发送数据
  * 参  数；Len 发送数据的长度
  * 参  数：Data 要发送的数据
  * 返回值：无
  */
uint8_t MyCAN_Send_Message(uint32_t ID,uint8_t Len,uint8_t* Data)
{
	uint8_t mailbox;
	
	//参数校验
    if ((Data == NULL) || (Len > CAN_MAX_DLC) || (ID > CAN_STD_ID_MAX))
    {
        return 0;
    }
	
	TxMessage.StdId = ID;
	TxMessage.ExtId = 0x00;
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;
	TxMessage.DLC = Len;
	for(uint8_t i = 0;i < Len;i++)
	{
		TxMessage.Data[i] = Data[i];
	}
	
	for(uint8_t i = Len;i < 8;i++)
	{
		TxMessage.Data[i] = 0;
	}
	
	mailbox = CAN_Transmit(CAN1,&TxMessage);
	
	if(mailbox == CAN_TxStatus_NoMailBox)
	{
        return 0;  // 发送失败
    }
	
	uint32_t start_tick = HAL_GetTick();
	while(CAN_TransmitStatus(CAN1,mailbox) != CAN_TxStatus_Ok)
	{
		if ((HAL_GetTick() - start_tick) > 10U)
		{  /* 10ms 超时 */
			CAN_CancelTransmit(CAN1, mailbox);
			return 0;
		}
	}
	return 1;
}

/** 函  数：接收CAN报文
  * 参  数：ID 用来接收总线上发送的ID的指针
  * 参  数：Len 用来接收总线上发送的数据长度的指针
  * 参  数：Data 用来接收总线上发送的数据的指针
  * 返回值：1 接收成功，0 没有新报文
  */
uint8_t MyCAN_Receive_Message(uint32_t* ID, uint8_t* Len, uint8_t* Data)
{
    uint8_t i;
    
    if ((ID == NULL) || (Len == NULL) || (Data == NULL))
    {
        return 0;
    }
    // 临界区：防止 ISR 在读取过程中修改
    __disable_irq();
	
    if (can_rx_buf.count == 0)
    {
        return 0;
    }
    
    *ID = can_rx_buf.msg[can_rx_buf.tail].StdId;
    *Len = can_rx_buf.msg[can_rx_buf.tail].DLC;
    for (i = 0; i < *Len; i++)
    {
        Data[i] = can_rx_buf.msg[can_rx_buf.tail].Data[i];
    }
    can_rx_buf.tail = (can_rx_buf.tail + 1) % CAN_RX_BUF_SIZE;
    can_rx_buf.count--;
    __enable_irq();
    
    return 1;
}

/**函  数：发送心跳包
  *参  数：node_id 发送设备的节点ID
  *参  数：status 发送设备的状态
  *参  数：error_code 发送设备的错误代码
  *参  数：uptime 发送设备的运行时间
  *返回值：无
  */
void MyCAN_Send_Heartbeat(uint8_t node_id, uint8_t status, uint8_t error_code, uint32_t uptime)
{
    HeartBeat_Data heartbeat;
    heartbeat.node_id = node_id;
    heartbeat.status = status;
    heartbeat.error_code = error_code;
    heartbeat.uptime = uptime;
    
    MyCAN_Send_Message(MSG_ID_HEARTBEAT, sizeof(HeartBeat_Data), (uint8_t*)&heartbeat);
}

/**函  数：发送速度指令
  *参  数：speed 需要的目标速度
  *参  数：direction 需要的旋转方向
  *参  数：acceleration 需要的加速度
  *返回值：无
  */
void MyCAN_Send_SpeedCmd(int16_t speed, uint8_t direction, uint8_t acceleration)
{
    SpeedCmd_Data cmd;
    cmd.target_speed = speed;
    cmd.direction = direction;
    cmd.acceleration = acceleration;
    
    MyCAN_Send_Message(MSG_ID_SPEED_CMD, sizeof(SpeedCmd_Data), (uint8_t*)&cmd);
}

/**函  数：发送电机状态
  *参  数：speed 的实际电机转速
  *参  数：current 电机电流
  *参  数：temp 电机温度
  *参  数：status 状态位
  *返回值：无
  */
void MyCAN_Send_MotorStatus(int16_t speed, uint16_t current, /*uint8_t temp,*/ uint8_t status)
{
    MotorStatus_Data motor;
    motor.actual_speed = speed;
    motor.current = current;
    //motor.temperature = temp;
    motor.status = status;
    
    MyCAN_Send_Message(MSG_ID_MOTOR_STATUS, sizeof(MotorStatus_Data), (uint8_t*)&motor);
}

/**函  数：发送传感器数据
  *参  数：distance 要发送的距离
  *参  数：pitch 要发送的俯仰角数据
  *参  数：roll 要发送的横滚角数据
  *参  数：voltage 要发送的电压数据
  *返回值：无
  */
void MyCAN_Send_SensorData(uint16_t distance, int16_t pitch, int16_t roll, int16_t yaw, uint16_t voltage)
{
    Sensor_Data sensor;
    sensor.distance = distance;
    sensor.pitch = pitch;
    sensor.roll = roll;
    sensor.yaw_high = (uint8_t)(yaw >> 8);
    sensor.yaw_low = (uint8_t)(yaw & 0xFF);
    sensor.voltage = voltage;
    
    MyCAN_Send_Message(MSG_ID_SENSOR_DATA, sizeof(Sensor_Data), (uint8_t*)&sensor);
}

/**函  数：发送系统控制命令
  *参  数：command 要发送的命令
  *参  数：param1 参数1
  *参  数：param2 参数2
  *返回值：无
  */
void MyCAN_Send_SystemCtrl(uint8_t command, uint8_t param1, uint8_t param2)
{
    SystemCtrl_Data ctrl;
    ctrl.command = command;
    ctrl.param1 = param1;
    ctrl.param2 = param2;
    
    MyCAN_Send_Message(MSG_ID_SYSTEM_CTRL, sizeof(SystemCtrl_Data), (uint8_t*)&ctrl); 
}

/**函  数：发送错误报告
  *参  数：node_id 发送设备的节点ID
  *参  数：error_type 产生的错误类型
  *参  数：error_code 产生的错误代码
  *返回值：无
  */
void MyCAN_Send_ErrorReport(uint8_t node_id, uint8_t error_type, uint16_t error_code)
{
    ErrorReport_Data error;
    error.node_id = node_id;
    error.error_type = error_type;
    error.error_code = error_code;
    error.timestamp = HAL_GetTick();
    
    MyCAN_Send_Message(MSG_ID_ERROR_REPORT, sizeof(ErrorReport_Data), (uint8_t*)&error);
}

/** 函  数：CAN接收中断服务函数
  * 参  数：无
  * 返回值：无
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	// 读取 FIFO 中所有挂起的报文（最多 3 帧）
    while (CAN_GetFlagStatus(CAN1, CAN_FLAG_FMP0) != RESET)
    {
        if (can_rx_buf.count < CAN_RX_BUF_SIZE)
        {
            CAN_Receive(CAN1, CAN_FIFO0, 
                       &can_rx_buf.msg[can_rx_buf.head]);
            can_rx_buf.head = (can_rx_buf.head + 1) % CAN_RX_BUF_SIZE;
            can_rx_buf.count++;
        }
        else
        {
            // 缓冲区满，丢弃最旧报文
            can_rx_buf.tail = (can_rx_buf.tail + 1) % CAN_RX_BUF_SIZE;
            CAN_Receive(CAN1, CAN_FIFO0, 
                       &can_rx_buf.msg[can_rx_buf.head]);
            can_rx_buf.head = (can_rx_buf.head + 1) % CAN_RX_BUF_SIZE;
            can_rx_buf.overflow++;
        }
    }
}
