#include "stm32f10x.h"                  // Device header
#include "ecu1_main.h"
#include "System_Config.h"
#include "Serial.h"
#include "MyCAN.h"
#include "Timer.h"
#include "Delay.h"
#include "OLED.h"
#include "OLED2.h"
#include "Key.h"
#include "LED.h"
#include <string.h>

// 全局变量
System_State system_state = SYSTEM_IDLE;
uint8_t selected_menu = 0;
uint32_t last_heartbeat_time = 0;
uint32_t last_display_time = 0;
uint32_t last_can_send_time = 0;

// CAN接收数据
SpeedCmd_Data speed_cmd = {0};
MotorStatus_Data motor_status = {0};
Sensor_Data sensor_data = {0};
uint8_t heartbeat_status[NODE_NUM] = {0};
uint32_t heartbeat_time[NODE_NUM] = {0};

// 菜单项
const char *menu_items[] = {
    "1.Motor Ctrl",                      //远程控制电机
    "2.Sensor View",                     //显示传感器数据
    "3.System Mon",                      //系统状态监控
    "4.CAN Test",                        //CAN通信测试
    "5.Settings"                         //参数设置
};

#define MENU_COUNT (sizeof(menu_items) / sizeof(menu_items[0]))

/** 函  数：系统初始化
  * 参  数：无
  * 返回值：无
  */
void System_Init(void)
{
    // 初始化外设
    LED_Init();
    Key_Init();
    OLED_Init();
	OLED2_Init();
	Timer_Init();
    Serial_Init(DEBUG_BAUDRATE);
    MyCAN_Init(CAN_BAUDRATE);
    
    printf("\r\n");
    printf("================================\r\n");
    printf("  ECU1 - Body Control Unit\r\n");
    printf("  Version: 1.0.0\r\n");
    printf("  Build Date: %s %s\r\n", __DATE__, __TIME__);
    printf("================================\r\n\r\n");
    
    // 系统启动指示
    LED1_ON();
    Delay_ms(500);
    LED2_ON();
    Delay_ms(500);
    LED3_ON();
    Delay_ms(500);
    
    LED1_OFF();
    LED2_OFF();
    LED3_OFF();
    
    // 显示启动界面
    OLED_Clear();
    OLED_ShowString(1, 1, "Vehicle Control");
    OLED_ShowString(2, 1, "ECU1-BodyControl");
    OLED_ShowString(4, 1, "Initializing...");
    
    Delay_ms(2000);
    
    system_state = SYSTEM_READY;
    
    printf("System initialized successfully!\r\n");
}

/** 函  数：主菜单显示
  * 参  数：无
  * 返回值：无
  */
void Show_MainMenu(void)
{
    OLED_Clear();
	OLED2_Clear();
    OLED2_ShowString(1, 1, "====Main Menu===");
    
    for(uint8_t i = 0; i < MENU_COUNT - 1; i++)
	{
        if(i == selected_menu)
		{
			OLED_ShowString(1 + i, 1, ">");
        }
        OLED_ShowString(1 + i , 2, (char*)menu_items[i]);
    }
	//由于屏幕空间不够，第五项放到第二个oled上显示
	if(selected_menu == MENU_COUNT - 1)
	{
		OLED2_ShowString(2, 1, ">");
	}
	OLED2_ShowString(2 , 2, (char*)menu_items[MENU_COUNT - 1]);
}

/** 函  数：按键处理
  * 参  数：无
  * 返回值：无
  */
void Key_Handler(void)
{

	uint8_t key_value = Key_Check(KEY_SINGLE);
    
    if(key_value)
	{
		
        switch(key_value)
		{
            case KEY1_PRESS:  // 上
                if(selected_menu > 0)
				{
                    selected_menu--;
                }
				else
				{
                    selected_menu = MENU_COUNT - 1;
                }
                Show_MainMenu();
                break;
                
            case KEY2_PRESS:  // 下
                selected_menu = (selected_menu + 1) % MENU_COUNT;
                Show_MainMenu();
                break;
                
            case KEY3_PRESS:  // 确定
                Execute_Menu(selected_menu);
                break;
                
            case KEY4_PRESS:  // 返回/取消
                system_state = SYSTEM_READY;
                Show_MainMenu();
                break;
        }
    }
}

/** 函  数：执行菜单功能
  * 参  数：当前所选菜单索引
  * 返回值：无
  */
void Execute_Menu(uint8_t menu_index)
{
    switch(menu_index)
	{
        case 0:  // 远程控制电机
            Remote_Control_Mode();
            break;
            
        case 1:  // 显示传感器数据
            Sensor_Display_Mode();
            break;
            
        case 2:  // 系统状态监控
            System_Monitor_Mode();
            break;
            
        case 3:  // CAN通信测试
            CAN_Test_Mode();
            break;
            
        case 4:  // 参数设置
            Parameter_Setting_Mode();
            break;
    }
}

/** 函  数：远程控制电机模式
  * 参  数：无
  * 返回值：无
  */
void Remote_Control_Mode(void)
{
    uint8_t speed = 0;
    uint8_t direction = 1;  // 1=正转
    
    system_state = REMOTE_CONTROL;
    
    while(system_state == REMOTE_CONTROL)
	{
        // 处理按键
        uint8_t key = Key_Check(KEY_SINGLE);
        if(key == KEY4_PRESS)
		{
            system_state = SYSTEM_READY;
            break;
        }
        
        // 按键控制速度
        if(key == KEY1_PRESS)
		{   // 加速
            if(speed < 100) speed += 10;
        }
		else if(key == KEY2_PRESS)
		{   // 减速
            if(speed > 0) speed -= 10;
        }
		else if(key == KEY3_PRESS)
		{   // 切换方向
            direction = (direction == 1) ? 2 : 1;
        }
        
        // 发送速度指令
        if(HAL_GetTick() - last_can_send_time > 100)
		{
            last_can_send_time = HAL_GetTick();
            MyCAN_Send_SpeedCmd(speed * 10, direction, 50);
            
            printf("Send SpeedCmd: %d RPM, Direction: %s\r\n", 
                   speed * 10, (direction == 1) ? "Forward" : "Reverse");
        }
        
        // 显示控制界面
        OLED_Clear();
		OLED2_Clear();
        OLED2_ShowString(1, 1, "RemoteControlMod");
        OLED_ShowString(1, 1, "Speed:");
        OLED_ShowNum(1, 7, speed, 3);
        OLED_ShowString(1, 10, "%");
        
        OLED_ShowString(2, 1, "Dir:");
        if(direction == 1)
		{
            OLED_ShowString(2, 5, "Forward");
        }
		else
		{
            OLED_ShowString(2, 5, "Reverse");
        }
        
        OLED2_ShowString(2, 1, "Key1:V+");
        OLED2_ShowString(2, 9, "Key2:V-");
        OLED2_ShowString(3, 1, "Key3:Change Dir");
        OLED2_ShowString(4, 1, "Key4:Back");
        
        Delay_ms(50);
    }
}

/** 函  数：传感器数据显示模式
  * 参  数：无
  * 返回值：无
  */
void Sensor_Display_Mode(void)
{
    system_state = SENSOR_DISPLAY;
    
    while(system_state == SENSOR_DISPLAY)
	{
        // 处理按键
        if(Key_Check(KEY_SINGLE) == KEY4_PRESS)
		{
            system_state = SYSTEM_READY;
            break;
        }
        
        // 显示传感器数据
        OLED_Clear();
		OLED2_Clear();
        OLED2_ShowString(1, 1, "Sensor Data");
        
        OLED_ShowString(1, 1, "Distance:");
        OLED_ShowNum(1, 10, sensor_data.distance, 4);
        OLED_ShowString(1, 14, "mm");
        
        OLED_ShowString(2, 1, "Pitch:");
        OLED_ShowSignedNum(2, 7, sensor_data.pitch, 4);
        OLED_ShowString(2, 11, "deg");
        
        OLED_ShowString(3, 1, "Roll:");
        OLED_ShowSignedNum(3, 6, sensor_data.roll, 4);
        OLED_ShowString(3, 10, "deg");
        
        OLED_ShowString(4, 1, "Yaw:");
        OLED_ShowSignedNum(4, 5, sensor_data.yaw, 4);
        OLED_ShowString(4, 9, "deg");
        
        OLED2_ShowString(2, 1, "Voltage:");
        OLED2_ShowNum(2, 9, sensor_data.voltage, 4);
        OLED2_ShowString(2, 13, "mV");
        
        OLED2_ShowString(4, 1, "Key4: Back");
        
        Delay_ms(100);
    }
}

/** 函  数：系统状态监控模式
  * 参  数：无
  * 返回值：无
  */
void System_Monitor_Mode(void)
{
    system_state = SYSTEM_MONITOR;
    
    while(system_state == SYSTEM_MONITOR)
	{
        // 处理按键
        if(Key_Check(KEY_SINGLE) == KEY4_PRESS)
		{
            system_state = SYSTEM_READY;
            break;
        }
        
        // 检查心跳状态
        uint32_t current_time = HAL_GetTick();
        for(uint8_t i = 0; i < NODE_NUM; i++)
		{
            if(current_time - heartbeat_time[i] > HEARTBEAT_TIMEOUT)
			{
                heartbeat_status[i] = 0;  // 离线
            }
			else
			{
                heartbeat_status[i] = 1;  // 在线
            }
        }
        
        // 显示系统状态
        OLED_Clear();
		OLED2_Clear();
        OLED2_ShowString(1, 1, "System Monitor");
        
        OLED2_ShowString(2, 1, "ECU1:");
        OLED2_ShowString(2, 6, heartbeat_status[0] ? "Online" : "Offline");
        
        OLED2_ShowString(3, 1, "ECU2:");
        OLED2_ShowString(3, 6, heartbeat_status[1] ? "Online" : "Offline");
        if(heartbeat_status[1])
		{
            OLED2_ShowString(4, 3, "Speed:");
            OLED2_ShowNum(4, 9, motor_status.actual_speed, 4);
        }
        
        OLED_ShowString(1, 1, "ECU3:");
        OLED_ShowString(1, 6, heartbeat_status[2] ? "Online" : "Offline");
        
        OLED_ShowString(2, 3, "MotorTemp:");
        OLED_ShowNum(2, 13, motor_status.temperature, 3);
        OLED_ShowString(2, 16, "C");
        
        OLED_ShowString(3, 3, "Motor_I:");
        OLED_ShowNum(3, 11, motor_status.current, 4);
        OLED_ShowString(3, 15, "mA");
        
        OLED_ShowString(4, 3, "MotorStatus:");
        OLED_ShowHexNum(4, 15, motor_status.status, 2);
        
        //OLED2_ShowString(4, 1, "Key4: Back");
        
        Delay_ms(100);
    }
}

/** 函  数：CAN通信测试模式
  * 参  数：无
  * 返回值：无
  */
void CAN_Test_Mode(void)
{
    static uint8_t test_counter = 0;
    system_state = CAN_TEST;
    
    while(system_state == CAN_TEST)
	{
        // 处理按键
        uint8_t key = Key_Check(KEY_SINGLE);
        if(key == KEY4_PRESS)
		{
            system_state = SYSTEM_READY;
            break;
        }
        
        // 发送测试数据
        if(key == KEY3_PRESS)
		{
            test_counter++;
            uint8_t test_data[8] = {test_counter, 0xAA, 0x55, 0x01, 0x02, 0x03, 0x04, 0x05};
            
            if(MyCAN_Send_Message(0x123, 8, test_data))
			{
                printf("Test data sent: %d\r\n", test_counter);
            }
			else
			{
                printf("Failed to send test data\r\n");
            }
        }
        
        // 显示CAN测试界面
        OLED_Clear();
		OLED2_Clear();
        OLED2_ShowString(1, 1, "CAN Test Mode");
        OLED_ShowString(1, 1, "Test Counter:");
        OLED_ShowNum(1, 14, test_counter, 3);
        
        OLED2_ShowString(3, 1, "Key3: Send Test");
        OLED2_ShowString(4, 1, "Key4: Back");
        
        // 显示接收到的CAN报文
        uint32_t can_id;
        uint8_t can_data[8];
        uint8_t can_len;
        
        if(MyCAN_Receive_Message(&can_id, can_data, &can_len))
		{
            OLED_ShowString(2, 1, "RX ID:");
            OLED_ShowHexNum(2, 7, can_id, 3);
            
            OLED_ShowString(3, 1, "Data:");
            for(uint8_t i = 0; i < can_len && i < 8; i++)
			{
                OLED_ShowHexNum( 3, 6 + i, can_data[i], 8);
            }
        }
        
        Delay_ms(50);
    }
}

/** 函  数：参数设置模式
  * 参  数：无
  * 返回值：无
  */
void Parameter_Setting_Mode(void)
{
    system_state = PARAM_SETTING;
    
    // 参数默认值
    static uint8_t can_baudrate = 2;  // 0=125k, 1=250k, 2=500k, 3=1M
    static uint8_t heartbeat_period = 1;  // 秒
    
    uint8_t selected_param = 0;
    const char *baudrate_names[] = {"125k", "250k", "500k", "1M"};
    
    while(system_state == PARAM_SETTING)
	{
        uint8_t key = Key_Check(KEY_SINGLE);
        
        switch(key)
		{
            case KEY1_PRESS:  // 上
                if(selected_param > 0) selected_param--;
                break;
                
            case KEY2_PRESS:  // 下
                if(selected_param < 1) selected_param++;
                break;
                
            case KEY3_PRESS:  // 修改参数
                if(selected_param == 0)
				{
                    can_baudrate = (can_baudrate + 1) % 4;
                }
				else if(selected_param == 1)
				{
                    heartbeat_period = (heartbeat_period % 10) + 1;
                }
                break;
                
            case KEY4_PRESS:  // 保存并退出
                // 保存参数到EEPROM（这里简化处理）
                printf("Parameters saved: Baudrate=%s, Heartbeat=%ds\r\n", 
                       baudrate_names[can_baudrate], heartbeat_period);
                system_state = SYSTEM_READY;
                return;
        }
        
        // 显示参数设置界面
        OLED_Clear();
		OLED2_Clear();
        OLED2_ShowString(1, 1, "ParameterSetting");
        
        // CAN波特率
        OLED_ShowString(1, 2, "CANBaudrate:");
        if(selected_param == 0) OLED_ShowString(1, 1, ">");
		
        OLED_ShowString(1, 13, (char*)baudrate_names[can_baudrate]);
        
        // 心跳周期
        OLED_ShowString(2, 2, "Heart Period:");
        if(selected_param == 1) OLED_ShowString(2, 1, ">");
		
        OLED_ShowNum(3, 4, heartbeat_period, 2);
        OLED_ShowString(3, 6, "s");
        
        OLED2_ShowString(2, 1, "Key1/2: Select");
        OLED2_ShowString(3, 1, "Key3: Change");
        OLED2_ShowString(4, 1, "Key4: Save&Back");
		
        Delay_ms(50);
    }
}

/** 函  数：心跳包管理
  * 参  数：无
  * 返回值：无
  */
void Heartbeat_Manager(void)
{
    static uint32_t last_heartbeat_send = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 发送心跳包
    if(current_time - last_heartbeat_send > HEARTBEAT_PERIOD)
	{
        last_heartbeat_send = current_time;
        MyCAN_Send_Heartbeat(NODE_ID, STATUS_NORMAL, ERROR_NONE, current_time / 1000);
        
        // LED指示
        LED1_Turn();
    }
    
    // 检查其他节点心跳
    static uint32_t last_check_time = 0;
    if(current_time - last_check_time > 1000)
	{  // 每秒检查一次
        last_check_time = current_time;
        
        for(uint8_t i = 0; i < NODE_NUM; i++)
		{
            if(i == NODE_ID - 1) continue;  // 跳过自己
            
            if(current_time - heartbeat_time[i] > HEARTBEAT_TIMEOUT)
			{
                printf("Warning: Node %d is offline!\r\n", i + 1);
                
                // LED报警
                if(i == 1) LED2_ON();  // ECU2离线
                if(i == 2) LED3_ON();  // ECU3离线
            }
			else
			{
                if(i == 1) LED2_OFF();
                if(i == 2) LED3_OFF();
            }
        }
    }
}

/** 函  数：CAN数据处理
  * 参  数：id 要处理的数据的ID，len 要处理的数据的长度，data 要处理的数据
  * 返回值：无
  */
void CAN_Data_Handler(uint32_t id, uint8_t len, uint8_t* data)
{
    switch(id)
	{
        case MSG_ID_HEARTBEAT:
		{
            HeartBeat_Data *hb = (HeartBeat_Data*)data;
            if(hb->node_id >= 1 && hb->node_id <= NODE_NUM)
			{
                heartbeat_time[hb->node_id - 1] = HAL_GetTick();
            }
            break;
        }
            
        case MSG_ID_MOTOR_STATUS:
		{
            memcpy(&motor_status, data, sizeof(MotorStatus_Data));
            
            // 如果电机状态异常，报警
            if(motor_status.status != 0)
			{
                printf("Motor Error: Status=0x%02X\r\n", motor_status.status);
                //LED4_ON();
            }
			else
			{
                //LED4_OFF();
            }
            break;
        }
            
        case MSG_ID_SENSOR_DATA:
		{
            memcpy(&sensor_data, data, sizeof(Sensor_Data));
            
            // 如果距离过近，报警
            if(sensor_data.distance < 200)
			{  // 20cm
                printf("Warning: Distance too close! %dmm\r\n", sensor_data.distance);
            }
            break;
        }
		
		case MSG_ID_ERROR_REPORT:
		{
			ErrorReport_Data *err = (ErrorReport_Data*)data;
			//打印错误报告
			printf("Error Report from Node %d: type=0x%02X, code=0x%04X, time=%u\r\n",
					err->node_id, err->error_type, err->error_code, err->timestamp);
			
			switch(err->error_type)
			{
				case ERROR_NONE:
					printf("No Error on node %d\r\n",err->node_id);
					break;
				case ERROR_CAN_COMM:
					printf("CAN communication error on node %d\r\n",err->node_id);
					break;
				case ERROR_MOTOR_OVERHEAT:
					printf("Motor overheat on node %d\r\n",err->node_id);
					break;
				case ERROR_SENSOR_FAIL:
					printf("Sensor failure on node %d\r\n",err->node_id);
					break;
				case ERROR_VOLTAGE_LOW:
					printf("Voltage low on node %d\r\n",err->node_id);
					break;
				case ERROR_ENCODER_FAIL:
					printf("Encoder failure on node %d\r\n",err->node_id);
					break;
				default:
					printf("Unknown error type\r\n");
					break;
					
			}
		}
            
        default:
		{	// 未知报文处理
			printf("Unknown CAN message: ID=0x%03x, Len=%d, Data=",id,len);
			for(uint8_t i = 0;i < len;i ++)
			{
				printf("%02x",data[i]);
			}
			printf("\r\n");
			break;
		}
    }
}



int main(void) 
{
	System_Init();
	
    Show_MainMenu();
	
	printf("System started. Press keys to navigate menu.\r\n");
	
	
	
    while(1) 
	{
		Key_Handler();
        
    }
}

/** 函  数：错误处理
  * 参  数：id 要处理的数据
  * 返回值：无
  */
void Error_Handler(void)
{
    // LED快速闪烁表示错误
    static uint32_t last_blink = 0;
    
    if(HAL_GetTick() - last_blink > 100)
	{
        last_blink = HAL_GetTick();
        LED1_Turn();
        LED2_Turn();
        LED3_Turn();
        //LED4_Turn();
    }
    
    // 尝试恢复
    static uint32_t error_start = 0;
    if(error_start == 0)
	{
        error_start = HAL_GetTick();
    }
    
    // 5秒后尝试重启
    if(HAL_GetTick() - error_start > 5000)
	{
        printf("System reset after error...\r\n");
        NVIC_SystemReset();
    }
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		Key_Scan();
		
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
