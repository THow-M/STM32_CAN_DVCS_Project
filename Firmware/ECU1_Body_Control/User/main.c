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
    //"5.Settings"                         //参数设置
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
    
    for(uint8_t i = 0; i < MENU_COUNT; i++)
	{
        if(i == selected_menu)
		{
            OLED_ShowString(1 + i, 1, ">");
        }
        OLED_ShowString(1 + i , 2, (char*)menu_items[i]);
    }
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
            //CAN_Test_Mode();
            break;
            
        case 4:  // 参数设置
            //Parameter_Setting_Mode();
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
        
        OLED2_ShowString(1, 1, "Voltage:");
        OLED2_ShowNum(1, 9, sensor_data.voltage, 4);
        OLED2_ShowString(1, 13, "mV");
        
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

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		Key_Scan();
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
