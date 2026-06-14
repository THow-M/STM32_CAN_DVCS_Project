#include "stm32f10x.h"                  // Device header
#include "ecu1_main.h"
#include "System_Config.h"
#include "Serial.h"
#include "MyCAN.h"
#include "Timer.h"
#include "Delay.h"
#include "OLED.h"
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

//系统初始化
void System_Init(void)
{
    // 初始化外设
    LED_Init();
    Key_Init();
    OLED_Init();
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

// 主菜单显示
void Show_MainMenu(void)
{
    OLED_Clear();
    OLED_ShowString(1, 1, "=== Main Menu ==");
	Delay_ms(500);
	OLED_Clear();
    
    for(uint8_t i = 0; i < MENU_COUNT; i++)
	{
        if(i == selected_menu)
		{
            OLED_ShowString(1 + i, 1, ">");
        }
        OLED_ShowString(1 + i , 2, (char*)menu_items[i]);
    }
}

// 按键处理
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
                //Execute_Menu(selected_menu);
                break;
                
            case KEY4_PRESS:  // 返回/取消
                system_state = SYSTEM_READY;
                Show_MainMenu();
                break;
        }
    }
}

/*// 执行菜单功能
void Execute_Menu(uint8_t menu_index) {
    switch(menu_index) {
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
*/

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
