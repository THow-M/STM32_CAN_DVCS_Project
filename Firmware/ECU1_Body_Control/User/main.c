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
#include <stdlib.h>

#define IWDG_TIMEOUT_RELOAD      ((uint16_t)0x0FFFU)
#define IWDG_TIMEOUT_MS          (6552U)
#define DIRECTION_SWITCH_TIMEOUT_MS  2000U  /* 方向切换超时 2 秒 */

// 全局变量
System_State system_state = SYSTEM_IDLE;
uint8_t selected_menu = 0;
uint32_t last_heartbeat_time = 0;
uint32_t last_display_time = 0;
uint32_t last_can_send_time = 0;
static volatile uint8_t Key_Scan_Flag = 0U;

/* CAN_Test_Mode 修复 */
static uint32_t g_last_rx_id = 0;
static uint8_t g_last_rx_data[8] = {0};
static uint8_t g_last_rx_len = 0;
static uint8_t g_rx_updated = 0;

static uint32_t g_heartbeat_period_ms = HEARTBEAT_PERIOD;

/* ---- 错误处理静态变量 ---- */
static ErrorReport_Data s_last_error = {0};    /* 最近一次收到的错误报告 */
static uint8_t s_error_pending = 0U;           /* 是否有待处理的错误 */

// CAN接收数据
SpeedCmd_Data speed_cmd = {0};
MotorStatus_Data motor_status = {0};
Sensor_Data sensor_data = {0};
uint8_t heartbeat_status[NODE_NUM] = {0};
uint32_t heartbeat_time[NODE_NUM] = {0};
uint16_t sensor_voltage_mv = 0U;

/* 三 ECU 心跳监控表。
 * 丢失阈值 = 3× 周期（默认 3×1s=3s）；推荐 3× 而非 5× 的原因是电机运行中断 CAN 3s 已经是安全上限。 */
#define HEARTBEAT_NODE_COUNT        3U   // 0=ECU1(自身) 1=ECU2 2=ECU3
#define HEARTBEAT_TIMEOUT_MULT      3U

typedef struct {
    uint32_t last_seen_ms;            /* 最近一次收到该节点心跳的 HAL_GetTick() */
    uint8_t  alive;                   /* 0=失联 1=在网 */
    uint8_t  prev_status;             /* 上次报的 status 字节 (0正常/1告警/2错误) */
} HBM_Monitor_t;

static HBM_Monitor_t hbm_nodes[HEARTBEAT_NODE_COUNT];

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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
    // 初始化外设
	Key_Init();
	Timer_Init();
    LED_Init();
    OLED_Init();
	OLED2_Init();
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
    OLED_ShowString(0, 0, "Vehicle Control", OLED_8X16);
    OLED_ShowString(0, 16, "ECU1-BodyControl", OLED_8X16);
    OLED_ShowString(0, 42, "Initializing...", OLED_8X16);
    OLED_Update();
	
    Delay_ms(2000);
    
    system_state = SYSTEM_READY;
	
	uint32_t boot_time = HAL_GetTick();
    for (uint8_t i = 0; i < NODE_NUM; i++)
    {
        heartbeat_time[i] = boot_time;  /* 给予所有节点启动宽限 */
    }
	
	/* 看门狗 */
	/* IWDG 参数宏 & 计算注释
	 * Tout(ms) = Reload × Prescaler / LSI_freq
	 *          = 4095 × 64 / 40000 = 6.552 s ≈ 6552 ms
	 * LSI 范围 30kHz~60kHz；最不利 (LSI=60kHz) Tout = 4095*64/60k ≈ 4.36s，仍 > 主循环喂狗间隔 (≤ 50ms×N)
	 */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64);   /* 40kHz/64 = 625Hz */
    IWDG_SetReload(IWDG_TIMEOUT_RELOAD);
    IWDG_ReloadCounter();
    IWDG_Enable();
    
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
    OLED2_ShowString(0, 0, "====Main Menu===", OLED_8X16);
    
    for(uint8_t i = 0; i < MENU_COUNT - 1; i++)
	{
        if(i == selected_menu)
		{
			OLED_ShowString(0, 16*i, ">" ,OLED_8X16);
        }
        OLED_ShowString(8, 16*i, (char*)menu_items[i] ,OLED_8X16);
    }
	//由于屏幕空间不够，第五项放到第二个oled上显示
	if(selected_menu == MENU_COUNT - 1)
	{
		OLED2_ShowString(0, 16, ">", OLED_8X16);
	}
	OLED2_ShowString(8 , 16, (char*)menu_items[MENU_COUNT - 1], OLED_8X16);
	
	OLED_Update();
	OLED2_Update();
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
	static uint8_t direction_switching = 0U;
	static uint8_t pending_direction = 0U;
	static uint32_t switch_start_time = 0U;
    
    system_state = REMOTE_CONTROL;
    
    while(system_state == REMOTE_CONTROL)
	{
		while (Key_Scan_Flag > 0U)
		{
			uint32_t primask = __get_PRIMASK();   /* 保存中断状态 */
			__disable_irq();
			Key_Scan_Flag--;
			if (primask == 0U) { __enable_irq(); }  /* 仅原状态为开才恢复 */
			Key_Scan();
		}
		
		HeartBeat_Manager();
		
		uint32_t can_id;
        uint8_t can_data[8];
        uint8_t can_len;
        uint8_t rx_count = 0U;
        while (MyCAN_Receive_Message(&can_id, &can_len, can_data) 
               && rx_count < 8U)
        {
            CAN_Data_Handler(can_id, can_len, can_data);
            rx_count++;
        }
		
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
		else if (key == KEY3_PRESS && !direction_switching)
		{
			direction_switching = 1U;
			pending_direction = (direction == 1) ? 2 : 1;
			switch_start_time = HAL_GetTick();
			/* 立即发送停止指令 */
			MyCAN_Send_SpeedCmd(0, 0, 100);
		}
		
		if (direction_switching)
		{
			if (abs(motor_status.actual_speed) <= 10)
			{
				direction = pending_direction;
				direction_switching = 0U;
			}
			else if (HAL_GetTick() - switch_start_time >= DIRECTION_SWITCH_TIMEOUT_MS)
			{
				/* 超时强制切换，避免永久卡住 */
				direction = pending_direction;
				direction_switching = 0U;
				printf("Direction switch timeout, forced\r\n");
			}
		}
        
        // 发送速度指令
        if ((HAL_GetTick() - last_can_send_time) >= 100U)
		{
			uint8_t cmd_direction;
			int16_t cmd_speed;
		
			last_can_send_time = HAL_GetTick();
		
			if (direction_switching)
			{
				cmd_speed = 0;
				cmd_direction = 0;   /* MOTOR_STOP */
			}
			else
			{
				cmd_speed = (int16_t)speed * 10;
				cmd_direction = direction;
			}
		
			(void)MyCAN_Send_SpeedCmd(cmd_speed, cmd_direction, 50U);
		}
        
        // 显示控制界面
        OLED_Clear();
		OLED2_Clear();
        OLED2_ShowString(0, 0, "RemoteControlMod", OLED_8X16);
        OLED_ShowString(0, 0, "Speed:", OLED_8X16);
        OLED_ShowNum(48, 0, speed, 3 ,OLED_8X16);
        OLED_ShowString(72, 0, "%", OLED_8X16);
        
        OLED_ShowString(0, 16, "Dir:", OLED_8X16);
        if(direction == 1)
		{
            OLED_ShowString(32, 16, "Forward", OLED_8X16);
        }
		else
		{
            OLED_ShowString(32, 16, "Reverse", OLED_8X16);
        }
        
        OLED2_ShowString(0, 16, "Key1:V+", OLED_8X16);
        OLED2_ShowString(64, 16, "Key2:V-", OLED_8X16);
        OLED2_ShowString(0, 32, "Key3:Change Dir", OLED_8X16);
        OLED2_ShowString(0, 48, "Key4:Back", OLED_8X16);
        
		OLED_Update();
		OLED2_Update();
		
        Delay_ms(50);
		
		IWDG_ReloadCounter();
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
		while (Key_Scan_Flag > 0U)
		{
			uint32_t primask = __get_PRIMASK();   /* 保存中断状态 */
			__disable_irq();
			Key_Scan_Flag--;
			if (primask == 0U) { __enable_irq(); }  /* 仅原状态为开才恢复 */
			Key_Scan();
		}
		
		HeartBeat_Manager();
		
		uint32_t can_id;
        uint8_t can_data[8];
        uint8_t can_len;
        uint8_t rx_count = 0U;
        while (MyCAN_Receive_Message(&can_id, &can_len, can_data) 
               && rx_count < 8U)
        {
            CAN_Data_Handler(can_id, can_len, can_data);
            rx_count++;
        }
		
        // 处理按键
        if(Key_Check(KEY_SINGLE) == KEY4_PRESS)
		{
            system_state = SYSTEM_READY;
            break;
        }
        
        // 显示传感器数据
        OLED_Clear();
		OLED2_Clear();
        OLED2_ShowString(0, 0, "Sensor Data", OLED_8X16);
        
        OLED_ShowString(0, 0, "Distance:", OLED_8X16);
        OLED_ShowNum(9*8, 0, sensor_data.distance, 4, OLED_8X16);
        OLED_ShowString(13*8 , 0, "mm", OLED_8X16);
        
		int16_t pitch_deg = sensor_data.pitch / 10;
		int16_t pitch_frac = sensor_data.pitch % 10;
        OLED_ShowString(0 , 16, "Pitch:", OLED_8X16);
		OLED_Printf(6*8, 16, OLED_8X16, "%d.%d", pitch_deg, abs(pitch_frac));
        //OLED_ShowSignedNum(6*8, 16, sensor_data.pitch, 4, OLED_8X16);
        OLED_ShowString(10*8, 16, "deg", OLED_8X16);
        
        OLED_ShowString(0, 32, "Roll:", OLED_8X16);
        OLED_ShowSignedNum(5*8, 32, sensor_data.roll, 4, OLED_8X16);
        OLED_ShowString(9*8, 32, "deg", OLED_8X16);
        
        int16_t yaw_deg = sensor_data.yaw / 10;
		int16_t yaw_frac = sensor_data.yaw % 10;

		OLED_ShowString(0, 48, "Yaw:", OLED_8X16);
		OLED_Printf(4 * 8, 48, OLED_8X16, "%d.%d",
					yaw_deg, abs(yaw_frac));
		OLED_ShowString(9 * 8, 48, "deg", OLED_8X16);
        
        OLED2_ShowString(0, 16, "Voltage:", OLED_8X16);
        OLED2_ShowNum(8*8, 16, sensor_voltage_mv, 4, OLED_8X16);
        OLED2_ShowString(12*8, 16, "mV", OLED_8X16);
        
        OLED2_ShowString(0, 48, "Key4: Back", OLED_8X16);
        
		OLED_Update();
		OLED2_Update();
		
        Delay_ms(100);
		
		IWDG_ReloadCounter();
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
		while (Key_Scan_Flag > 0U)
		{
			uint32_t primask = __get_PRIMASK();   /* 保存中断状态 */
			__disable_irq();
			Key_Scan_Flag--;
			if (primask == 0U) { __enable_irq(); }  /* 仅原状态为开才恢复 */
			Key_Scan();
		}
		
		HeartBeat_Manager();
		
		uint32_t can_id;
        uint8_t can_data[8];
        uint8_t can_len;
        uint8_t rx_count = 0U;
        while (MyCAN_Receive_Message(&can_id, &can_len, can_data) 
               && rx_count < 8U)
        {
            CAN_Data_Handler(can_id, can_len, can_data);
            rx_count++;
        }
		
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
        OLED2_ShowString(0, 0, "System Monitor", OLED_8X16);
        
        OLED_ShowString(0, 0, "ECU1:", OLED_6X8);
        OLED_ShowString(5*6, 0, heartbeat_status[0] ? "Online" : "Offline", OLED_6X8);
        
        OLED_ShowString(0, 8, "ECU2:", OLED_6X8);
        OLED_ShowString(5*6, 8, heartbeat_status[1] ? "Online" : "Offline", OLED_6X8);
        if(heartbeat_status[1])
		{
            OLED_ShowString(2*6, 16, "Speed:", OLED_6X8);
            OLED_ShowSignedNum(8*6, 16, motor_status.actual_speed, 4, OLED_6X8);
        }
        
        OLED_ShowString(0, 24, "ECU3:", OLED_6X8);
        OLED_ShowString(5*6, 24, heartbeat_status[2] ? "Online" : "Offline", OLED_6X8);
        
        //OLED_ShowString(2*6, 32, "MotorTemp:", OLED_6X8);
        //OLED_ShowNum(12*6, 32, motor_status.temperature, 3, OLED_6X8);
        //OLED_ShowString(15*6, 32, "C", OLED_6X8);
        
        OLED_ShowString(2*6, 40, "MotorCurrent:", OLED_6X8);
        OLED_ShowNum(15*6, 40, motor_status.current, 4, OLED_6X8);
        OLED_ShowString(19*6, 40, "mA", OLED_6X8);
        
        OLED_ShowString(2*6, 48, "MotorStatus:", OLED_6X8);
        OLED_ShowHexNum(14*6, 48, motor_status.status, 2, OLED_6X8);
        
        OLED2_ShowString(0, 48, "Key4: Back", OLED_8X16);
		
		OLED_Update();
		OLED2_Update();
        
        Delay_ms(100);
		
		IWDG_ReloadCounter();
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
		while (Key_Scan_Flag > 0U)
		{
			uint32_t primask = __get_PRIMASK();   /* 保存中断状态 */
			__disable_irq();
			Key_Scan_Flag--;
			if (primask == 0U) { __enable_irq(); }  /* 仅原状态为开才恢复 */
			Key_Scan();
		}
		
		HeartBeat_Manager();
		
		uint32_t can_id;
        uint8_t can_data[8];
        uint8_t can_len;
        uint8_t rx_count = 0U;
        while (MyCAN_Receive_Message(&can_id, &can_len, can_data) 
               && rx_count < 8U)
        {
            CAN_Data_Handler(can_id, can_len, can_data);
            rx_count++;
        }
		
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
        OLED2_ShowString(0, 0, "CAN Test Mode", OLED_8X16);
        OLED_ShowString(0, 0, "Test Counter:", OLED_8X16);
        OLED_ShowNum(13*8, 0, test_counter, 3, OLED_8X16);
        
        OLED2_ShowString(0, 32, "Key3: Send Test", OLED_8X16);
        OLED2_ShowString(0, 48, "Key4: Back", OLED_8X16);
        
        // 显示接收到的CAN报文
        //uint32_t can_id;
        //uint8_t can_data[8];
        //uint8_t can_len;
		
		OLED_ShowString(0, 16, "RX ID:", OLED_8X16);
		OLED_ShowString(0, 32, "Data:", OLED_8X16);
        
        /*if(MyCAN_Receive_Message(&can_id, &can_len, can_data))
		{
            //OLED_ShowString(0, 16, "RX ID:", OLED_8X16);
            OLED_ShowHexNum(6*8, 16, can_id, 3, OLED_8X16);
            
            //OLED_ShowString(0, 32, "Data:", OLED_8X16);
            for(uint8_t i = 0; i < can_len && i < 8; i++)
			{
                OLED_ShowHexNum( 5*8 + 2*8*i, 32, can_data[i], 2, OLED_8X16);
            }
        }*/
		
		if(g_rx_updated)
		{
			g_rx_updated = 0;
			OLED_ShowHexNum(6*8, 16, g_last_rx_id, 3, OLED_8X16);
			for(uint8_t i = 0; i < g_last_rx_len && i < 8; i++)
			{
				OLED_ShowHexNum(5*8 + 2*8*i, 32, g_last_rx_data[i], 2, OLED_8X16);
			}
		}
		
		OLED_Update();
		OLED2_Update();
        
        Delay_ms(50);
		
		IWDG_ReloadCounter();
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
		while (Key_Scan_Flag > 0U)
		{
			uint32_t primask = __get_PRIMASK();   /* 保存中断状态 */
			__disable_irq();
			Key_Scan_Flag--;
			if (primask == 0U) { __enable_irq(); }  /* 仅原状态为开才恢复 */
			Key_Scan();
		}
		
		HeartBeat_Manager();
		
		uint32_t can_id;
        uint8_t can_data[8];
        uint8_t can_len;
        uint8_t rx_count = 0U;
        while (MyCAN_Receive_Message(&can_id, &can_len, can_data) 
               && rx_count < 8U)
        {
            CAN_Data_Handler(can_id, can_len, can_data);
            rx_count++;
        }
		
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
				if(selected_param == 1)
				{
                    heartbeat_period = (heartbeat_period % 10) + 1;
                }
                break;
                
            case KEY4_PRESS:  // 保存并退出
                // 保存参数到EEPROM（这里简化处理）
                printf("Parameters saved: Baudrate=%s, Heartbeat=%ds\r\n", 
                       baudrate_names[can_baudrate], heartbeat_period);
        
				/* 更新心跳周期 */
				g_heartbeat_period_ms = heartbeat_period * 1000U;  /* 秒转毫秒 */
        
				/* TODO: 保存到 EEPROM/Flash */
				// EEPROM_Write(EEPROM_ADDR_HEARTBEAT_PERIOD, heartbeat_period);
        
				printf("Parameters applied. System restart...\r\n");
				Delay_ms(100);
				
                system_state = SYSTEM_READY;
                return;
        }
        
        // 显示参数设置界面
        OLED_Clear();
		OLED2_Clear();
        OLED2_ShowString(0, 0, "ParameterSetting", OLED_8X16);
        
        // CAN波特率
        OLED_ShowString(8, 0, "CANBaudrate:", OLED_8X16);
        if(selected_param == 0)
			OLED_ShowString(0, 0, ">", OLED_8X16);
		
        OLED_ShowString(2*8, 16, (char*)baudrate_names[can_baudrate], OLED_8X16);
        
        // 心跳周期
        OLED_ShowString(8, 32, "Heart Period:", OLED_8X16);
        if(selected_param == 1)
			OLED_ShowString(0, 32, ">", OLED_8X16);
		
        OLED_ShowNum(2*8, 48, heartbeat_period, 2, OLED_8X16);
        OLED_ShowString(4*8, 48, "s", OLED_8X16);
        
        OLED2_ShowString(0, 16, "Key1/2: Select", OLED_8X16);
        OLED2_ShowString(0, 32, "Key3: Change", OLED_8X16);
        OLED2_ShowString(0, 48, "Key4: Save&Back", OLED_8X16);
		
		OLED_Update();
		OLED2_Update();
		
        Delay_ms(50);
		
		IWDG_ReloadCounter();
    }
}

/** 函  数：心跳包管理
  * 参  数：无
  * 返回值：无
  */
void HeartBeat_Manager(void)
{
    static uint32_t last_heartbeat_send = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 发送心跳包
    if(current_time - last_heartbeat_send > g_heartbeat_period_ms)
	{
        last_heartbeat_send = current_time;
        MyCAN_Send_Heartbeat(NODE_ID, STATUS_NORMAL, ERROR_NONE, current_time / 1000);
        
        // LED指示
        LED1_Turn();
    }
	heartbeat_time[NODE_ID - 1] = current_time;
	
	/* 巡检 ECU2/ECU3 心跳 */
    {
        uint32_t period = g_heartbeat_period_ms;
        if (period == 0U)  period = 1000U;   //配置异常时兜底 1 秒
        uint32_t tout = HEARTBEAT_TIMEOUT_MULT * period;
        uint8_t i;
        for (i = 1U; i < HEARTBEAT_NODE_COUNT; i++)
		{     /* idx1=ECU2 idx2=ECU3 */
            if (hbm_nodes[i].alive != 0U)
			{
                if ((HAL_GetTick() - hbm_nodes[i].last_seen_ms) > tout)
				{
                    hbm_nodes[i].alive = 0U;
                    /* 诊断打印 + 通过 CAN 上报 DTC 0xE102 (ECU2) / 0xE103 (ECU3)，见 M28 DTC 简表 */
                    printf("HEARTBEAT: node ECU%u MISSING! tout=%lums\r\n", (unsigned)(i+1U), (unsigned long)tout);
                    MyCAN_Send_ErrorReport(NODE_ID_ECU1, 1U, (uint16_t)(0xE100U + (i+1U)));
                }
            }
        }
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
	if (data == NULL)
    {
        printf("CAN_Data_Handler: NULL data\r\n");
        return;
    }
	
    switch(id)
	{
        case MSG_ID_HEARTBEAT:
		{
            if (len < sizeof(HeartBeat_Data)) break;
            HeartBeat_Data hb;
            memcpy(&hb, data, sizeof(HeartBeat_Data));
			/* 收到任何 ECU 心跳 → 更新监控表 */
			if ((hb.node_id >= NODE_ID_ECU1) && (hb.node_id <= NODE_ID_ECU3))
			{
				uint8_t idx = (uint8_t)(hb.node_id - NODE_ID_ECU1);       // 0/1/2
				hbm_nodes[idx].last_seen_ms = HAL_GetTick();
				hbm_nodes[idx].alive = 1U;
				hbm_nodes[idx].prev_status = hb.status;
			}
            if (hb.node_id >= 1U && hb.node_id <= NODE_NUM)
            {
                heartbeat_time[hb.node_id - 1] = HAL_GetTick();
            }
            break;
        }
            
        case MSG_ID_MOTOR_STATUS:
		{
			if (len != sizeof(MotorStatus_Data))
			{
				printf("MotorStatus len err: %u\r\n", len);
				break;
			}
			
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
            if (len < sizeof(Sensor_Data)) break;
            memcpy(&sensor_data, data, sizeof(Sensor_Data));
            if (sensor_data.distance < 200)
            {
                printf("Warning: Distance too close! %dmm\r\n", sensor_data.distance);
            }
            break;
        }
		
		case MSG_ID_SENSOR_VOLTAGE:
		{
			SensorVoltage_Data voltage_frame;
		
			if (len != sizeof(voltage_frame))
			{
				break;
			}
		
			memcpy(&voltage_frame, data, sizeof(voltage_frame));
			sensor_voltage_mv = voltage_frame.voltage;
			break;
		}
		
		case MSG_ID_SYSTEM_CTRL:
		{
			if (len < sizeof(SystemCtrl_Data)) break;
            SystemCtrl_Data ctrl;
            memcpy(&ctrl, data, sizeof(SystemCtrl_Data));
            printf("SystemCtrl: cmd=%d\r\n", ctrl.command);
			//根据需要处理
			
			break;
		}
		
		case MSG_ID_ERROR_REPORT:
		{
			/* NULL 指针校验 */
			if (data == NULL || len < sizeof(ErrorReport_Data))
			{
				printf("ErrorReport: invalid data or len=%u\r\n", len);
				break;
			}
		
			/* 使用 memcpy 替代强制类型转换，符合 MISRA C Rule 11.3 */
			ErrorReport_Data err;
			memcpy(&err, data, sizeof(ErrorReport_Data));
		
			/* 存储到文件作用域变量，供 Error_Handler 使用 */
			s_last_error = err;
			s_error_pending = 1U;
			//打印错误报告
			printf("Error Report from Node %d: type=0x%02X, code=0x%04X, time=%u\r\n",
					err.node_id, err.error_type, err.error_code, err.timestamp);
			
			if(err.error_type == ERROR_NONE)
			{
				printf("No Error on node %d\r\n", err.node_id);
			}
			else
			{
				if(err.error_type & ERROR_CAN_COMM)
				{
					printf("CAN communication error on node %d\r\n", err.node_id);
				}
				if(err.error_type & ERROR_VOLTAGE_LOW)
				{
					printf("Voltage low on node %d\r\n", err.node_id);
				}
				if(err.error_type & ERROR_MPU6050_FAIL)
				{
					printf("MPU6050 failure on node %d\r\n", err.node_id);
				}
				if(err.error_type & ERROR_ULTRASONIC_FAIL)
				{
					printf("Ultrasonic failure on node %d\r\n", err.node_id);
				}
				if(err.error_type & ERROR_MOTOR_FAULT)
				{
					printf("Motor fault on node %d\r\n", err.node_id);
				}
				if(err.error_type & ERROR_SENSOR_FUSION)
				{
					printf("Sensor fusion error on node %d\r\n", err.node_id);
				}
				if(err.error_type & ERROR_OVERCURRENT)
				{
					printf("Overcurrent on node %d\r\n", err.node_id);
				}
				if(err.error_type & ERROR_WATCHDOG)
				{
					printf("Watchdog reset on node %d\r\n", err.node_id);
				}
				/* 检查未知错误位 */
				if(err.error_type & ~(ERROR_CAN_COMM | ERROR_VOLTAGE_LOW |
									ERROR_MPU6050_FAIL | ERROR_ULTRASONIC_FAIL |
									ERROR_MOTOR_FAULT | ERROR_SENSOR_FUSION |
									ERROR_OVERCURRENT | ERROR_WATCHDOG))
				{
					printf("Unknown error type 0x%02X on node %d\r\n",
						err.error_type, err.node_id);
				}
				/* ---- 触发安全机制：严重错误进入 SYSTEM_ERROR ---- */
				if (err.error_type & (ERROR_MOTOR_FAULT | ERROR_OVERCURRENT | ERROR_WATCHDOG))
				{
					/* 先发送停止指令，再进入错误状态 */
					MyCAN_Send_SpeedCmd(0, 0, 100);
					Delay_ms(5);  /* 等待 CAN 发送完成 */
					system_state = SYSTEM_ERROR;
					printf("ECU2 critical fault 0x%04X, motor stopped\r\n", err.error_type);
				}
			}
			break;
		}
            
		default:
		{	// 未知报文处理
			uint8_t print_len = (len > 8) ? 8 : len;
			printf("Unknown CAN message: ID=0x%03x, Len=%d, Data=",id,len);
			for(uint8_t i = 0;i < print_len;i ++)
			{
				printf("%02x",data[i]);
			}
			printf("\r\n");
			break;
		}
    }
	
	g_last_rx_id = id;
	g_last_rx_len = len;
	memcpy(g_last_rx_data, data, len < 8 ? len : 8);
	
	g_rx_updated = 1;
}



int main(void) 
{
	System_Init();
	
    Show_MainMenu();
	
	printf("System started. Press keys to navigate menu.\r\n");
	
	
	
    while(1) 
	{
		while (Key_Scan_Flag > 0U)
		{
			uint32_t primask = __get_PRIMASK();   /* 保存中断状态 */
			__disable_irq();
			Key_Scan_Flag--;
			if (primask == 0U) { __enable_irq(); }  /* 仅原状态为开才恢复 */
			Key_Scan();
		}
		Key_Handler();
		
		HeartBeat_Manager();
		
		// CAN数据处理
        uint32_t can_id;
        uint8_t can_data[8];
        uint8_t can_len;
        
        while(MyCAN_Receive_Message(&can_id, &can_len, can_data))
		{
            CAN_Data_Handler(can_id, can_len, can_data);
        }
        
		// 系统状态机
        switch(system_state)
		{
			case SYSTEM_IDLE:
			{
				//LED1慢闪表示等待
				static uint32_t idle_blink = 0;
				if(HAL_GetTick() - idle_blink > 1000)
				{
					idle_blink = HAL_GetTick();
					LED1_Turn();
				}
				break;
			}
			
            case SYSTEM_READY:
            {
				//LED1常亮表示就绪
				LED1_ON();
				LED2_OFF();
				LED3_OFF();
				//LED4_OFF();
                break;
			}
            
            case REMOTE_CONTROL:
            case SENSOR_DISPLAY:
            case SYSTEM_MONITOR:
            case CAN_TEST:
            case PARAM_SETTING:
                // 这些状态在各自的函数中处理
                break;
                
            case SYSTEM_ERROR:
                // 错误处理
                Error_Handler();
                break;
			
			default:
				//未知状态，复位系统
				printf("Unknown system state! Resetting...\r\n");
				NVIC_SystemReset();
				break;
        }
		/* 喂狗 */
		IWDG_ReloadCounter();
    }
}

/**
  * 函    数：错误处理
  * 参    数：无
  * 返 回 值：无
  * 说    明：基于 s_last_error / s_error_pending 状态机实现
  *           1. 接收 CAN 错误报告时由 CAN_Data_Handler 设置 s_error_pending=1
  *           2. 首次进入错误态时记录起始时间并打印错误详情
  *           3. 5 秒超时后触发系统复位
  */
void Error_Handler(void)
{
    /* LED 快闪（100ms 周期）表示错误 */
    static uint32_t last_blink = 0U;
    if (HAL_GetTick() - last_blink > 100U)
    {
        last_blink = HAL_GetTick();
        LED1_Turn();
        LED2_Turn();
        LED3_Turn();
    }

    /* 错误恢复状态变量（static，函数退出后保持） */
    static uint32_t error_start = 0U;
    static uint8_t  error_active = 0U;

    /* 收到新错误时，记录起始时间并打印 */
    if (s_error_pending && !error_active)
    {
        error_start  = HAL_GetTick();
        error_active = 1U;
        printf("=== ERROR HANDLER ACTIVATED ===\r\n");
        printf("Source: Node %d, Type: 0x%02X, Code: 0x%04X\r\n",
               s_last_error.node_id, s_last_error.error_type, s_last_error.error_code);
        printf("Time: %u ms\r\n", s_last_error.timestamp);
    }

    /* 5 秒超时后触发系统复位 */
    if (error_active && (HAL_GetTick() - error_start > 5000U))
    {
        printf("System reset after error timeout (5s)...\r\n");
        NVIC_SystemReset();
        /* NVIC_SystemReset() 不会返回 */
    }
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		Tick_ms ++;
		/* 计数器累加（不是置位）；uint8_t ++ 在 Cortex-M3 单字节 STRB 原子 */
		if (Key_Scan_Flag < 0xFFU)
		{
			Key_Scan_Flag++;
		}
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
