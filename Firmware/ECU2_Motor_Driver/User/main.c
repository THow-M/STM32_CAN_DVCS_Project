#include "stm32f10x.h"                  // Device header
#include "System_Config.h"
#include "ecu2_main.h"
#include "OLED.h"
#include "Delay.h"
#include "LED.h"
#include "Timer.h"
#include "MyCAN.h"
#include "Serial.h"
#include "PWM.h"
#include "PID_Control.h"
#include "Encoder.h"
#include "Motor.h"
#include <stdio.h>
#include <string.h>

// 全局变量
System_State system_state = SYS_IDLE;
uint32_t system_uptime = 0;
uint8_t error_code = ERROR_NONE;
uint8_t can_connected = 0;
static uint8_t led_blink_state = 0;

static uint8_t diagnostic_requested = 0;
static uint8_t calibration_requested = 0;


// 控制变量
PID_Controller speed_pid;
float target_speed_rpm = 0.0f;
float actual_speed_rpm = 0.0f;
float pid_output = 0.0f;
uint8_t motor_direction = MOTOR_STOP;
uint8_t control_mode = CONTROL_MODE_AUTO;

// 定时器
uint32_t last_pid_time = 0;
uint32_t last_can_send_time = 0;
uint32_t last_heartbeat_time = 0;
uint32_t last_status_time = 0;
uint32_t last_protection_check = 0;
uint32_t last_debug_time = 0;
uint32_t last_led_time = 0;

// CAN接收数据
SpeedCmd_Data speed_cmd = {0};
SystemCtrl_Data system_ctrl = {0};
uint8_t heartbeat_status[NODE_NUM] = {0};
uint32_t heartbeat_time[NODE_NUM] = {0};

static uint8_t SystemCtrl_TargetsThisNode(const SystemCtrl_Data *ctrl)
{
    return (ctrl->param1 == CAN_TARGET_BROADCAST ||
            ctrl->param1 == NODE_ID) ? 1U : 0U;
}

/** 函  数：系统初始化
  * 参  数：无
  * 返回值：无
  */
void System_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	Timer_Init();
	
	Serial_Init(DEBUG_BAUDRATE);
	
	printf("\r\n\r\n");
    printf("================================\r\n");
    printf("  ECU2 - Motor Drive Unit\r\n");
    printf("  Version: 1.0.0\r\n");
    printf("  Build Date: %s %s\r\n", __DATE__, __TIME__);
    printf("================================\r\n\r\n");
	
    LED_Init();
    
    MyCAN_Init(CAN_BAUDRATE);
    
    Encoder_Init();
    
    // 初始化电机
    // PWM频率 = 72MHz / (0+1) / (7199+1) = 10kHz
	Motor_Init(7199, 0);
    
    // 初始化PID控制器
    // 参数需要根据实际电机调整
    PID_Init(&speed_pid, 0.8f, 0.05f, 0.01f, 1000.0f, -1000.0f, 1000.0f);
    speed_pid.dead_zone = 5.0f;  // 5RPM死区
    speed_pid.filter_coeff = 0.7f;  // 低通滤波系数
    
    // 系统启动指示
    printf("System initializing...\r\n");
	
	LED_ON();
	Delay_ms(200);
	LED_OFF();
	Delay_ms(200);
	LED_ON();
	Delay_ms(200);
	LED_OFF();
	Delay_ms(200);
	LED_ON();
	Delay_ms(200);
	LED_OFF();
	Delay_ms(200);
	
	// 编码器校准
    printf("Calibrating encoder...\r\n");
    Encoder_Calibrate();
	
	/* 看门狗 */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_64);    /* 1.6kHz */
	IWDG_SetReload(0x0FFF);                  /* ~2.5s */
	IWDG_ReloadCounter();
	IWDG_Enable();
    
    // 启动系统
    system_state = SYS_READY;
    system_uptime = 0;
    
    printf("System initialized successfully!\r\n");
    printf("Control Mode: %s\r\n", 
           control_mode == CONTROL_MODE_MANUAL ? "Manual" : "Auto");
    printf("Waiting for commands...\r\n");
	
	heartbeat_time[NODE_ID - 1] = HAL_GetTick();
    
	//启动成功指示
    LED_ON();
}

/** 函  数：主控制循环
  * 参  数：无
  * 返回值：无
  */
void Control_Loop(void)
{
    uint32_t current_time = HAL_GetTick();
    
    // 1. 保护检测
    if(current_time - last_protection_check > 100)
	{  // 100ms
        last_protection_check = current_time;
		
        // 编码器故障检测
		uint8_t encoder_fault = Encoder_Fault_Check();
		if(encoder_fault)
		{
			error_code = ERROR_SENSOR_FUSION;
			system_state = SYS_ERROR;
			Motor_EmergencyStop();
			printf("Encoder fault: 0x%02X\r\n", encoder_fault);
			// 可以切换到开环控制
		}
		
        uint8_t protection_error = Motor_ProtectionCheck();
        if(protection_error)
		{
            Motor_ErrorHandler(protection_error);
            error_code = protection_error;
            system_state = SYS_ERROR;
        }
    }
    
    
    
    // 3. PID控制
    if(control_mode == CONTROL_MODE_AUTO && system_state == SYS_RUN)
	{
		Motor_RunPIDControl();
        /*if(current_time - last_pid_time >= 10)
		{  // 10ms控制周期
            last_pid_time = current_time;
            
			float dt = (float)(current_time - last_pid_time) / 1000.0f;
			
            // 获取实际速度
            Encoder_Data encoder = Encoder_GetData();
            actual_speed_rpm = encoder.speed_rpm;
            
            // PID计算
            pid_output = PID_Calculate(&speed_pid, target_speed_rpm, actual_speed_rpm, dt);
            
            // 设置电机速度
            int16_t speed = (int16_t)pid_output;
            if(speed >= 0)
			{
                motor_direction = MOTOR_FORWARD;
            }
			else
			{
                speed = -speed;
                motor_direction = MOTOR_REVERSE;
            }
            
            Motor_SetTargetSpeed(speed, motor_direction);
            
            // 调试输出
            if(current_time - last_debug_time > 500)
			{  // 500ms
                last_debug_time = current_time;
                printf("PID: Target=%.1f, Actual=%.1f, Output=%d, Dir=%d\r\n",
                       target_speed_rpm, actual_speed_rpm, speed, motor_direction);
            }
        }*/
    }
}

/** 函  数：通信处理
  * 参  数：无
  * 返回值：无
  */
void Communication_Handler(void)
{
    uint32_t current_time = HAL_GetTick();
    
    // 1. CAN数据处理
    uint32_t can_id;
    uint8_t can_data[8];
    uint8_t can_len;
    
    while(MyCAN_Receive_Message(&can_id, &can_len, can_data))
	{
        MyCAN_Data_Handler(can_id, can_len,can_data);
    }
    
    // 2. 发送心跳包
    if(current_time - last_heartbeat_time > HEARTBEAT_PERIOD)
	{
        last_heartbeat_time = current_time;
        
        uint8_t status = (system_state == SYS_ERROR) ? STATUS_ERROR : STATUS_NORMAL;
        MyCAN_Send_Heartbeat(NODE_ID, status, error_code, system_uptime);
		
		heartbeat_time[NODE_ID - 1] = current_time;
        
        // LED指示
        //LEDx_ON();
    }
    
    // 3. 发送电机状态
    if(current_time - last_can_send_time > 100)
	{  // 100ms
        last_can_send_time = current_time;
        
        if(can_connected)
		{
            // 获取电机状态
            Motor_Status motor_status = Motor_GetStatus();
            Encoder_Data encoder = Encoder_GetData();
            //uint8_t temperature = Motor_GetTemperature();
            uint16_t current = Motor_GetCurrent();
            
            // 组合状态位
            uint8_t status_byte = 0;
            if(motor_status.protection_status & ERROR_OVER_CURRENT) status_byte |= ERROR_OVER_CURRENT;  // 过流
            if(motor_status.protection_status & ERROR_OVER_TEMP) status_byte |= ERROR_OVER_TEMP;  // 过热
            if(motor_status.protection_status & ERROR_STALL) status_byte |= ERROR_STALL;  // 堵转
			if(motor_status.protection_status & ERROR_OVER_SPEED) status_byte |= ERROR_OVER_SPEED;  // 过速
			if(motor_status.protection_status & ERROR_UNDER_VOLTAGE) status_byte |= ERROR_UNDER_VOLTAGE;  // 低压
            
            MyCAN_Send_MotorStatus((int16_t)encoder.speed_rpm, current, /*temperature,*/ status_byte);
            
            // LED指示
            //LEDx_ON();
        }
    }
	else if(current_time - last_can_send_time > 10)
	{
        //LEDx_OFF();
    }
    
    // 4. 检查CAN连接状态
    static uint32_t last_can_msg_time = 0;
    static uint8_t can_msg_received = 0;
    
    for(uint8_t i = 0; i < NODE_NUM; i++)
	{
        if(i == NODE_ID - 1) continue;  // 跳过自己
        
        if(current_time - heartbeat_time[i] < HEARTBEAT_TIMEOUT)
		{
            can_msg_received = 1;
            last_can_msg_time = current_time;
        }
    }
    
    if(can_msg_received && current_time - last_can_msg_time < CAN_TIMEOUT)
	{
        can_connected = 1;
        //LEDx_ON();  // CAN连接指示
    }
	else
	{
        can_connected = 0;
        //LEDx_OFF();
        
        // 如果长时间没有收到CAN消息，切换到手动模式
        printf("CAN disconnected, stopping motor\r\n");
		Motor_SetTargetSpeed(0, MOTOR_STOP);
		if(control_mode == CONTROL_MODE_AUTO)
		{
			control_mode = CONTROL_MODE_MANUAL;
		}
		system_state = SYS_IDLE;
    }
}

/** 函  数：CAN数据处理
  * 参  数：id 要处理的报文id
  * 参  数：len 要处理的报文数据长度
  * 参  数：data 要处理的报文数据
  * 返回值：无
  */
void MyCAN_Data_Handler(uint32_t id, uint8_t len,const uint8_t* data)
{
	if (data == NULL || len == 0 || len > 8) return;
	
    switch(id)
	{
        case MSG_ID_HEARTBEAT:
		{
			HeartBeat_Data hb = {0};
			/* 精确长度判断（packed(1) 结构体 = 8B） */
			if (len != sizeof(HeartBeat_Data)) break;
			memcpy(&hb, data, sizeof(HeartBeat_Data));
            
            if(hb.node_id >= 1 && hb.node_id <= NODE_NUM)
			{
                heartbeat_time[hb.node_id - 1] = HAL_GetTick();
                
                if(hb.node_id == NODE_ID_ECU1)
				{  // 来自ECU1
                    can_connected = 1;
                }
            }
            break;
        }
            
        case MSG_ID_SPEED_CMD:
		{
            if(len >= sizeof(SpeedCmd_Data))
			{
                memcpy(&speed_cmd, data, sizeof(SpeedCmd_Data));
                
                printf("Received SpeedCmd: %d RPM, Dir=%d\r\n", 
                       speed_cmd.target_speed, speed_cmd.direction);
                
                // 处理速度指令
                if(system_state != SYS_ERROR)
				{
                    target_speed_rpm = speed_cmd.target_speed;
                    motor_direction = speed_cmd.direction;
                    
                    if(control_mode == CONTROL_MODE_MANUAL)
					{
                        // 手动模式直接控制
                        if(motor_direction == MOTOR_STOP)
						{
                            Motor_SetTargetSpeed(0, MOTOR_STOP);
                            system_state = SYS_IDLE;
                        }
						else
						{
                            Motor_SetTargetSpeed(target_speed_rpm, motor_direction);
                            system_state = SYS_RUN;
                        }
                    }
					else if(control_mode == CONTROL_MODE_AUTO)
					{
                        // 自动模式，PID控制会自动处理
                        if(motor_direction == MOTOR_STOP)
						{
                            target_speed_rpm = 0;
                            system_state = SYS_IDLE;
                        }
						else
						{
                            system_state = SYS_RUN;
                        }
                    }
                    
                    // LED指示
					if(system_state == SYS_RUN)
					{
						LED_ON();
					}
                }
            }
            break;
        }
            
        case MSG_ID_SYSTEM_CTRL:
		{
            if(len >= sizeof(SystemCtrl_Data))
			{
                memcpy(&system_ctrl, data, sizeof(SystemCtrl_Data));
				
				if (SystemCtrl_TargetsThisNode(&system_ctrl) == 0U)
				{
					break;
				}
                
                printf("Received SystemCtrl: Command=%d\r\n", system_ctrl.command);
                
                switch(system_ctrl.command)
				{
                    case SYS_CMD_START:
						Motor_SetControlMode(CONTROL_MODE_AUTO);
                        system_state = SYS_RUN;
                        printf("System started in auto mode\r\n");
                        break;
                        
                    case SYS_CMD_STOP:
                        Motor_SetTargetSpeed(0, MOTOR_STOP);
                        system_state = SYS_IDLE;
                        printf("System stopped\r\n");
                        break;
                        
                    case SYS_CMD_RESET:
						motor_control.state = MOTOR_STATE_IDLE;
						motor_control.error_count = 0;
						motor_control.error_code = 0;
                        NVIC_SystemReset();
                        break;
                        
                    case SYS_CMD_MANUAL:
                        Motor_SetControlMode(CONTROL_MODE_MANUAL);
                        printf("Switched to manual mode\r\n");
                        break;
                        
                    case SYS_CMD_AUTO:
                        Motor_SetControlMode(CONTROL_MODE_AUTO);
                        printf("Switched to auto mode\r\n");
                        break;
                        
                    case SYS_CMD_CALIBRATE:
                        calibration_requested = 1;
                        break;
                        
                    case SYS_CMD_DIAGNOSTIC:
                        diagnostic_requested = 1;
                        break;
                }
            }
            break;
        }
            
        default:
            // 其他报文
            break;
    }
}

/** 函  数：系统诊断
  * 参  数：无
  * 返回值：无
  */
void System_Diagnostic(void)
{
    printf("\r\n=== System Diagnostic ===\r\n");
    printf("System State: %d\r\n", system_state);
    printf("Control Mode: %s\r\n", 
           control_mode == CONTROL_MODE_MANUAL ? "Manual" : "Auto");
    printf("Uptime: %d seconds\r\n", system_uptime);
    printf("Error Code: 0x%02X\r\n", error_code);
    printf("CAN Connected: %s\r\n", can_connected ? "Yes" : "No");
    printf("\r\n");
    
    // 电机状态
    Motor_Status motor = Motor_GetStatus();
    printf("Motor State: %d\r\n", motor.state);
    printf("Motor Speed: %d/%d\r\n", motor.speed, motor.target_speed);
    printf("Motor Direction: %d\r\n", motor.direction);
    printf("Motor Protection: 0x%02X\r\n", motor.protection_status);
    printf("\r\n");
    
    // 编码器状态
    Encoder_Data encoder = Encoder_GetData();
    printf("Encoder Speed: %.2f RPM\r\n", encoder.speed_rpm);
    printf("Encoder Direction: %d\r\n", encoder.direction);
    printf("Encoder Position: %d\r\n", encoder.position);
    printf("Encoder Valid: %s\r\n", encoder.valid ? "Yes" : "No");
    printf("\r\n");
    
    // PID状态
    printf("PID Target: %.1f RPM\r\n", target_speed_rpm);
    printf("PID Actual: %.1f RPM\r\n", actual_speed_rpm);
    printf("PID Output: %.1f\r\n", pid_output);
    printf("PID Integral: %.3f\r\n", speed_pid.integral);
    printf("\r\n");
    
    // 心跳状态
    for(uint8_t i = 0; i < NODE_NUM; i++)
	{
        uint32_t time_since = HAL_GetTick() - heartbeat_time[i];
        printf("Node %d: %s (%.1fs ago)\r\n", 
               i + 1, 
               time_since < HEARTBEAT_TIMEOUT ? "Online" : "Offline",
               time_since / 1000.0f);
    }
    
    printf("===========================\r\n\r\n");
}

/**
  * 函  数：LED状态更新
  * 参  数：无
  * 返回值：无
  * 注  释：根据系统状态控制LED闪烁模式
  */
static void LED_State_Update(void)
{
    uint32_t current_time = HAL_GetTick();
    uint32_t blink_period = 0;

    switch (system_state)
    {
        case SYS_IDLE:
            blink_period = 1000;  /* 慢闪 1Hz */
            break;
        case SYS_READY:
            blink_period = 0;     /* 常亮 */
            break;
        case SYS_RUN:
            blink_period = 200;   /* 快闪 5Hz */
            break;
        case SYS_ERROR:
            blink_period = 100;   /* 极快闪 */
            break;
        case SYS_CALIBRATING:
            blink_period = 500;   /* 中闪 2Hz */
            break;
        default:
            blink_period = 1000;
            break;
    }

    if (blink_period == 0)
    {
        LED_ON();
    }
    else if (current_time - last_led_time >= blink_period)
    {
        last_led_time = current_time;
        led_blink_state = !led_blink_state;
        if (led_blink_state)
        {
            LED_ON();
        }
        else
        {
            LED_OFF();
        }
    }
}

// 测试函数
void Motor_Test_Suite(void)
{
    printf("Starting motor test suite...\r\n");
    
    // 测试1：PWM输出测试
    printf("\nTest 1: PWM Output Test\r\n");
    for(int i = 0; i <= 1000; i += 100)
	{
        Motor_SetTargetSpeed(i, MOTOR_FORWARD);
        printf("PWM: %d/1000\r\n", i);
        Delay_ms(500);
    }
    Motor_SetTargetSpeed(0, MOTOR_STOP);
    Delay_ms(1000);
    
    // 测试2：编码器测试
    printf("\nTest 2: Encoder Test\r\n");
    for(int i = 0; i < 10; i++)
	{
        Encoder_Data encoder = Encoder_GetData();
        printf("Encoder: Speed=%.2f RPM, Dir=%d, Pos=%d\r\n",
               encoder.speed_rpm, encoder.direction, encoder.position);
        Delay_ms(100);
    }
    
    // 测试3：PID控制测试
    printf("\nTest 3: PID Control Test\r\n");
    control_mode = CONTROL_MODE_AUTO;
	Motor_SetControlMode(control_mode);
    target_speed_rpm = 300.0f;
    system_state = SYS_RUN;
    
    for(int i = 0; i < 50; i++)
	{  // 测试5秒
        Control_Loop();
        Delay_ms(100);
        
        if(i % 10 == 0)
		{
            Encoder_Data encoder = Encoder_GetData();
            printf("PID: Target=%.1f, Actual=%.1f, Error=%.1f\r\n",
                   target_speed_rpm, encoder.speed_rpm, 
                   target_speed_rpm - encoder.speed_rpm);
        }
    }
    
    Motor_SetTargetSpeed(0, MOTOR_STOP);
    system_state = SYS_IDLE;
    
    printf("\nMotor test suite completed.\r\n");
}





int main(void) 
{
	System_Init();
	
	printf("ECU2 Main Loop Started\r\n");
	
	//Motor_Test_Suite();
	
	while (1)
	{
		uint32_t current_time = HAL_GetTick();
        
        // 更新运行时间
        if(current_time - last_status_time >= 1000)
		{  // 1秒
            last_status_time = current_time;
            system_uptime++;
            
            // 每秒输出状态
            if(system_uptime % 5 == 0)
			{  // 每5秒
                Encoder_Data encoder = Encoder_GetData();
                printf("Uptime: %ds, Speed: %.1f RPM, State: %d\r\n",
                       system_uptime, encoder.speed_rpm, system_state);
            }
        }
        
        // 控制循环
        Control_Loop();
        
        // 通信处理
        Communication_Handler();
		
		if (diagnostic_requested)
		{
			diagnostic_requested = 0;
			System_Diagnostic();  /* 异步执行 */
		}
		if (calibration_requested)
		{
			calibration_requested = 0;
			Encoder_Calibrate();
		}
		
		//状态机
		switch (system_state)
		{
            case SYS_IDLE:
                /* 空闲状态：电机停止，PID清零 */
                PID_Reset(&speed_pid);
                LED_State_Update();
                break;

            case SYS_READY:
                /* 准备就绪：等待指令 */
                LED_State_Update();
                break;

            case SYS_RUN:
                /* 运行状态：PID控制已在Control_Loop中执行 */
                LED_State_Update();
                break;

            case SYS_ERROR:
                /* 错误状态 */
                Error_Handler();
                break;

            case SYS_CALIBRATING:
                /* 校准状态 */
                LED_State_Update();
                break;

            default:
                system_state = SYS_IDLE;
                break;
        }
		/* 喂狗 */
		IWDG_ReloadCounter();
	}
}

// 错误处理
void Error_Handler(void)
{
    static uint32_t error_start_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    if(error_start_time == 0)
	{
        error_start_time = current_time;
		Motor_EmergencyStop();          /* 强制切断 PWM */
        PID_Reset(&speed_pid);
        printf("Entering error state. Code: 0x%02X\r\n", error_code);
    }
    
    // LED快速闪烁表示错误
    static uint32_t last_blink = 0;
    if(current_time - last_blink > 100)
	{
		last_blink = current_time;  // 必须更新时间戳
		static uint8_t led_state = 0;
		led_state = !led_state;
		if(led_state)
			LED_ON();
		else
			LED_OFF();
	}
    
	/* 根据错误类型决定是否允许自动恢复 */
    uint8_t auto_recoverable = 0U;
    switch (error_code)
    {
        case ERROR_OVER_TEMP:
            auto_recoverable = 1U;  /* 过热可恢复 */
            break;
        case ERROR_OVER_CURRENT:
        case ERROR_STALL:
            auto_recoverable = 0U;  /* 过流/堵转需手动复位 */
            break;
        default:
            auto_recoverable = 1U;
            break;
    }
	
    // 尝试自动恢复
    if(auto_recoverable && (current_time - error_start_time > 5000))
	{
		
		// 5秒后尝试恢复
        printf("Attempting to recover from error...\r\n");
        
        // 清除错误标志
        if(system_state == SYS_ERROR)
		{
            system_state = SYS_IDLE;
            error_code = ERROR_NONE;
            error_start_time = 0;
            
            // 重置控制器
            PID_Reset(&speed_pid);
            
            printf("Error recovery successful.\r\n");
        }
    }
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		Tick_ms++;
		
		
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
