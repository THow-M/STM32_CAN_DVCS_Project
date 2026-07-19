#include "stm32f10x.h"                  // Device header
#include "System_Config.h"
#include "ecu3_main.h"
#include "Timer.h"
#include "Delay.h"
#include "Key.h"
#include "LED.h"
#include "Serial.h"
#include "MyCAN.h"
#include "MPU6050.h"
#include "Ultrasonic.h"
#include "Voltage.h"
#include <stdio.h>
#include <math.h>

// 全局变量
System_State system_state = SYS_IDLE;
uint32_t system_uptime = 0;
uint8_t error_code = ERROR_NONE;
uint8_t can_connected = 0;

// 传感器数据
Sensor_Fusion sensor_fusion = {0};
uint8_t sensor_ready = 0;
uint8_t calibration_complete = 0;

// 定时器
uint32_t last_sensor_update = 0;
uint32_t last_can_send = 0;
uint32_t last_heartbeat = 0;
uint32_t last_status_update = 0;
uint32_t last_diagnostic = 0;
uint32_t last_calibration_check = 0;

// CAN接收数据
SystemCtrl_Data system_ctrl = {0};
uint8_t heartbeat_status[NODE_NUM] = {0};
uint32_t heartbeat_time[NODE_NUM] = {0};

/** 函  数：系统初始化
  * 参  数：无
  * 返回值：无
  */
void System_Init(void)
{
	// 初始化定时器2
	Timer_Init();
	
    // 初始化串口
    Serial_Init(DEBUG_BAUDRATE);
    
    printf("\r\n\r\n");
    printf("================================\r\n");
    printf("  ECU3 - Sensor Fusion Unit\r\n");
    printf("  Version: 1.0.0\r\n");
    printf("  Build Date: %s %s\r\n", __DATE__, __TIME__);
    printf("================================\r\n\r\n");
    
    // 初始化LED
    LED_Init();
    
    // 系统启动指示
    printf("System initializing...\r\n");
    
    LED1_ON();
    Delay_ms(200);
    LED2_ON();
    Delay_ms(200);
    LED3_ON();
    Delay_ms(200);
    
    LED1_OFF();
    LED2_OFF();
    LED3_OFF();
    
    // 初始化CAN
    MyCAN_Init(CAN_BAUDRATE);
    
    // 初始化传感器
    printf("Initializing sensors...\r\n");
    
    // 1. 初始化MPU6050
    if(MPU6050_Init())
	{
        printf("MPU6050 initialized successfully\r\n");
        LED1_ON();
    }
	else
	{
        printf("MPU6050 initialization failed\r\n");
        error_code |= ERROR_MPU6050_FAIL;
    }
    
    // 2. 初始化超声波
    Ultrasonic_Init();
    printf("Ultrasonic initialized\r\n");
    LED2_ON();
    
    // 3. 初始化电压检测
    Voltage_Init();
    printf("Voltage detection initialized\r\n");
    LED3_ON();
    
    // 等待传感器稳定
    Delay_ms(1000);
    
    LED1_OFF();
    LED2_OFF();
    LED3_OFF();
    
    // 传感器自检
    printf("Running sensor self-tests...\r\n");
    Sensor_Self_Test();
    
    // 初始化传感器融合
    Sensor_Fusion_Init();
    
    // 启动系统
    system_state = SYS_READY;
    system_uptime = 0;
    sensor_ready = 1;
    
    printf("System initialized successfully!\r\n");
    
    // 启动成功指示
    for(int i = 0; i < 3; i++)
	{
        LED1_ON();
		LED2_ON();
		LED3_ON();
        Delay_ms(100);
        LED1_OFF();
		LED2_OFF();
		LED3_OFF();
        Delay_ms(100);
    }
}

/** 函  数：传感器自检
  * 参  数：无
  * 返回值：无
  */
void Sensor_Self_Test(void)
{
    printf("=== Sensor Self Test ===\r\n");
    
    // MPU6050自检
    printf("MPU6050 self test: ");
    if(MPU6050_Self_Test())
	{
        printf("PASS\r\n");
    }
	else
	{
        printf("FAIL\r\n");
        error_code |= ERROR_MPU6050_SELFTEST;
    }
    
    // 超声波自检
    printf("Ultrasonic test: ");
    Ultrasonic_Trigger();
    Delay_ms(100);
    if(ultrasonic_data.valid)
	{
        printf("PASS (Distance: %dmm)\r\n", ultrasonic_data.distance_mm);
    }
	else
	{
        printf("FAIL\r\n");
        error_code |= ERROR_ULTRASONIC_FAIL;
    }
    
    // 电压检测自检
    printf("Voltage detection test: ");
    Voltage_Update();
    Voltage_Data volt = Voltage_GetData();
    if(volt.voltage_v > 5.0f && volt.voltage_v < 15.0f)
	{
        printf("PASS (Voltage: %.2fV)\r\n", volt.voltage_v);
    }
	else
	{
        printf("FAIL (Voltage: %.2fV)\r\n", volt.voltage_v);
        error_code |= ERROR_VOLTAGE_FAIL;
    }
    
    printf("=======================\r\n");
}

/** 函  数：传感器融合初始化
  * 参  数：无
  * 返回值：无
  */
void Sensor_Fusion_Init(void)
{
    // 初始化传感器融合数据结构
    sensor_fusion.distance_mm = 0;
    sensor_fusion.distance_cm = 0.0f;
    sensor_fusion.roll = 0.0f;
    sensor_fusion.pitch = 0.0f;
    sensor_fusion.yaw = 0.0f;
    sensor_fusion.voltage_v = 0.0f;
    sensor_fusion.temperature_c = 0.0f;
    sensor_fusion.valid = 0;
    sensor_fusion.timestamp = 0;
    
    printf("Sensor fusion initialized\r\n");
}

/** 函  数：传感器融合处理
  * 参  数：无
  * 返回值：无
  */
void Sensor_Fusion_Process(void)
{
    static uint32_t last_fusion_time = 0;
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_fusion_time) / 1000.0f;  // 转换为秒
    
    if(dt < 0.01f)
	{  // 至少10ms
        return;
    }
    
    // 1. 更新MPU6050姿态
    MPU6050_Calculate_Attitude(dt);
    MPU6050_Data mpu = MPU6050_GetData();
    
    // 2. 更新超声波
    Ultrasonic_Update();
    
    // 3. 更新电压
    Voltage_Update();
    Voltage_Data volt = Voltage_GetData();
    
    // 4. 融合数据
    sensor_fusion.distance_mm = ultrasonic_data.distance_mm;
    sensor_fusion.distance_cm = ultrasonic_data.distance_cm;
    sensor_fusion.roll = mpu.roll;
    sensor_fusion.pitch = mpu.pitch;
    sensor_fusion.yaw = mpu.yaw;
    sensor_fusion.voltage_v = volt.filtered_v;
    sensor_fusion.temperature_c = mpu.temperature_c;
    sensor_fusion.valid = ultrasonic_data.valid;
    sensor_fusion.timestamp = current_time;
    
    // 5. 异常检测
    Sensor_Anomaly_Detection();
    
    // 6. 数据滤波
    Sensor_Data_Filtering();
    
    last_fusion_time = current_time;
}

/** 函  数：传感器异常检测
  * 参  数：无
  * 返回值：无
  */
void Sensor_Anomaly_Detection(void)
{
    static uint8_t anomaly_count = 0;
    uint8_t anomaly_detected = 0;
    
    // 检查超声波数据
    if(ultrasonic_data.distance_mm < 20)
	{  // 小于2cm
        if(++anomaly_count > 5)
		{
            printf("Warning: Object too close! (%dmm)\r\n", ultrasonic_data.distance_mm);
            anomaly_detected = 1;
        }
    }
	else if(ultrasonic_data.distance_mm > 4000)
	{  // 大于4m
        if(++anomaly_count > 5)
		{
            printf("Warning: Ultrasonic reading out of range! (%dmm)\r\n", ultrasonic_data.distance_mm);
            anomaly_detected = 1;
        }
    }
	else
	{
        anomaly_count = 0;
    }
    
    // 检查姿态角度
    if(fabs(sensor_fusion.roll) > 45.0f || fabs(sensor_fusion.pitch) > 45.0f)
	{
        printf("Warning: Excessive tilt! Roll=%.1f, Pitch=%.1f\r\n", 
               sensor_fusion.roll, sensor_fusion.pitch);
        anomaly_detected = 1;
    }
    
    // 检查电压
    if(sensor_fusion.voltage_v < VOLTAGE_MIN)
	{
        printf("Warning: Low voltage! %.2fV\r\n", sensor_fusion.voltage_v);
        anomaly_detected = 1;
    }
	else if(sensor_fusion.voltage_v > VOLTAGE_MAX)
	{
        printf("Warning: High voltage! %.2fV\r\n", sensor_fusion.voltage_v);
        anomaly_detected = 1;
    }
    
    if(anomaly_detected)
	{
        LED3_ON();  // 报警指示
    }
	else
	{
        LED3_OFF();
    }
}

/** 函  数：数据滤波
  * 参  数：无
  * 返回值：无
  */
void Sensor_Data_Filtering(void)
{
    static float filtered_distance = 0;
    static float filtered_roll = 0;
    static float filtered_pitch = 0;
    static float filtered_yaw = 0;
    
    if(filtered_distance == 0)
	{
        filtered_distance = sensor_fusion.distance_cm;
        filtered_roll = sensor_fusion.roll;
        filtered_pitch = sensor_fusion.pitch;
        filtered_yaw = sensor_fusion.yaw;
    }
	else
	{
        // 低通滤波
        float alpha = 0.7f;  // 滤波系数
        
        filtered_distance = alpha * filtered_distance + (1 - alpha) * sensor_fusion.distance_cm;
        filtered_roll = alpha * filtered_roll + (1 - alpha) * sensor_fusion.roll;
        filtered_pitch = alpha * filtered_pitch + (1 - alpha) * sensor_fusion.pitch;
        filtered_yaw = alpha * filtered_yaw + (1 - alpha) * sensor_fusion.yaw;
        
        sensor_fusion.filtered_distance_cm = filtered_distance;
        sensor_fusion.filtered_roll = filtered_roll;
        sensor_fusion.filtered_pitch = filtered_pitch;
        sensor_fusion.filtered_yaw = filtered_yaw;
    }
}

int main(void) 
{
    System_Init();
	
    while(1) 
	{
        
		
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
