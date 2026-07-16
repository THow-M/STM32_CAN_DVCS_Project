#include "stm32f10x.h"                  // Device header
#include "Ultrasonic.h"
#include "Delay.h"
#include "Serial.h"

// 超声波引脚定义
#define TRIG_PORT       GPIOB
#define TRIG_PIN        GPIO_Pin_8
#define TRIG_CLK        RCC_APB2Periph_GPIOB

#define ECHO_PORT       GPIOB
#define ECHO_PIN        GPIO_Pin_9
#define ECHO_CLK        RCC_APB2Periph_GPIOB

// 超声波定时器
#define US_TIM          TIM4
#define US_TIM_CLK      RCC_APB1Periph_TIM4
#define US_TIM_IRQn     TIM4_IRQn
#define US_TIM_Channel  TIM_Channel_4

// 全局变量
Ultrasonic_Data ultrasonic_data = {0};
static volatile uint32_t echo_start_time = 0;
static volatile uint32_t echo_end_time = 0;
static volatile uint8_t echo_received = 0;
static volatile uint8_t measurement_state = 0;
static volatile uint32_t timeout_counter = 0;

/** 函  数：超声波初始化
  * 参  数：无
  * 返回值：无
  */
void Ultrasonic_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 1. 开启时钟
    RCC_APB2PeriphClockCmd(TRIG_CLK | ECHO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(US_TIM_CLK, ENABLE);
    
    // 2. 配置Trig引脚为推挽输出
    GPIO_InitStructure.GPIO_Pin = TRIG_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TRIG_PORT, &GPIO_InitStructure);
    
    // 3. 配置Echo引脚为浮空输入
    GPIO_InitStructure.GPIO_Pin = ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ECHO_PORT, &GPIO_InitStructure);
    
    // 4. 初始化定时器4
    // 定时器时钟=72MHz，预分频72，计数频率=1MHz，1个计数=1us
    TIM_TimeBaseStructure.TIM_Period = 65535;
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(US_TIM, &TIM_TimeBaseStructure);
    
    // 5. 配置输入捕获通道4
    TIM_ICInitStructure.TIM_Channel = US_TIM_Channel;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;  // 上升沿捕获
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x00;  // 不滤波
    TIM_ICInit(US_TIM, &TIM_ICInitStructure);
    
    // 6. 配置中断
    TIM_ITConfig(US_TIM, TIM_IT_CC4, ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = US_TIM_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 7. 使能定时器
    TIM_Cmd(US_TIM, ENABLE);
    
    // 8. 初始化数据
    ultrasonic_data.distance_mm = 0;
    ultrasonic_data.distance_cm = 0.0f;
    ultrasonic_data.valid = 1;
    ultrasonic_data.signal_strength = 100;
    
    printf("Ultrasonic initialized\r\n");
}

/** 函  数：发送触发信号
  * 参  数：无
  * 返回值：无
  */
void Ultrasonic_Trigger(void)
{
    // 确保Echo为低电平
    GPIO_ResetBits(TRIG_PORT, TRIG_PIN);
    Delay_us(2);
    
    // 发送10us的高电平触发脉冲
    GPIO_SetBits(TRIG_PORT, TRIG_PIN);
    Delay_us(10);
    GPIO_ResetBits(TRIG_PORT, TRIG_PIN);
    
    // 重置状态
    echo_received = 0;
    measurement_state = 1;
    timeout_counter = 0;
    
    // 重置定时器
    TIM_SetCounter(US_TIM, 0);
    echo_start_time = 0;
    echo_end_time = 0;
}

/** 函  数：获取距离
  * 参  数：无
  * 返回值：distance_mm 测得的距离
  */
uint16_t Ultrasonic_GetDistance(void)
{
    return ultrasonic_data.distance_mm;
}

/** 函  数：更新超声波数据
  * 参  数：无
  * 返回值：无
  */
void Ultrasonic_Update(void)
{
    static uint32_t last_measure_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 每100ms测量一次
    if(current_time - last_measure_time >= 100)
	{
        last_measure_time = current_time;
        
        if(ultrasonic_data.valid)
		{
            Ultrasonic_Trigger();
        }
    }
    
    // 处理超时
    if(measurement_state && current_time - last_measure_time > 50)
	{  // 50ms超时
        measurement_state = 0;
        ultrasonic_data.valid = 0;
        printf("Ultrasonic timeout\r\n");
    }
}

/** 函  数：计算距离
  * 参  数：无
  * 返回值：无
  */
void Ultrasonic_Calculate_Distance(void)
{
    uint32_t pulse_width;
    
    if(echo_start_time == 0 || echo_end_time == 0 || echo_end_time <= echo_start_time)
	{
        ultrasonic_data.valid = 0;
        return;
    }
    
    // 计算高电平时间（us）
    pulse_width = echo_end_time - echo_start_time;
    
    // 检查是否在有效范围内
    if(pulse_width < 116 || pulse_width > 23200)
	{  // 2cm-400cm
        ultrasonic_data.valid = 0;
        return;
    }
    
    // 计算距离
    // 声速: 340m/s = 0.034cm/us
    // 距离 = 时间 * 声速 / 2 (往返时间)
    ultrasonic_data.distance_mm = (uint16_t)(pulse_width * 0.017f);  // 单位: mm
    
    // 转换为cm
    ultrasonic_data.distance_cm = ultrasonic_data.distance_mm / 10.0f;
    
    // 信号强度（基于脉冲宽度）
    if(pulse_width > 23200) pulse_width = 23200;  // 最大400cm
    ultrasonic_data.signal_strength = 100 - (pulse_width * 100 / 23200);
    
    ultrasonic_data.valid = 1;
    
    printf("Distance: %dmm (%.1fcm)\r\n", 
           ultrasonic_data.distance_mm, 
           ultrasonic_data.distance_cm);
}

/** 函  数：超声波校准
  * 参  数：无
  * 返回值：无
  */
void Ultrasonic_Calibrate(void)
{
    printf("Ultrasonic calibration started...\r\n");
    printf("Please place object at known distance (e.g., 100mm)\r\n");
    
    uint32_t sum = 0;
    uint16_t samples = 20;
    uint16_t valid_samples = 0;
    
    for(uint16_t i = 0; i < samples; i++)
	{
        Ultrasonic_Trigger();
        Delay_ms(100);
        
        if(ultrasonic_data.valid)
		{
            sum += ultrasonic_data.distance_mm;
            valid_samples++;
        }
        
        printf("Sample %d: %dmm\n", i + 1, ultrasonic_data.distance_mm);
    }
    
    if(valid_samples > 0)
	{
        uint16_t avg_distance = sum / valid_samples;
        printf("Average distance: %dmm\r\n", avg_distance);
        printf("Expected distance: 100mm\r\n");
        printf("Calibration factor: %.3f\r\n", 100.0f / avg_distance);
    }
	else
	{
        printf("No valid samples received\r\n");
    }
}

/** 函  数：超声波诊断
  * 参  数：无
  * 返回值：无
  */
void Ultrasonic_Diagnostic(void)
{
    printf("=== Ultrasonic Diagnostic ===\r\n");
    printf("Distance: %dmm (%.1fcm)\r\n", 
           ultrasonic_data.distance_mm, 
           ultrasonic_data.distance_cm);
    printf("Valid: %s\r\n", ultrasonic_data.valid ? "Yes" : "No");
    printf("Signal Strength: %d%%\r\n", ultrasonic_data.signal_strength);
    printf("Echo Start: %lu us\r\n", echo_start_time);
    printf("Echo End: %lu us\r\n", echo_end_time);
    printf("============================\r\n");
}
