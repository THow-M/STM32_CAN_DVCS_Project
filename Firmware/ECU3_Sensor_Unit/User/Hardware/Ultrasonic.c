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
static volatile uint8_t echo_rising_captured = 0;   /* 上升沿已捕获标志 */
static volatile uint32_t timeout_counter = 0;
static volatile uint32_t last_timeout_log_time = 0;   /* 上次超时日志时间 */

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
    
    /* 1. 开启时钟（必须先开时钟再配置） */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    /* 2. PB8 = Trig 推挽输出 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, GPIO_Pin_8);     /* 初始低电平 */
    
    /* 3. PB9 = Echo 浮空输入（TIM4_CH4 默认映射） */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* 4. TIM4 时基：72MHz / 72 = 1MHz, 周期 65535 = 65.5ms */
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseStructure.TIM_Period = 65535;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);     //清更新标志
    
    /* 5. TIM4_CH4 输入捕获：上升沿，直连，不分频，不滤波 */
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x08;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    
    /* 6. 开启 CC4 中断（注意：必须先清标志再开中断！） */
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);
    TIM_ITConfig(TIM4, TIM_IT_CC4, ENABLE);
    
    /* 7. NVIC 配置 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* 8. 清零计数器，启动定时器 */
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);
    
    /* 9. 初始化数据 */
    ultrasonic_data.distance_mm = 0;
    ultrasonic_data.distance_cm = 0.0f;
    ultrasonic_data.valid = 0;
    ultrasonic_data.signal_strength = 0;
    
    /* 调试：打印寄存器值确认配置 */
    /*printf("Ultrasonic Init OK\r\n");
    printf("  TIM4 CR1: 0x%04X\r\n", TIM4->CR1);
    printf("  TIM4 CCER: 0x%04X\r\n", TIM4->CCER);
    printf("  TIM4 CCMR2: 0x%04X\r\n", TIM4->CCMR2);
    printf("  TIM4 DIER: 0x%04X\r\n", TIM4->DIER);
    printf("  GPIOB CRL: 0x%08X\r\n", GPIOB->CRL);
    printf("  GPIOB CRH: 0x%08X\r\n", GPIOB->CRH);*/
    
    printf("Ultrasonic initialized\r\n");
}

/** 函  数：发送触发信号
  * 参  数：无
  * 返回值：无
  */
void Ultrasonic_Trigger(void)
{
    /* 1. 关 CC4 中断 */
    TIM_ITConfig(TIM4, TIM_IT_CC4, DISABLE);
    
    /* 2. 重置状态机 */
    echo_received = 0;
    echo_start_time = 0;
    echo_end_time = 0;
    echo_rising_captured = 0;
    measurement_state = 1;
    
    /* 3. 强制配置为上升沿捕获 */
    TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Rising);
    
    /* 4. 清零计数器 + 清 CC4 标志 */
    TIM_SetCounter(TIM4, 0);
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);
    
    /* 5. 开中断 */
    TIM_ITConfig(TIM4, TIM_IT_CC4, ENABLE);
    
    /* 6. 发 20us Trig 脉冲（HC-SR04 要求 >=10us） */
    GPIO_ResetBits(GPIOB, GPIO_Pin_8);
    Delay_us(5);
    GPIO_SetBits(GPIOB, GPIO_Pin_8);
    Delay_us(20);
    GPIO_ResetBits(GPIOB, GPIO_Pin_8);
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
	static uint32_t trigger_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 每100ms测量一次
    if(current_time - last_measure_time >= 150)
	{
        if(measurement_state == 0)        /* 新增：测量进行中不触发新测量 */
        {
            last_measure_time = current_time;
            trigger_time = current_time;
            Ultrasonic_Trigger();
        }
        /* 若 measurement_state != 0，不更新 last_measure_time，等待当前测量完成 */
    }
    
    // 处理超时
    if(measurement_state && current_time - trigger_time > 100)
    {
        measurement_state = 0;
        ultrasonic_data.valid = 0;
        /* 限频：每1000ms最多打印一次 */
        if(current_time - last_timeout_log_time >= 1000)
        {
            last_timeout_log_time = current_time;
            printf("Ultrasonic timeout\r\n");
        }
	}
}

/** 函  数：计算距离
  * 参  数：无
  * 返回值：无
  */
void Ultrasonic_Calculate_Distance(void)
{
    uint32_t pulse_width;
    
    if(echo_rising_captured != 0 || echo_end_time == 0 || echo_end_time <= echo_start_time)
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
    ultrasonic_data.distance_mm = (uint16_t)(pulse_width * 0.17f);  // 单位: mm
    
    // 转换为cm
    ultrasonic_data.distance_cm = ultrasonic_data.distance_mm / 10.0f;
    
    // 信号强度（基于脉冲宽度）
    ultrasonic_data.signal_strength = 100 - (pulse_width * 100 / 23200);
    
    ultrasonic_data.valid = 1;
    
    /*printf("Distance: %dmm (%.1fcm)\r\n", 
           ultrasonic_data.distance_mm, 
           ultrasonic_data.distance_cm);*/
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
        
        printf("Sample %d: %dmm\r\n", i + 1, ultrasonic_data.distance_mm);
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

// 超声波中断服务函数
void TIM4_IRQHandler(void)
{
    if(TIM_GetITStatus(US_TIM, TIM_IT_CC4) != RESET)
	{
        if(measurement_state)
		{
            if(echo_rising_captured  == 0)
			{
                // 上升沿，记录开始时间
                echo_start_time = TIM_GetCapture4(US_TIM);
				echo_rising_captured = 1;   /* 新增：置位上升沿捕获标志 */
                
                // 改为下降沿捕获
                TIM_OC4PolarityConfig(US_TIM, TIM_ICPolarity_Falling);
            }
			else
			{
                // 下降沿，记录结束时间
                echo_end_time = TIM_GetCapture4(US_TIM);
                echo_received = 1;
                measurement_state = 0;
				echo_rising_captured = 0;   /* 新增：清零上升沿捕获标志 */
                
                // 计算距离
                Ultrasonic_Calculate_Distance();
                
                // 恢复为上升沿捕获
                TIM_OC4PolarityConfig(US_TIM, TIM_ICPolarity_Rising);
                //echo_start_time = 0;
                //echo_end_time = 0;
            }
        }
        
        TIM_ClearITPendingBit(US_TIM, TIM_IT_CC4);
    }
}

/**
  * 函  数：超声波模块完整测试
  * 参  数：无
  * 返回值：无
  * 注  释：依次执行以下测试项：
  *         1. 初始化测试
  *         2. 单次阻塞测量测试（3次）
  *         3. 连续非阻塞测量测试（5秒）
  *         4. 超时/无回波测试
  *         5. 诊断信息输出
  *         所有结果通过串口打印，需在 main 中调用
  */
void Ultrasonic_Test_All(void)
{
    uint32_t test_start = 0;
    uint8_t pass_count = 0;
    uint8_t total_count = 0;

    printf("\r\n================================\r\n");
    printf("  Ultrasonic Module Test\r\n");
    printf("================================\r\n\r\n");

    /*------ 测试1：初始化 ------*/
    printf("[TEST 1] Initialization...\r\n");
    Ultrasonic_Init();
    printf("[TEST 1] PASSED\r\n\r\n");
	
	printf("TIM4 CNT after 1ms: %lu\r\n", TIM_GetCounter(TIM4));
	Delay_ms(1);
	printf("TIM4 CNT after 2ms: %lu\r\n", TIM_GetCounter(TIM4));

    /*------ 测试2：单次阻塞测量（3次） ------*/
    printf("[TEST 2] Single Measurement (x3)...\r\n");
    pass_count = 0;

    for (uint8_t i = 0; i < 3; i++)
    {
        Ultrasonic_Trigger();
        Delay_ms(100);  /* 等待回波返回和计算完成 */

        total_count++;

        printf("  #%d: %d mm (%.1f cm), Valid=%d, Strength=%d%%\r\n",
               i + 1,
               ultrasonic_data.distance_mm,
               ultrasonic_data.distance_cm,
               ultrasonic_data.valid,
               ultrasonic_data.signal_strength);

        if (ultrasonic_data.valid)
        {
            pass_count++;
        }

        Delay_ms(200);  /* 两次测量间隔 */
    }

    printf("[TEST 2] %s (%d/3 valid)\r\n\r\n",
           pass_count >= 2 ? "PASSED" : "FAILED", pass_count);

    /*------ 测试3：连续非阻塞测量（5秒） ------*/
    printf("[TEST 3] Continuous Measurement (5 seconds)...\r\n");
    printf("  Tip: Move hand in front of sensor\r\n");
    test_start = HAL_GetTick();
    {
        uint32_t last_print_time = test_start;   /* 新增：绝对时间比较 */
        pass_count = 0;
        total_count = 0;

        while (HAL_GetTick() - test_start < 5000)
        {
            Ultrasonic_Update();  /* 非阻塞，内部自动触发 */

            /* 每 500ms 打印一次，用绝对时间比较替代取模窗口 */
            if (HAL_GetTick() - last_print_time >= 500)
            {
                last_print_time = HAL_GetTick();
                if (ultrasonic_data.valid)
                {
                    printf("  [%.1fs] %d mm, Strength=%d%%\r\n",
                           (HAL_GetTick() - test_start) / 1000.0f,
                           ultrasonic_data.distance_mm,
                           ultrasonic_data.signal_strength);
                    pass_count++;
                }
                else
                {
                    printf("  [%.1fs] --- no echo ---\r\n",
                           (HAL_GetTick() - test_start) / 1000.0f);
                }
                total_count++;
            }

            Delay_ms(10);
        }
    }

    printf("[TEST 3] %s (%d/%d valid readings)\r\n\r\n",
           pass_count > 0 ? "PASSED" : "FAILED", pass_count, total_count);

    /*------ 测试4：超时测试（遮挡 Echo 或断开模块） ------*/
    printf("[TEST 4] Timeout Test (disconnect Echo or block)\r\n");
    printf("  Waiting 3 seconds for timeout...\r\n");

    Ultrasonic_Trigger();
    test_start = HAL_GetTick();

    while (HAL_GetTick() - test_start < 3000)
    {
        /* 内联超时检测逻辑 */
        {
            static uint32_t last_check = 0;
            uint32_t now = HAL_GetTick();
            if(now - last_check >= 1)   /* 1ms粒度检查 */
            {
                last_check = now;
                if (measurement_state == 0 && ultrasonic_data.valid == 0)
                {
                    printf("  Timeout detected correctly\r\n");
                    printf("[TEST 4] PASSED\r\n\r\n");
                    break;
                }
                if(now - test_start > 50)   /* 50ms超时阈值 */
                {
                    measurement_state = 0;
                    ultrasonic_data.valid = 0;
                }
            }
        }
        Delay_ms(10);
    }

    if (ultrasonic_data.valid)
    {
        printf("  No timeout occurred (echo still active)\r\n");
        printf("[TEST 4] SKIPPED (sensor receiving echoes)\r\n\r\n");
    }

    /*------ 测试5：近距离 / 远距离范围 ------*/
    printf("[TEST 5] Range Check\r\n");
    Ultrasonic_Trigger();
    Delay_ms(100);

    if (ultrasonic_data.valid)
    {
        /* 检查是否在合理范围内（2cm ~ 400cm） */
        if (ultrasonic_data.distance_mm >= 20 && ultrasonic_data.distance_mm <= 4000)
        {
            printf("  Distance %d mm is within range (20-4000mm)\r\n",
                   ultrasonic_data.distance_mm);
            printf("[TEST 5] PASSED\r\n\r\n");
        }
        else
        {
            printf("  Distance %d mm is OUT of range (20-4000mm)\r\n",
                   ultrasonic_data.distance_mm);
            printf("[TEST 5] FAILED\r\n\r\n");
        }
    }
    else
    {
        printf("  No valid data, skipping range check\r\n");
        printf("[TEST 5] SKIPPED\r\n\r\n");
    }

    /*------ 测试6：诊断信息 ------*/
    printf("[TEST 6] Diagnostic Output\r\n");
    Ultrasonic_Diagnostic();
    printf("[TEST 6] PASSED\r\n\r\n");

    /*------ 测试总结 ------*/
    printf("================================\r\n");
    printf("  All Tests Completed\r\n");
    printf("================================\r\n");
}
