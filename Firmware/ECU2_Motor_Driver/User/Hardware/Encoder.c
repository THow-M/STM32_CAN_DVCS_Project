#include "stm32f10x.h"                  // Device header
#include "Encoder.h"
#include "Serial.h"
#include "Delay.h"

// 编码器定时器配置
#define ENCODER_TIM              TIM3
#define ENCODER_TIM_CLK          RCC_APB1Periph_TIM3
#define ENCODER_GPIO_CLK         RCC_APB2Periph_GPIOB
#define ENCODER_GPIO             GPIOB
#define ENCODER_PIN_A            GPIO_Pin_4  // PB4 - TIM3_CH1
#define ENCODER_PIN_B            GPIO_Pin_5  // PB5 - TIM3_CH2

// 编码器全局变量
Encoder_Data encoder_data = {0};
int32_t encoder_total_pulses = 0;    // 总脉冲数
int32_t encoder_last_count = 0;      // 上次计数值
uint32_t last_speed_time = 0;               // 上次测速时间

// 编码器初始化
void Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    
    // 1. 开启时钟
    RCC_APB2PeriphClockCmd(ENCODER_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(ENCODER_TIM_CLK, ENABLE);
    
    // 2. 配置GPIO为浮空输入
    GPIO_InitStructure.GPIO_Pin = ENCODER_PIN_A | ENCODER_PIN_B;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ENCODER_GPIO, &GPIO_InitStructure);
    
    // 3. 定时器基础配置
    TIM_TimeBaseStructure.TIM_Period = 65535;  // 自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler = 0;   // 预分频器
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(ENCODER_TIM, &TIM_TimeBaseStructure);
    
    // 4. 编码器接口配置
    // 使用TI1和TI2，4倍频模式
    TIM_EncoderInterfaceConfig(ENCODER_TIM, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    
    // 5. 输入捕获配置
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 0x0F;  // 滤波，抗干扰
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(ENCODER_TIM, &TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(ENCODER_TIM, &TIM_ICInitStructure);
    
    // 6. 使能定时器
    TIM_Cmd(ENCODER_TIM, ENABLE);
    
    // 7. 重置计数器
    TIM_SetCounter(ENCODER_TIM, 32768);  // 设置为中间值，支持正反转
    
    // 8. 初始化编码器数据
    encoder_data.speed_rpm = 0;
    encoder_data.position = 0;
    encoder_data.direction = 0;
    encoder_data.pulse_count = 0;
    encoder_data.valid = 1;
    
    last_speed_time = HAL_GetTick();
    encoder_last_count = 32768;
    
    printf("Encoder initialized successfully.\r\n");
}

// 获取编码器计数值
int32_t Encoder_GetCount(void)
{
    int32_t count = TIM_GetCounter(ENCODER_TIM);
    int32_t diff = 0;
    
    // 处理溢出
    if(count >= encoder_last_count)
	{
        diff = count - encoder_last_count;
    }
	else
	{
        diff = 65536 - encoder_last_count + count;
    }
    
    // 判断方向
    if(diff > 32768)
	{
        // 反转
        diff = diff - 65536;
    }
    
    encoder_total_pulses += diff;
    encoder_last_count = count;
    
    return diff;  // 返回相对变化量
}

// 获取总脉冲数
int32_t Encoder_GetTotalPulses(void)
{
    return encoder_total_pulses;
}

// 计算速度（RPM）
float Encoder_CalculateSpeed(uint16_t sample_time_ms)
{
    static uint32_t last_calc_time = 0;
    static int32_t last_total_pulses = 0;
    
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed_time = current_time - last_calc_time;
    
    if(elapsed_time < sample_time_ms)
	{
        return encoder_data.speed_rpm;  // 返回上次计算的速度
    }
    
    int32_t current_pulses = Encoder_GetTotalPulses();
    int32_t pulse_diff = current_pulses - last_total_pulses;
    
    // 计算速度
    // 公式：转速(RPM) = (脉冲数 / (编码器线数 * 4)) * (60000 / 采样时间) / 减速比
    // 假设：编码器13线，减速比30:1
    float speed_rpm = (pulse_diff / (13.0f * 4.0f)) * (60000.0f / elapsed_time) / 30.0f;
    
    // 更新数据
    encoder_data.speed_rpm = speed_rpm;
    encoder_data.pulse_count = current_pulses;
    
    // 判断方向
    if(pulse_diff > 0)
	{
        encoder_data.direction = 1;  // 正转
    }
	else if(pulse_diff < 0)
	{
        encoder_data.direction = 2;  // 反转
    }
	else
	{
        encoder_data.direction = 0;  // 停止
    }
    
    // 更新位置（假设每个脉冲对应一定的位移）
    // 需要根据实际机械结构计算
    // 假设：轮子周长 = 2 * π * 半径，编码器每转脉冲数 = 线数 * 4 * 减速比
    // 这里简化处理
    encoder_data.position += pulse_diff;
    
    // 保存本次数据
    last_total_pulses = current_pulses;
    last_calc_time = current_time;
    
    return speed_rpm;
}
