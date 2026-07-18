#include "stm32f10x.h"                  // Device header
#include "Voltage.h"
#include "Delay.h"
#include "Serial.h"

// ADC配置
#define ADC_PORT        GPIOA
#define ADC_PIN         GPIO_Pin_1
#define ADC_CHANNEL     ADC_Channel_1
#define ADC_CLK         RCC_APB2Periph_GPIOA
#define ADC_ADC_CLK     RCC_APB2Periph_ADC1

// 电压分压比
// 假设：R1=10k, R2=2k, 12V时分压=12*(2/(10+2))=2V
#define VOLTAGE_DIVIDER_RATIO  6.0f  // 12V/2V = 6
#define ADC_REF_VOLTAGE        3.3f  // ADC参考电压
#define ADC_RESOLUTION         4096  // 12位ADC

// 全局变量
Voltage_Data voltage_data = {0};
static uint16_t adc_samples[ADC_SAMPLE_COUNT] = {0};
static uint8_t sample_index = 0;

// ADC初始化
void Voltage_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    
    // 1. 开启时钟
    RCC_APB2PeriphClockCmd(ADC_CLK | ADC_ADC_CLK, ENABLE);
	
	//RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    
    // 2. 配置PA1为模拟输入
    GPIO_InitStructure.GPIO_Pin = ADC_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ADC_PORT, &GPIO_InitStructure);
    
    // 3. ADC初始化(单次转换、非扫描）
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    
    // 4. 配置ADC通道1，采样时间239.5周期
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL, 1, ADC_SampleTime_239Cycles5);
    
    // 5. 使能ADC
    ADC_Cmd(ADC1, ENABLE);
    
    // 6. ADC校准
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    
    printf("Voltage detection initialized\n");
}

// 读取ADC值
uint16_t Voltage_Read_ADC(void)
{
    // 启动转换
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
    // 等待转换完成
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    
    // 读取转换值
    return ADC_GetConversionValue(ADC1);
}
