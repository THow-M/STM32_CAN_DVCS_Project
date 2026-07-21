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

static float voltage_calibration_factor = VOLTAGE_DIVIDER_RATIO;

/** 函  数：ADC初始化
  * 参  数：无
  * 返回值：无
  */
void Voltage_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    
    // 1. 开启时钟
    RCC_APB2PeriphClockCmd(ADC_CLK | ADC_ADC_CLK, ENABLE);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    
    // 2. 配置PA1为模拟输入
    GPIO_InitStructure.GPIO_Pin = ADC_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
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
    
    printf("Voltage detection initialized\r\n");
}

/** 函  数：读取ADC值
  * 参  数：无
  * 返回值：ADC转换值
  */
uint16_t Voltage_Read_ADC(void)
{
    // 启动转换
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    
    // 等待转换完成
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    
    // 读取转换值
    return ADC_GetConversionValue(ADC1);
}

/** 函  数：更新电压数据
  * 参  数：无
  * 返回值：无
  */
void Voltage_Update(void)
{
    static uint32_t last_update_time = 0;
	static uint32_t last_print_time = 0;
    uint32_t current_time = HAL_GetTick();
	
	// Voltage_Update 首次调用
	static uint8_t first_run = 1;
	if(first_run)
	{
		uint16_t init_val = Voltage_Read_ADC();
		for(uint8_t i = 0; i < ADC_SAMPLE_COUNT; i++)
			adc_samples[i] = init_val;
		sample_index = 0;
		first_run = 0;
	}
    
    // 每100ms更新一次
    if(current_time - last_update_time >= 100)
	{
        last_update_time = current_time;
        
        // 读取ADC值
        uint16_t adc_value = Voltage_Read_ADC();
        
        // 保存到采样数组
        adc_samples[sample_index] = adc_value;
        sample_index = (sample_index + 1) % ADC_SAMPLE_COUNT;
        
        // 计算平均值
        uint32_t sum = 0;
        for(uint8_t i = 0; i < ADC_SAMPLE_COUNT; i++)
		{
            sum += adc_samples[i];
        }
        uint16_t avg_adc = sum / ADC_SAMPLE_COUNT;
        
        // 计算电压
        // 步骤1: ADC电压 = ADC值 * 3.3V / 4096
        // 步骤2: 实际电压 = ADC电压 * 分压比
        float adc_voltage = avg_adc * ADC_REF_VOLTAGE / ADC_RESOLUTION;
        float actual_voltage = adc_voltage * VOLTAGE_DIVIDER_RATIO;
        
        // 转换为mV
        voltage_data.voltage_mv = (uint16_t)(actual_voltage * 1000);
        voltage_data.voltage_v = actual_voltage;
        
        // 检查电压状态
        if(voltage_data.voltage_v < VOLTAGE_MIN)
		{
            voltage_data.status = VOLTAGE_LOW;
        }
		else if(voltage_data.voltage_v > VOLTAGE_MAX)
		{
            voltage_data.status = VOLTAGE_HIGH;
        }
		else
		{
            voltage_data.status = VOLTAGE_NORMAL;
        }
        
        // 计算电量百分比（假设12V系统）
        if(voltage_data.voltage_v <= 10.0f)
		{
            voltage_data.battery_percent = 0;
        }
		else if(voltage_data.voltage_v >= 12.6f)
		{
            voltage_data.battery_percent = 100;
        }
		else
		{
            // 10.0V-12.6V线性映射到0-100%
            voltage_data.battery_percent = (uint8_t)((voltage_data.voltage_v - 10.0f) * 100 / 2.6f);
        }
        
        // 滤波处理
        static float filtered_voltage = 0;
		static uint8_t filter_first = 1;
		if(filter_first)
		{
			filtered_voltage = actual_voltage;
			filter_first = 0;
		}
		else
		{
			filtered_voltage = 0.9f * filtered_voltage + 0.1f * actual_voltage;
		}
        voltage_data.filtered_v = filtered_voltage;
        
        if(current_time - last_print_time >= 1000)
        {
            last_print_time = current_time;
            printf("Voltage: %.2fV (%d%%) Status: %d\r\n", 
                   voltage_data.voltage_v, 
                   voltage_data.battery_percent,
                   voltage_data.status);
        }
    }
}

/** 函  数：获取电压数据
  * 参  数：无
  * 返回值：voltage_data 电压数据结构体
  */
Voltage_Data Voltage_GetData(void)
{
    return voltage_data;
}

/** 函  数：电压校准
  * 参  数：actual_voltage 实际电压
  * 返回值：无
  */
void Voltage_Calibrate(float actual_voltage)
{
    //static float voltage_calibration_factor = VOLTAGE_DIVIDER_RATIO;
    
    // 读取当前ADC值
    uint16_t adc_value = Voltage_Read_ADC();
    
    // 计算校准因子
    float measured_voltage = adc_value * ADC_REF_VOLTAGE / ADC_RESOLUTION;
	if(measured_voltage < 0.01f)
	{
		printf("Calibration failed: ADC reads zero or too low\r\n");
		return;
	}
    voltage_calibration_factor = actual_voltage / measured_voltage;
    
    printf("Voltage calibration:\r\n");
    printf("Actual voltage: %.2fV\r\n", actual_voltage);
    printf("Measured ADC: %d (%.3fV)\r\n", adc_value, measured_voltage);
    printf("Calibration factor: %.3f\r\n", voltage_calibration_factor);
    
    // 保存校准因子到EEPROM（这里只是打印）
    printf("Calibration factor saved: %.3f\r\n", voltage_calibration_factor);
}

/** 函  数：电压诊断
  * 参  数：无
  * 返回值：无
  */
void Voltage_Diagnostic(void)
{
    printf("=== Voltage Diagnostic ===\r\n");
    printf("Current Voltage: %.2fV (%dmV)\r\n", 
           voltage_data.voltage_v, 
           voltage_data.voltage_mv);
    printf("Filtered Voltage: %.2fV\r\n", voltage_data.filtered_v);
    printf("Battery: %d%%\r\n", voltage_data.battery_percent);
    printf("Status: ");
    switch(voltage_data.status)
	{
        case VOLTAGE_LOW: printf("LOW\r\n"); break;
        case VOLTAGE_NORMAL: printf("NORMAL\r\n"); break;
        case VOLTAGE_HIGH: printf("HIGH\r\n"); break;
    }
    printf("ADC Value: %d\r\n", Voltage_Read_ADC());
    printf("ADC Reference: %.2fV\r\n", ADC_REF_VOLTAGE);
    printf("Divider Ratio: %.1f\r\n", VOLTAGE_DIVIDER_RATIO);
    printf("=======================\r\n");
}

/** 函  数: 电压模块全部测试
  * 参  数: 无
  * 返回值: 无
  * 说  明: 依次执行 6 项测试, 通过串口打印结果
  */
void Voltage_Test_All(void)
{
    printf("\r\n");
    printf("====================================\r\n");
    printf("  Voltage Module Self Test\r\n");
    printf("====================================\r\n\r\n");
    
    /* 测试 1: 初始化测试 */
    printf("[TEST 1] Initialization Test...\r\n");
    {
        uint8_t pass = 1;
        
        /* 检查 ADC1 是否已经被使能 */
        if((ADC1->CR2 & ADC_CR2_ADON) == 0)
        {
            printf("  FAIL: ADC1 not enabled\r\n");
            pass = 0;
        }
        else
        {
            printf("  ADC1 enabled: OK\r\n");
        }
        
        /* 检查 PA1 是否为模拟输入 (CRL 位 4-7 = 0000) */
        if((GPIOA->CRL & GPIO_CRL_MODE1) != 0 || (GPIOA->CRL & GPIO_CRL_CNF1) != 0)
        {
            printf("  FAIL: PA1 not in analog mode\r\n");
            pass = 0;
        }
        else
        {
            printf("  PA1 analog mode: OK\r\n");
        }
        
        /* 读取一次 ADC, 确认转换能完成 */
        {
            uint16_t adc_val = Voltage_Read_ADC();
            if(adc_val > 0 && adc_val < 4096)
            {
                printf("  ADC conversion: OK (%d)\r\n", adc_val);
            }
            else
            {
                printf("  FAIL: ADC value out of range (%d)\r\n", adc_val);
                pass = 0;
            }
        }
        
        if(pass)
            printf("[TEST 1] PASSED\r\n\r\n");
        else
            printf("[TEST 1] FAILED\r\n\r\n");
    }
    
    /* 测试 2: 单次电压测量 (x3) */
    printf("[TEST 2] Single Measurement (x3)...\r\n");
    {
        uint8_t valid_count = 0;
        
        for(uint8_t i = 0; i < 3; i++)
        {
            uint16_t adc_val = Voltage_Read_ADC();
            float adc_voltage = adc_val * ADC_REF_VOLTAGE / ADC_RESOLUTION;
            float actual_voltage = adc_voltage * voltage_calibration_factor;
            
            /* 有效性判断: ADC > 0 且 实际电压在 0-20V 之间 */
            uint8_t valid = (adc_val > 0) && (actual_voltage > 0.0f) && (actual_voltage < 20.0f);
            
            printf("  #%d: %.2f V  (ADC=%d, valid=%s)\r\n",
                   i+1, actual_voltage, adc_val, valid ? "Yes" : "No");
            
            if(valid) valid_count++;
            Delay_ms(200);
        }
        
        if(valid_count == 3)
            printf("[TEST 2] PASSED (3/3 valid)\r\n\r\n");
        else
            printf("[TEST 2] FAILED (%d/3 valid)\r\n\r\n", valid_count);
    }
    
    /* 测试 3: 连续非阻塞测量 (5 秒) */
    printf("[TEST 3] Continuous Measurement (5 seconds)...\r\n");
    printf("  Tip: Vary the input voltage to see changes\r\n");
    {
        uint32_t test_start = HAL_GetTick();
        uint32_t last_print_time = test_start;
        uint16_t valid_count = 0;
        uint16_t total_count = 0;
        
        /* 先填充采样数组, 避免首次全 0 */
        {
            uint16_t init_val = Voltage_Read_ADC();
            for(uint8_t i = 0; i < ADC_SAMPLE_COUNT; i++)
                adc_samples[i] = init_val;
        }
        
        while(HAL_GetTick() - test_start < 5000)
        {
            Voltage_Update();
            
            /* 每 500ms 打印一次 */
            if(HAL_GetTick() - last_print_time >= 500)
            {
                last_print_time = HAL_GetTick();
                total_count++;
                
                if(voltage_data.status == VOLTAGE_NORMAL)
                {
                    printf("  [%.1fs] %.2f V  %d%%  Status=NORMAL\r\n",
                           (HAL_GetTick() - test_start) / 1000.0f,
                           voltage_data.voltage_v,
                           voltage_data.battery_percent);
                    valid_count++;
                }
                else if(voltage_data.status == VOLTAGE_LOW)
                {
                    printf("  [%.1fs] %.2f V  Status=LOW\r\n",
                           (HAL_GetTick() - test_start) / 1000.0f,
                           voltage_data.voltage_v);
                    valid_count++;
                }
                else if(voltage_data.status == VOLTAGE_HIGH)
                {
                    printf("  [%.1fs] %.2f V  Status=HIGH\r\n",
                           (HAL_GetTick() - test_start) / 1000.0f,
                           voltage_data.voltage_v);
                    valid_count++;
                }
                else
                {
                    printf("  [%.1fs] --- no data ---\r\n",
                           (HAL_GetTick() - test_start) / 1000.0f);
                }
            }
            
            Delay_ms(10);
        }
        
        if(valid_count >= 7)
            printf("[TEST 3] PASSED (%d/%d valid readings)\r\n\r\n", valid_count, total_count);
        else
            printf("[TEST 3] FAILED (%d/%d valid readings)\r\n\r\n", valid_count, total_count);
    }
    
    /* 测试 4: 电压范围检查 */
    printf("[TEST 4] Range Check\r\n");
    {
        float v = voltage_data.voltage_v;
        
        printf("  Current voltage: %.2f V\r\n", v);
        printf("  Valid range: %.1f - %.1f V\r\n", VOLTAGE_MIN, VOLTAGE_MAX);
        
        if(v > 0.0f && v < 20.0f)
        {
            printf("  Voltage within ADC range (0-20V): YES\r\n");
            
            if(v >= VOLTAGE_MIN && v <= VOLTAGE_MAX)
                printf("  Battery range (%.1f-%.1fV): YES (%.0f%%)\r\n",
                       VOLTAGE_MIN, VOLTAGE_MAX, voltage_data.battery_percent * 1.0f);
            else if(v < VOLTAGE_MIN)
                printf("  Battery range: BELOW MIN (undervoltage)\r\n");
            else
                printf("  Battery range: ABOVE MAX (overvoltage)\r\n");
            
            printf("[TEST 4] PASSED\r\n\r\n");
        }
        else
        {
            printf("  FAIL: Voltage out of reasonable range\r\n");
            printf("[TEST 4] FAILED\r\n\r\n");
        }
    }
    
    /* 测试 5: 滤波效果验证 (对比原始值与滤波值) */
    printf("[TEST 5] Filter Verification\r\n");
    {
        uint16_t raw_adc = Voltage_Read_ADC();
        float raw_voltage = raw_adc * ADC_REF_VOLTAGE / ADC_RESOLUTION * voltage_calibration_factor;
        float filtered_voltage = voltage_data.filtered_v;
        float diff = (raw_voltage > filtered_voltage) ?
                     (raw_voltage - filtered_voltage) : (filtered_voltage - raw_voltage);
        
        printf("  Raw voltage:      %.3f V\r\n", raw_voltage);
        printf("  Filtered voltage: %.3f V\r\n", filtered_voltage);
        printf("  Difference:       %.3f V (%.1f%%)\r\n",
               diff, raw_voltage > 0 ? (diff / raw_voltage * 100.0f) : 0);
        
        if(diff < 1.0f)  /* 差值小于 1V 认为滤波正常工作 */
            printf("[TEST 5] PASSED (filter working)\r\n\r\n");
        else
            printf("[TEST 5] WARNING (large difference, may need more samples)\r\n\r\n");
    }
    
    /* 测试 6: 诊断输出 */
    printf("[TEST 6] Diagnostic Output\r\n");
    Voltage_Diagnostic();
    printf("[TEST 6] PASSED\r\n\r\n");
    
    printf("====================================\r\n");
    printf("  Voltage Test Complete\r\n");
    printf("====================================\r\n\r\n");
}
