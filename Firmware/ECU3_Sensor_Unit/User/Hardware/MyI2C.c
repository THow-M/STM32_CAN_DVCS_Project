#include "stm32f10x.h"                  // Device header
#include "MyI2C.h"
#include "Delay.h"
#include "Serial.h"

// I2C引脚定义
#define I2C_PORT        GPIOB
#define I2C_SCL_PIN     GPIO_Pin_10
#define I2C_SDA_PIN     GPIO_Pin_11
#define I2C_CLK         RCC_APB2Periph_GPIOB
#define I2C_SPEED       100000  // 100kHz

// 软件I2C延迟
#define I2C_DELAY()     Delay_us(5)

// I2C初始化
void MyI2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 开启GPIOB时钟
    RCC_APB2PeriphClockCmd(I2C_CLK, ENABLE);
    
    // 配置PB10(SCL)和PB11(SDA)为开漏输出
    GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  // 开漏输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(I2C_PORT, &GPIO_InitStructure);
    
    // 初始状态：SCL和SDA都为高电平
    GPIO_SetBits(I2C_PORT, I2C_SCL_PIN | I2C_SDA_PIN);
    
    printf("I2C initialized (Software)\n");
}

// 产生起始信号
void MyI2C_Start(void)
{
    SDA_OUT();  // SDA设置为输出
    I2C_SDA_HIGH();
    I2C_SCL_HIGH();
    I2C_DELAY();
    
    I2C_SDA_LOW();
    I2C_DELAY();
    
    I2C_SCL_LOW();
    I2C_DELAY();
}

// 产生停止信号
void MyI2C_Stop(void)
{
    SDA_OUT();  // SDA设置为输出
    I2C_SDA_LOW();
    I2C_DELAY();
    
    I2C_SCL_HIGH();
    I2C_DELAY();
    
    I2C_SDA_HIGH();
    I2C_DELAY();
}

// 等待ACK
uint8_t MyI2C_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    
    SDA_IN();  // SDA设置为输入
    I2C_SDA_HIGH();  // 释放SDA
    I2C_DELAY();
    
    I2C_SCL_HIGH();
    I2C_DELAY();
    
    while(READ_SDA())
	{
        ucErrTime++;
        if(ucErrTime > 250)
		{
            MyI2C_Stop();
            return 1;  // 无ACK
        }
    }
    
    I2C_SCL_LOW();
    return 0;  // 收到ACK
}

// 发送ACK
void MyI2C_Ack(void)
{
    SDA_OUT();  // SDA设置为输出
    I2C_SDA_LOW();
    I2C_DELAY();
    
    I2C_SCL_HIGH();
    I2C_DELAY();
    
    I2C_SCL_LOW();
    I2C_DELAY();
}

// 发送NACK
void MyI2C_NAck(void)
{
    SDA_OUT();  // SDA设置为输出
    I2C_SDA_HIGH();
    I2C_DELAY();
    
    I2C_SCL_HIGH();
    I2C_DELAY();
    
    I2C_SCL_LOW();
    I2C_DELAY();
}

// 发送一个字节
void MyI2C_Send_Byte(uint8_t data)
{
    uint8_t i;
    SDA_OUT();  // SDA设置为输出
    
    for(i = 0; i < 8; i++)
	{
        if(data & 0x80)
		{
            I2C_SDA_HIGH();
        }
		else
		{
            I2C_SDA_LOW();
        }
        
        I2C_DELAY();
        I2C_SCL_HIGH();
        I2C_DELAY();
        I2C_SCL_LOW();
        I2C_DELAY();
        
        data <<= 1;
    }
}

// 接收一个字节
uint8_t MyI2C_Read_Byte(uint8_t ack)
{
    uint8_t i, receive = 0;
    SDA_IN();  // SDA设置为输入
    
    for(i = 0; i < 8; i++)
	{
        receive <<= 1;
        
        I2C_SCL_HIGH();
        I2C_DELAY();
        
        if(READ_SDA())
		{
            receive |= 0x01;
        }
        
        I2C_SCL_LOW();
        I2C_DELAY();
    }
    
    if(ack)
	{
        MyI2C_Ack();
    }
	else
	{
        MyI2C_NAck();
    }
    
    return receive;
}

// 向指定设备写入一个字节
uint8_t MyI2C_Write_Byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    MyI2C_Start();
    
    // 发送设备地址（写模式）
    MyI2C_Send_Byte(dev_addr & 0xFE);  // 清空最后一位（写）
    if(MyI2C_Wait_Ack())
	{
        MyI2C_Stop();
        return 1;  // 失败
    }
    
    // 发送寄存器地址
    MyI2C_Send_Byte(reg_addr);
    if(MyI2C_Wait_Ack())
	{
        MyI2C_Stop();
        return 2;  // 失败
    }
    
    // 发送数据
    MyI2C_Send_Byte(data);
    if(MyI2C_Wait_Ack())
	{
        MyI2C_Stop();
        return 3;  // 失败
    }
    
    MyI2C_Stop();
    return 0;  // 成功
}

// 从指定设备读取一个字节
uint8_t MyI2C_Read_Byte_From_Reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
    MyI2C_Start();
    
    // 发送设备地址（写模式）
    MyI2C_Send_Byte(dev_addr & 0xFE);  // 清空最后一位（写）
    if(MyI2C_Wait_Ack())
	{
        MyI2C_Stop();
        return 1;  // 失败
    }
    
    // 发送寄存器地址
    MyI2C_Send_Byte(reg_addr);
    if(MyI2C_Wait_Ack())
	{
        MyI2C_Stop();
        return 2;  // 失败
    }
    
    // 重新启动
    MyI2C_Start();
    
    // 发送设备地址（读模式）
    MyI2C_Send_Byte(dev_addr | 0x01);  // 设置最后一位（读）
    if(MyI2C_Wait_Ack())
	{
        MyI2C_Stop();
        return 3;  // 失败
    }
    
    // 读取数据
    *data = MyI2C_Read_Byte(0);  // 不发送ACK
    
    MyI2C_Stop();
    return 0;  // 成功
}
