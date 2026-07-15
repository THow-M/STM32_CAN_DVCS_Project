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

/** 函  数：I2C初始化
  * 参  数：无
  * 返回值：无
  */
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
    
    printf("I2C initialized (Software)\r\n");
}

/** 函  数：产生起始信号
  * 参  数：无
  * 返回值：无
  */
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

/** 函  数：产生停止信号
  * 参  数：无
  * 返回值：无
  */
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

/** 函  数：等待ACK
  * 参  数：无
  * 返回值：0 收到应答位，1 未收到应答
  */
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

/** 函  数：发送ACK
  * 参  数：无
  * 返回值：无
  */
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

/** 函  数：发送NACK
  * 参  数：无
  * 返回值：无
  */
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

/** 函  数：发送一个字节
  * 参  数：data 要发送的数据
  * 返回值：无
  */
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

/** 函  数：接收一个字节
  * 参  数：ack 应答位
  * 返回值：receive 接收到的一个字节
  */
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

/** 函  数：向指定设备写入一个字节
  * 参  数：dev_addr 被写入设备的地址
  * 参  数：reg_addr 被写入寄存器的地址
  * 参  数：data 要写入的数据
  * 返回值：0 发送成功，1 发送设备地址失败，2 发送寄存器地址失败，3 发送数据失败
  */
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

/** 函  数：从指定设备读取一个字节
  * 参  数：dev_addr 被读取设备的地址
  * 参  数：reg_addr 被读取寄存器的地址
  * 参  数：data 读取的数据
  * 返回值：0 发送成功，1 发送设备地址（写模式）失败，2 发送寄存器地址失败，3 发送设备地址（读模式）失败
  */
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

/** 函  数：从指定设备读取多个字节
  * 参  数：dev_addr 被读取设备的地址
  * 参  数：reg_addr 被读取寄存器的地址
  * 参  数：len 要读取的数据长度
  * 参  数：data 读取的数据
  * 返回值：0 发送成功，1 发送设备地址（写模式）失败，2 发送寄存器地址失败，3 发送设备地址（读模式）失败
  */
uint8_t MyI2C_Read_Bytes(uint8_t dev_addr, uint8_t reg_addr, uint16_t len, uint8_t *data)
{
    uint16_t i;
    
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
    for(i = 0; i < len; i++)
	{
        if(i == len - 1)
		{
            data[i] = MyI2C_Read_Byte(0);  // 最后一个字节，发送NACK
        }
		else
		{
            data[i] = MyI2C_Read_Byte(1);  // 发送ACK
        }
    }
    
    MyI2C_Stop();
    return 0;  // 成功
}

/** 函  数：向指定设备写入多个字节
  * 参  数：dev_addr 被写入设备的地址
  * 参  数：reg_addr 被写入寄存器的地址
  * 参  数：len 要写入的数据长度
  * 参  数：data 要写入的数据
  * 返回值：0 发送成功，1 发送设备地址（写模式）失败，2 发送寄存器地址失败，3 发送数据失败
  */
uint8_t MyI2C_Write_Bytes(uint8_t dev_addr, uint8_t reg_addr, uint16_t len, uint8_t *data)
{
    uint16_t i;
    
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
    for(i = 0; i < len; i++)
	{
        MyI2C_Send_Byte(data[i]);
        if(MyI2C_Wait_Ack())
		{
            MyI2C_Stop();
            return 3;  // 失败
        }
    }
    
    MyI2C_Stop();
    return 0;  // 成功
}

/** 函  数：I2C扫描设备
  * 参  数：无
  * 返回值：无
  */
void MyI2C_Scan_Devices(void)
{
    uint8_t i, ret;
    uint8_t found = 0;
    
    printf("Scanning I2C bus...\r\n");
    
    for(i = 1; i < 128; i++)
	{
        MyI2C_Start();
        MyI2C_Send_Byte(i << 1);  // 地址左移一位
        ret = MyI2C_Wait_Ack();
        MyI2C_Stop();
        
        if(ret == 0)
		{
            printf("Found device at address: 0x%02X\r\n", i);
            found++;
        }
    }
    
    if(found == 0)
	{
        printf("No I2C devices found.\r\n");
    }
	else
	{
        printf("Found %d I2C device(s).\r\n", found);
    }
}
