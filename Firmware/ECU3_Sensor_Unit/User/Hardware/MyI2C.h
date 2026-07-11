#ifndef __MYI2C_H
#define __MYI2C_H

// I2C引脚控制宏
#define I2C_SCL_HIGH()  GPIO_SetBits(I2C_PORT, I2C_SCL_PIN)
#define I2C_SCL_LOW()   GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN)
#define I2C_SDA_HIGH()  GPIO_SetBits(I2C_PORT, I2C_SDA_PIN)
#define I2C_SDA_LOW()   GPIO_ResetBits(I2C_PORT, I2C_SDA_PIN)
#define READ_SDA()      GPIO_ReadInputDataBit(I2C_PORT, I2C_SDA_PIN)

// SDA方向控制
#define SDA_IN()        {GPIO_InitTypeDef GPIO_InitStructure; \
                         GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN; \
                         GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; \
                         GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
                         GPIO_Init(I2C_PORT, &GPIO_InitStructure);}
                         
#define SDA_OUT()       {GPIO_InitTypeDef GPIO_InitStructure; \
                         GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN; \
                         GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; \
                         GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; \
                         GPIO_Init(I2C_PORT, &GPIO_InitStructure);}

// 函数声明
void MyI2C_Init(void);
void MyI2C_Start(void);
void MyI2C_Stop(void);
uint8_t MyI2C_Wait_Ack(void);
void MyI2C_Ack(void);
void MyI2C_NAck(void);
void MyI2C_Send_Byte(uint8_t data);
uint8_t MyI2C_Read_Byte(uint8_t ack);

#endif
