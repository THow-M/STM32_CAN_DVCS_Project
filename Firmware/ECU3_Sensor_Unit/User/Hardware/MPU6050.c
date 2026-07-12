#include "stm32f10x.h"                  // Device header
#include "MPU6050.h"
#include "MPU6050_Reg.h"
#include "MyI2C.h"
#include "Serial.h"
#include "Delay.h"

// MPU6050地址
#define MPU6050_ADDR    0xD0  // 0x68左移一位

// 全局变量
MPU6050_Data mpu6050_data = {0};
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // 四元数
static float exInt = 0, eyInt = 0, ezInt = 0;             // 误差积分

// MPU6050初始化
uint8_t MPU6050_Init(void)
{
    uint8_t check;
    
    // 初始化I2C
    MyI2C_Init();
    
    // 扫描I2C设备
    MyI2C_Scan_Devices();
    
    // 检查设备ID
    if(MyI2C_Read_Byte_From_Reg(MPU6050_ADDR, WHO_AM_I, &check))
	{
        printf("MPU6050: Failed to read WHO_AM_I register\n");
        return 0;
    }
    
    if(check != 0x68)
	{
        printf("MPU6050: Wrong device ID: 0x%02X (expected 0x68)\n", check);
        return 0;
    }
    
    printf("MPU6050 detected. ID: 0x%02X\n", check);
    
    // 唤醒MPU6050
    if(MyI2C_Write_Byte(MPU6050_ADDR, PWR_MGMT_1, 0x00))
	{
        printf("MPU6050: Failed to wake up\n");
        return 0;
    }
    
    Delay_ms(100);
    
    // 设置采样率
    if(MyI2C_Write_Byte(MPU6050_ADDR, SMPLRT_DIV, 0x07))
	{  // 1kHz/(7+1)=125Hz
        printf("MPU6050: Failed to set sample rate\n");
        return 0;
    }
    
    // 设置低通滤波器
    if(MyI2C_Write_Byte(MPU6050_ADDR, CONFIG, 0x06))
	{  // 5Hz
        printf("MPU6050: Failed to set low-pass filter\n");
        return 0;
    }
    
    // 设置陀螺仪量程
    if(MyI2C_Write_Byte(MPU6050_ADDR, GYRO_CONFIG, 0x18))
	{  // ±2000°/s
        printf("MPU6050: Failed to set gyro range\n");
        return 0;
    }
    
    // 设置加速度计量程
    if(MyI2C_Write_Byte(MPU6050_ADDR, ACCEL_CONFIG, 0x10))
	{  // ±8g
        printf("MPU6050: Failed to set accelerometer range\n");
        return 0;
    }
    
    // 校准传感器
    //MPU6050_Calibrate();
    
    printf("MPU6050 initialized successfully\n");
    return 1;
}
