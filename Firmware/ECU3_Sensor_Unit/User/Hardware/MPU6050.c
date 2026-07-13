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

/** 函  数：MPU6050初始化
  * 参  数：无
  * 返回值：0 初始化失败，1 初始化成功
  */
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

/** 函  数：读取原始数据
  * 参  数：无
  * 返回值：无
  */
void MPU6050_Read_RawData(void)
{
    uint8_t buffer[14];
    
    if(MyI2C_Read_Bytes(MPU6050_ADDR, ACCEL_XOUT_H, 14, buffer) == 0)
	{
        // 加速度数据
        mpu6050_data.accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
        mpu6050_data.accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
        mpu6050_data.accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
        
        // 温度数据
        mpu6050_data.temp = (int16_t)((buffer[6] << 8) | buffer[7]);
        
        // 陀螺仪数据
        mpu6050_data.gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
        mpu6050_data.gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
        mpu6050_data.gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);
        
        // 转换为实际单位
        mpu6050_data.accel_x_g = mpu6050_data.accel_x / 4096.0f;  // ±8g: 4096 LSB/g
        mpu6050_data.accel_y_g = mpu6050_data.accel_y / 4096.0f;
        mpu6050_data.accel_z_g = mpu6050_data.accel_z / 4096.0f;
        
        mpu6050_data.gyro_x_dps = mpu6050_data.gyro_x / 16.4f;  // ±2000°/s: 16.4 LSB/°/s
        mpu6050_data.gyro_y_dps = mpu6050_data.gyro_y / 16.4f;
        mpu6050_data.gyro_z_dps = mpu6050_data.gyro_z / 16.4f;
        
        mpu6050_data.temperature_c = mpu6050_data.temp / 340.0f + 36.53f;
    }
	else
	{
        printf("MPU6050: Failed to read data\n");
    }
}
