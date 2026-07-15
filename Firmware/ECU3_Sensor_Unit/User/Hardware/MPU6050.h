#ifndef __MPU6050_H
#define __MPU6050_H

// MPU6050数据结构
typedef struct
{
    // 原始数据
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
    
    // 校准偏移
    int16_t accel_offset_x;
    int16_t accel_offset_y;
    int16_t accel_offset_z;
    int16_t gyro_offset_x;
    int16_t gyro_offset_y;
    int16_t gyro_offset_z;
    
    // 处理后的数据
    float accel_x_g;    // 加速度 (g)
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;   // 角速度 (度/秒)
    float gyro_y_dps;
    float gyro_z_dps;
    float temperature_c; // 温度 (°C)
    
    // 姿态角
    float roll;         // 横滚角 (度)
    float pitch;        // 俯仰角 (度)
    float yaw;          // 航向角 (度)
} MPU6050_Data;

//函数声明
uint8_t MPU6050_Init(void);
void MPU6050_Read_RawData(void);
void MPU6050_Calibrate(void);
void MPU6050_Apply_Calibration(void);
void MPU6050_Calculate_Attitude(float dt);
MPU6050_Data MPU6050_GetData(void);
void MPU6050_Temperature_Compensation(void);
uint8_t MPU6050_Self_Test(void);

// 外部变量声明
extern MPU6050_Data mpu6050_data;

#endif
