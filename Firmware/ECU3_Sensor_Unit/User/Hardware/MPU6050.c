#include "stm32f10x.h"                  // Device header
#include "MPU6050.h"
#include "MPU6050_Reg.h"
#include "MyI2C.h"
#include "Serial.h"
#include "Delay.h"
#include <math.h>

// MPU6050地址
#define MPU6050_ADDR    0xD0  // 0x68左移一位

// 全局变量
MPU6050_Data mpu6050_data = {0};
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // 四元数
static float exInt = 0, eyInt = 0, ezInt = 0;             // 误差积分
static Mahony_Params_t mahony_params = {
    .Kp = 0.1f,
    .Ki = 0.01f,
    .half_Dt = 0.0f
};

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
        printf("MPU6050: Failed to read WHO_AM_I register\r\n");
        return 0;
    }
    
    if(check != 0x70)
	{
        printf("MPU6050: Wrong device ID: 0x%02X (expected 0x68)\r\n", check);
        return 0;
    }
    
    printf("MPU6050 detected. ID: 0x%02X\r\n", check);
    
    // 唤醒MPU6050
    if(MyI2C_Write_Byte(MPU6050_ADDR, PWR_MGMT_1, 0x00))
	{
        printf("MPU6050: Failed to wake up\r\n");
        return 0;
    }
    
    Delay_ms(100);
    
    // 设置采样率
    if(MyI2C_Write_Byte(MPU6050_ADDR, SMPLRT_DIV, 0x07))
	{  // 1kHz/(7+1)=125Hz
        printf("MPU6050: Failed to set sample rate\r\n");
        return 0;
    }
    
    // 设置低通滤波器
    if(MyI2C_Write_Byte(MPU6050_ADDR, CONFIG, 0x06))
	{  // 5Hz
        printf("MPU6050: Failed to set low-pass filter\r\n");
        return 0;
    }
    
    // 设置陀螺仪量程
    if(MyI2C_Write_Byte(MPU6050_ADDR, GYRO_CONFIG, GYRO_CONFIG_2000DPS))
	{  // ±2000°/s
        printf("MPU6050: Failed to set gyro range\r\n");
        return 0;
    }
    
    // 设置加速度计量程
    if(MyI2C_Write_Byte(MPU6050_ADDR, ACCEL_CONFIG, ACCEL_CONFIG_8G))
	{  // ±8g
        printf("MPU6050: Failed to set accelerometer range\r\n");
        return 0;
    }
	
	/* 新增：等待传感器数据稳定（数据手册要求 >= 30ms） */
    Delay_ms(100);
	
	/* 新增：先丢弃前 10 次采样（瞬态数据） */
    for(uint8_t i = 0; i < 10; i++)
    {
        MPU6050_Read_RawData();
        Delay_ms(10);
    }
    
    // 校准传感器
    MPU6050_Calibrate();
    
    printf("MPU6050 initialized successfully\r\n");
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
        printf("MPU6050: Failed to read data\r\n");
    }
}

/** 函  数：传感器校准
  * 参  数：无
  * 返回值：无
  */
void MPU6050_Calibrate(void)
{
    printf("MPU6050: Starting calibration...\r\n");
    printf("Please keep sensor stable and level...\r\n");
    
    int32_t accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0;
    int32_t gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
    uint16_t sample_count = 100;
    
    for(uint16_t i = 0; i < sample_count; i++)
	{
        MPU6050_Read_RawData();
        
        accel_sum_x += mpu6050_data.accel_x;
        accel_sum_y += mpu6050_data.accel_y;
        accel_sum_z += mpu6050_data.accel_z;
        
        gyro_sum_x += mpu6050_data.gyro_x;
        gyro_sum_y += mpu6050_data.gyro_y;
        gyro_sum_z += mpu6050_data.gyro_z;
        
        Delay_ms(10);
        
        if(i % 50 == 0)
		{
            printf("Calibrating... %d%%\r\n", (i * 100) / sample_count);
        }
    }
    
    // 计算平均值
    mpu6050_data.accel_offset_x = accel_sum_x / sample_count;
    mpu6050_data.accel_offset_y = accel_sum_y / sample_count;
    mpu6050_data.accel_offset_z = (accel_sum_z / sample_count) - 4096;  // 减去1g
    
    mpu6050_data.gyro_offset_x = gyro_sum_x / sample_count;
    mpu6050_data.gyro_offset_y = gyro_sum_y / sample_count;
    mpu6050_data.gyro_offset_z = gyro_sum_z / sample_count;
    
    printf("Calibration completed:\r\n");
    printf("Accel Offset: X=%d, Y=%d, Z=%d\r\n", 
           mpu6050_data.accel_offset_x, 
           mpu6050_data.accel_offset_y, 
           mpu6050_data.accel_offset_z);
    printf("Gyro Offset: X=%d, Y=%d, Z=%d\r\n", 
           mpu6050_data.gyro_offset_x, 
           mpu6050_data.gyro_offset_y, 
           mpu6050_data.gyro_offset_z);
}

/** 函  数：应用校准数据
  * 参  数：无
  * 返回值：无
  */
void MPU6050_Apply_Calibration(void)
{
    mpu6050_data.accel_x -= mpu6050_data.accel_offset_x;
    mpu6050_data.accel_y -= mpu6050_data.accel_offset_y;
    mpu6050_data.accel_z -= mpu6050_data.accel_offset_z;
    
    mpu6050_data.gyro_x -= mpu6050_data.gyro_offset_x;
    mpu6050_data.gyro_y -= mpu6050_data.gyro_offset_y;
    mpu6050_data.gyro_z -= mpu6050_data.gyro_offset_z;
    
    // 重新计算g单位
    mpu6050_data.accel_x_g = mpu6050_data.accel_x / 4096.0f;
    mpu6050_data.accel_y_g = mpu6050_data.accel_y / 4096.0f;
    mpu6050_data.accel_z_g = mpu6050_data.accel_z / 4096.0f;
    
    // 重新计算dps单位
    mpu6050_data.gyro_x_dps = mpu6050_data.gyro_x / 16.4f;
    mpu6050_data.gyro_y_dps = mpu6050_data.gyro_y / 16.4f;
    mpu6050_data.gyro_z_dps = mpu6050_data.gyro_z / 16.4f;
}

/** 函  数：互补滤波姿态解算
  * 参  数：dt  采样周期(秒)，需与实际调用频率一致
  *           例如10ms定时器调用则传入0.01f，5ms则传入0.005f
  * 返回值：无
  * 注  释：融合MPU6050的加速度计和陀螺仪数据，通过四元数计算
  *         三轴姿态角(roll/pitch/yaw)，结果存入mpu6050_data结构体
  *         算法：Mahony互补滤波 + 一阶欧拉四元数更新
  *         Kp=0.1(比例增益), Ki=0.01(积分增益)
  *         首次调用时仅记录时间戳并返回，不进行计算
  */
void MPU6050_Calculate_Attitude(float dt)
{
    static uint8_t first_run = 1;
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float halfT = dt / 2.0f;
    
    if(first_run)
	{
        first_run = 0;
        return;
    }
    
    // 读取原始数据
    //MPU6050_Read_RawData();
    
    // 应用校准
    MPU6050_Apply_Calibration();
    
    // 归一化加速度计数据
    norm = sqrt(mpu6050_data.accel_x_g * mpu6050_data.accel_x_g + 
                mpu6050_data.accel_y_g * mpu6050_data.accel_y_g + 
                mpu6050_data.accel_z_g * mpu6050_data.accel_z_g);
    
    if(norm > 0.001f)
	{
        mpu6050_data.accel_x_g /= norm;
        mpu6050_data.accel_y_g /= norm;
        mpu6050_data.accel_z_g /= norm;
    }
    
    // 估计重力的方向
    vx = 2 * (q1 * q3 - q0 * q2);
    vy = 2 * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    
    // 计算误差
    ex = (mpu6050_data.accel_y_g * vz - mpu6050_data.accel_z_g * vy);
    ey = (mpu6050_data.accel_z_g * vx - mpu6050_data.accel_x_g * vz);
    ez = (mpu6050_data.accel_x_g * vy - mpu6050_data.accel_y_g * vx);
    
    // 积分误差
    exInt += ex * 0.5f * dt;
    eyInt += ey * 0.5f * dt;
    ezInt += ez * 0.5f * dt;
    
    // 修正陀螺仪数据
    mpu6050_data.gyro_x_dps += mahony_params.Kp * ex + mahony_params.Ki * exInt;
    mpu6050_data.gyro_y_dps += mahony_params.Kp * ey + mahony_params.Ki * eyInt;
    mpu6050_data.gyro_z_dps += mahony_params.Kp * ez + mahony_params.Ki * ezInt;
    
    // 转换为弧度/秒
    float gx = mpu6050_data.gyro_x_dps * 0.0174533f;  // 度/秒 -> 弧度/秒
    float gy = mpu6050_data.gyro_y_dps * 0.0174533f;
    float gz = mpu6050_data.gyro_z_dps * 0.0174533f;
    
    // 四元数微分方程
    float q0t = (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    float q1t = ( q0 * gx + q2 * gz - q3 * gy) * halfT;
    float q2t = ( q0 * gy - q1 * gz + q3 * gx) * halfT;
    float q3t = ( q0 * gz + q1 * gy - q2 * gx) * halfT;
    
    // 更新四元数
    q0 += q0t;
    q1 += q1t;
    q2 += q2t;
    q3 += q3t;
    
    // 归一化四元数
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if(norm > 0.001f)
	{
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }
    
    // 计算欧拉角
    mpu6050_data.roll = atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2)) * 57.2958f;  // 弧度 -> 度
    mpu6050_data.pitch = asin(2 * (q0 * q2 - q3 * q1)) * 57.2958f;
    mpu6050_data.yaw = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * 57.2958f;
    
    // 保持yaw在0-360度
    if(mpu6050_data.yaw < 0) mpu6050_data.yaw += 360.0f;
    if(mpu6050_data.yaw > 360.0f) mpu6050_data.yaw -= 360.0f;
}

/** 函  数：获取MPU6050数据
  * 参  数：无
  * 返回值：mpu6050_data MPU6050数据结构体
  */
MPU6050_Data MPU6050_GetData(void)
{
    return mpu6050_data;
}

/** 函  数：温度补偿
  * 参  数：无
  * 返回值：mpu6050_data MPU6050数据结构体
  */
void MPU6050_Temperature_Compensation(void)
{
    // 简单的温度补偿
    static float avg_temp = 25.0f;
    static uint8_t first_run = 1;
    
    if(first_run)
	{
        avg_temp = mpu6050_data.temperature_c;
        first_run = 0;
    }
	else
	{
        // 低通滤波
        avg_temp = 0.9f * avg_temp + 0.1f * mpu6050_data.temperature_c;
    }
    
    // 温度变化对陀螺仪的影响
    float temp_factor = 1.0f + (avg_temp - 25.0f) * 0.001f;  // 0.1%/°C
    
    mpu6050_data.gyro_x_dps = (mpu6050_data.gyro_x / 16.4f) * temp_factor;
    mpu6050_data.gyro_y_dps = (mpu6050_data.gyro_y / 16.4f) * temp_factor;
    mpu6050_data.gyro_z_dps = (mpu6050_data.gyro_z / 16.4f) * temp_factor;
}

/** 函  数：自检
  * 参  数：无
  * 返回值：result：0，自检失败或未通过，1，自检通过
  */
uint8_t MPU6050_Self_Test(void)
{
    uint8_t data[4];
    uint8_t result = 0;
    
    printf("MPU6050 Self Test...\r\n");
    
    // 读取自检寄存器
    if(MyI2C_Read_Bytes(MPU6050_ADDR, 0x0D, 4, data) == 0)
	{
		/* 陀螺仪自检值：5位，在 data[0~2] 的高5位 [7:3] */
		uint8_t gyro_x_test = data[0] >> 3;
		uint8_t gyro_y_test = data[1] >> 3;
		uint8_t gyro_z_test = data[2] >> 3;
	
		/* 加速度计自检值：4位，低3位在 data[0~2] 的 [2:0]，高1~2位在 data[3] */
		uint8_t accel_x_test = (data[0] & 0x07) | ((data[3] & 0x20) >> 2);
		uint8_t accel_y_test = (data[1] & 0x07) | ((data[3] & 0x10) >> 1);
		uint8_t accel_z_test = (data[2] & 0x07) | (data[3] & 0x0C);

		printf("Self Test Results:\r\n");
		printf("Accel X: %d (0-15)\r\n", accel_x_test);
		printf("Accel Y: %d (0-15)\r\n", accel_y_test);
		printf("Accel Z: %d (0-15)\r\n", accel_z_test);
		printf("Gyro X:  %d (0-24)\r\n", gyro_x_test);
		printf("Gyro Y:  %d (0-24)\r\n", gyro_y_test);
		printf("Gyro Z:  %d (0-24)\r\n", gyro_z_test);

		/* 陀螺仪 5位最大31，出厂值通常 0~24；加速度计 4位最大15 */
		if(accel_x_test <= 15 && accel_y_test <= 15 && accel_z_test <= 15 &&
		gyro_x_test  <= 24 && gyro_y_test  <= 24 && gyro_z_test  <= 24)
		{
			result = 1;
			printf("Self Test PASSED\r\n");
		}
		else
		{
			printf("Self Test FAILED\r\n");
		}
	}
	else
	{
        printf("Failed to read self-test registers\r\n");
    }
    
    return result;
}
