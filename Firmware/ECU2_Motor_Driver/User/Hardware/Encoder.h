#ifndef __ENCODER_H
#define __ENCODER_H

// 编码器故障代码
#define ENCODER_FAULT_NONE       0x00
#define ENCODER_FAULT_NO_PULSE   0x01
#define ENCODER_FAULT_PHASE_ERROR 0x02
#define ENCODER_FAULT_HARDWARE   0x04

// 编码器数据结构
typedef struct
{
    float speed_rpm;        // 转速(RPM)
    int32_t position;       // 位置（脉冲数）
    uint8_t direction;      // 方向：0=停止，1=正转，2=反转
    int32_t pulse_count;    // 脉冲计数
    uint8_t valid;          // 数据是否有效
} Encoder_Data;

// 函数声明
void Encoder_Init(void);
int32_t Encoder_GetCount(void);


// 外部变量声明
extern Encoder_Data encoder_data;

#endif
