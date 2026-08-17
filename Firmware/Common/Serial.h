#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

/* 串口接收包最大长度（含结尾 '\0'，实际有效载荷 = 100-1 = 99 字节） */
#define SERIAL_RX_PACKET_SIZE   100U

extern volatile char Serial_RxPacket[SERIAL_RX_PACKET_SIZE];
extern volatile uint8_t Serial_RxFlag;

void Serial_Init(uint32_t baudrate);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

#endif
