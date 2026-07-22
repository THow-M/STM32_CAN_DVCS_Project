#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

extern volatile char Serial_RxPacket[];
extern volatile uint8_t Serial_RxFlag;

void Serial_Init(uint32_t baudrate);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

#endif
