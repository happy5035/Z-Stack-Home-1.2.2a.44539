#ifndef I2C_H
#define I2C_H

#include "ZComDef.h"
void delay_us(uint8);
void WriteSDA1(void);
void WriteSDA0(void);
void WriteSCL1(void);
void WriteSCL0(void);
void ReadSDA(void);
void IIC_Start(void);
void IIC_Stop(void);
void SEND_0(void);   
void SEND_1(void);
char IIC_Wait_Ack(void);
void Write_Acknowledge(void);
void IIC_Send_Byte(uint8);
uint8 IIC_Read_Byte(void);
#endif