#include "tmp275.h"

#include "ioCC2530.h"

#include "i2c.h"
#include "user_printf.h"
#define TEMP_REG_POINTER	0x00
#define CONIFG_REG_POINTER	0x01
#define TLOW_REG_POINTER	0x02
#define THIGH_REG_POINTER	0x03
#define ONE_SHOT_MODEL		0x80
#define SHUTDOWN_MODEL		0x01
#define RESOLUTION_9 		0x00
#define RESOLUTION_10 		0x20
#define RESOLUTION_11 		0x40
#define RESOLUTION_12 		0x60
#define POLARITY			0x04	//0为残生负跳变，1为产生正跳变
#define THERMOSTAT			0x02	// 1为终端模式，0为比较模式

void WriteAlert1(void);
#define ALERT_PORT 	P0_4
#define ALERT_BIT	0x10
#define ALERT_SEL	P0SEL
#define ALERT_DIR	P0DIR

uint8 TMP275_startMeasure(){
	uint8 res;
	delay_us(50);
	IIC_Config();
	IIC_Start();
	IIC_Send_Byte(0x90); //addr 0,write
	
	if(IIC_Wait_Ack() == 0){
		IIC_Send_Byte(CONIFG_REG_POINTER);
		if(IIC_Wait_Ack() == 0){
			IIC_Send_Byte(SHUTDOWN_MODEL | ONE_SHOT_MODEL | RESOLUTION_12 | THERMOSTAT);
			if(IIC_Wait_Ack() == 0) {
				IIC_Stop();
				res =  TRUE;
			}
			
		}
	}
	res =  FALSE;

	//将sda，scl，alert拉高。防止漏电。
//	WriteSDA1(); nk
//	WriteSCL1();
//	WriteAlert1();
	return res;
}
int16 TMP275_ReadTemp(){
	uint8 byte1;
	uint8 byte2;
	int16 temp;
	temp = 0xFFFF;
	delay_us(50);
	IIC_Start();
	IIC_Send_Byte(0x90); //addr 0,write
	if(IIC_Wait_Ack() == 0){
		IIC_Send_Byte(TEMP_REG_POINTER);
		if(IIC_Wait_Ack() == 0){
			IIC_Start();
			IIC_Send_Byte(0x91); // addr 0, read
			if(IIC_Wait_Ack() == 0){
				temp = 0;
				byte1 = IIC_Read_Byte();
				byte2 = IIC_Read_Byte();
				IIC_Stop();
				temp = (byte1<<4) | (byte2>>4);
//				result = temp * 0.0625;
				printf("%02x%02x   ",byte1,byte2);
			}
			
		}
	}
	float res = temp * 6.25;
	temp = (int)(res);
	return temp;
	
}
void WriteAlert1(){
	ALERT_SEL &=~ALERT_BIT;
	ALERT_DIR |= ALERT_BIT;
	ALERT_PORT = 1;
}
