#include "tmp275.h"

#include "i2c.h"
#define TEMP_REG_POINTER	0x00
#define CONIFG_REG_POINTER	0x01
#define TLOW_REG_POINTER	0x02
#define THIGH_REG_POINTER	0x03
#define ONE_SHOT_MODEL		0x80
#define SHUTDOWN_MODEL		0x01
#define RESOLUTION_9 		0x00
#define RESOLUTION_10 		0x02
#define RESOLUTION_11 		0x04
#define RESOLUTION_12 		0x06



uint8 TMP275_startMeasure(){
	delay_us(50);
	IIC_Config();
	IIC_Start();
	IIC_Send_Byte(0x90); //addr 0,write
	
	if(IIC_Wait_Ack() == 0){
		IIC_Send_Byte(CONIFG_REG_POINTER);
		if(IIC_Wait_Ack() == 0){
			IIC_Send_Byte(SHUTDOWN_MODEL | ONE_SHOT_MODEL | RESOLUTION_10);
			if(IIC_Wait_Ack() == 0) {
				IIC_Stop();
				return TRUE;
			}
			
		}
	}
	return FALSE;
}
uint16 TMP275_ReadTemp(){
	uint8 byte1;
	uint8 byte2;
	uint16 temp;
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
				byte1 = IIC_Read_Byte();
				delay_us(50);
				byte2 = IIC_Read_Byte();
				delay_us(50);
				temp = (byte1<<4) | (byte2>>4);
//				result = temp * 0.0625;
			}
			
		}
	}
	return temp;
	
}
