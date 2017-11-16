#ifndef SH20_H
#define SH20_H

#include "i2c.h"


#define  TEMP_MEASURE_N_MASTER 		0xF3
#define  HUMI_MEASURE_N_MASTER 		0xF5
#define  TEMP_MEASURE_MASTER 		0xE3
#define  HUMI_MEASURE_MASTER 		0xE5

#define	 RESOLUTION_RH12	    	0x00
#define	 RESOLUTION_T14    			0x00
#define  RESOLUTION_RH8				0x01
#define  RESOLUTION_T12				0x01
#define  RESOLUTION_RH10			0x80
#define  RESOLUTION_T13				0x80
#define  RESOLUTION_RH11			0x81
#define  RESOLUTION_T11				0x81



bool SHT2X_StartMeasureNHM(char whatdo);
bool SHT2X_MeasureReady(char whatdo);
int16 SHT2X_ReadMeasure(char whatdo);

void SoftReset(void) ;

void Set_Resolution(uint8);

#endif