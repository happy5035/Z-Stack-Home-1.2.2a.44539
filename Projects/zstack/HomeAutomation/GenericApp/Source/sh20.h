#ifndef SH20_H
#define SH20_H

#include "i2c.h"


#define  TEMP_MEASURE_N_MASTER 	0xF3
#define  HUMI_MEASURE_N_MASTER 	0xF5
#define  TEMP_MEASURE_MASTER 	0xE3
#define  HUMI_MEASURE_MASTER 	0xE5

float SHT2X_MeasureNHM(char whatdo);

void SoftReset(void) ;

void Set_Resolution(void);

#endif