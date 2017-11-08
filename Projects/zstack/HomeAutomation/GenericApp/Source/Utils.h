#ifndef UTILS_H
#define UTILS_H
#ifdef __cplusplus
extern "C"
{
#endif

#include <string.h>
#include "ZComDef.h"
#include "OSAL_Clock.h"
#include "OSAL.h"
#include "hal_uart.h"
#include "FIFOQueue.h"
#include "OSAL_Memory.h"
  
  
#ifndef UART_DEBUG_FLAG
#define UART_DEBUG_FLAG TRUE
#endif

void printString(uint8 *);
void printStringLn(uint8 * data);
void printDataLen(uint8 * data,uint16 len);
void hextoword(uint8,uint8*);
uint8* hexTime2String(uint8* time);
void dec2word(uint8 t,uint8* dest);
void printData(uint8 * data,uint16 len);
void printTime(UTCTime );
void printLn(void);
void printTempPacket(sendData_t * pkt);
void printChar(char c);
void delayUs(uint16 us);
void delayMs(uint16 ms);
void printUint16(uint16 data);
void printUint16Ln(uint16 data);
void printHeapStatus(void);



#endif
