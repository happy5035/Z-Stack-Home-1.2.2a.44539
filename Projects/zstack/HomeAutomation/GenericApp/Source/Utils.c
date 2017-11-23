#include "Utils.h"

#include "OSAL_Clock.h"
#include "OSAL.h"
#include "hal_uart.h"
#include "FIFOQueue.h"
#include "OSAL_Memory.h"

void printString(uint8 * data){
#if(UART_DEBUG_FLAG == TRUE )
  {
  HalUARTWrite(HAL_UART_PORT_1,(uint8*)data,osal_strlen(data)+1);
  }
#endif
}
void printDataLen(uint8 * data,uint16 len){
#if(UART_DEBUG_FLAG == TRUE)
  {
  HalUARTWrite(HAL_UART_PORT_1,(uint8*)data,len);
  }
#endif
}
void printStringLn(uint8 * data){
#if(UART_DEBUG_FLAG == TRUE)
  {
  HalUARTWrite(HAL_UART_PORT_1,(uint8*)data,osal_strlen(data)+1);
  char ln[] = {'\n'};
  HalUARTWrite(HAL_UART_PORT_1,(uint8*)ln,1);
  }
#endif
}
void printData(uint8 * data,uint16 len){
   if(HalUARTWrite(HAL_UART_PORT_1,data,len)==0){
	printStringLn("send data full.");
   }
}
void hextoword(uint8 t,uint8* dest)
{
  uint8 *result = osal_mem_alloc(2);
  uint8 byte_H;
  uint8 byte_L;
  byte_H = (t & 0xf0) >> 4;
  byte_L = t & 0x0f;
  if(byte_H<10)
  {
    result[0]=byte_H+48;
  }
  else
  {
    result[0]=byte_H+55;
  }
  if(byte_L<10)
  {
    result[1]=byte_L+48;
  }
  else
  {
    result[1]=byte_L+55;
  }
  memcpy(dest,result,sizeof(uint8) *2);
  osal_mem_free((uint8*) result);
}
void dec2word(uint8 t,uint8* dest){
  uint8 *result = osal_mem_alloc(2);
  if(t < 10){
    result[0] = 48;
    result[1] = t + 48;
  }
  if(t>=10 && t<100){
    result[0] = t/10 + 48;
    result[1] = t%10 + 48;
  }
  memcpy(dest,result,sizeof(uint8) *2);
  osal_mem_free((uint8*) result);
}
//16进制时间转化为字符串时间格式
uint8* hexTime2String(uint8* time){
  uint8* stringTime;
  uint8 len = 19;
  stringTime = osal_mem_alloc(len);
  if(stringTime){
    uint16 year = *(time+6) << 8 | *(time+5);
    uint8 day = *(time+4);
    uint8 month = *(time+3);
    uint8 secs = *(time+2);
    uint8 min = *(time+1);
    uint8 hour = *time;
    uint8 year_h = year / 100 ;
    uint8 year_l = year % 100;
    dec2word(year_h,stringTime);
    dec2word(year_l,stringTime+2);
    *(stringTime+4) = ':';
    dec2word(month,stringTime+5);
    *(stringTime+7) = ':';
    dec2word(day,stringTime+8);
    *(stringTime+10) = ' ';
    dec2word(hour,stringTime+11);
    *(stringTime+13) = ':';
    dec2word(min,stringTime+14);
    *(stringTime+16) = ':';
    dec2word(secs,stringTime+17);
  }
  return stringTime;
}

/**
*	通过UTCtime，通过串口发送字符串时间。
*/
void printTime(UTCTime ustcSecs){
	uint8 len;
	uint8 * buf;
	len 				= sizeof(UTCTime) +sizeof(UTCTimeStruct);
	buf 				= osal_mem_alloc(len);
	if(buf){
		UTCTime utcSecs;
		uint8 * pBuf;
		utcSecs 			= osal_getClock();
		UTCTimeStruct utcTime;

		// Get current 32-bit UTC time and parse it
		utcSecs 			= osal_getClock();
		osal_ConvertUTCTime(&utcTime, utcSecs);

		// Start with 32-bit UTC time
		pBuf				= osal_buffer_uint32(buf, utcSecs);

		// Concatenate parsed UTC time fields
		*pBuf++ 			= utcTime.hour;
		*pBuf++ 			= utcTime.minutes;
		*pBuf++ 			= utcTime.seconds;
		*pBuf++ 			= utcTime.month + 1; // Convert to human numbers
		*pBuf++ 			= utcTime.day + 1;
		*pBuf++ 			= LO_UINT16(utcTime.year);
		*pBuf++ 			= HI_UINT16(utcTime.year);
		uint8 * date		= hexTime2String(buf + 4);
		HalUARTWrite(HAL_UART_PORT_1, date, 19);
		osal_mem_free(date);
	}
	osal_mem_free(buf);
}
void printLn(void){
	char ln[] = {'\n'};
	HalUARTWrite(HAL_UART_PORT_1, (uint8 *)ln, 1);
}
void printChar(char c){
	HalUARTWrite(HAL_UART_PORT_1, (uint8 *)(&c), 1);
}

void printTempPacket(sendData_t * pkt){
#if(UART_DEBUG_FLAG == TRUE)
//	UTCTime utcSecs = pkt->utcSecs;
//	UTCTimeStruct utcTime;
//	osal_ConvertUTCTime(&utcTime,utcSecs);
//	printTime(utcSecs);
//	uint8 t[2];
//	dec2word(utcTime.year/100, t);
//	printDataLen(t, 2);
//	dec2word(utcTime.year%100, t);
//	printDataLen(t, 2);
//	printChar('-');
//	dec2word(utcTime.month+1, t);
//	printDataLen(t, 2);
//	printChar('-');
//	dec2word(utcTime.day+1, t);
//	printDataLen(t, 2);
//	printChar( ' ');
//	dec2word(utcTime.hour, t);
//	printDataLen(t, 2);
//	printChar('-');
//	dec2word(utcTime.minutes, t);
//	printDataLen(t, 2);
//	printChar('-');
//	dec2word(utcTime.seconds, t);
//	printDataLen(t, 2);
//	printChar(',');
	int16 _temp = pkt->data;
	if(_temp <0){
		printChar('-');
		_temp = 0 - _temp;
	}else{
		printChar('+');
	}
	if(_temp >= 10000 || _temp < -10000){
		//温度数据有问题
		_temp %= 10000;
	}
	uint8 tempBuf[4];
	tempBuf[0] = _temp / 1000 + 48;
	tempBuf[1] = _temp % 1000 / 100 + 48;
	tempBuf[2] = _temp % 100 / 10 + 48;
	tempBuf[3] = _temp % 10  + 48;
	printDataLen(tempBuf, 4);
    printLn();
//    //需要延时一会等待串口发送数据
//    delayMs(100);
    
#endif
}

void delayUs(uint16 us){
	us *= 8;
    int i;
	for (i=0; i<us; i++) asm("NOP");  
}
void delayMs(uint16 ms){
	delayUs(ms * 1000);
}

//答应16位数据hex字符串
void printUint16(uint16 data){
	printChar('0');
	printChar('X');
	uint8* buf;
	buf = osal_mem_alloc(4);
	if(buf){
		hextoword((uint8)((data&0xFF00)>>8), buf);
		hextoword((uint8)(data&0x00FF), buf+2);
		printDataLen(buf, 4);
		osal_mem_free(buf);
	}
	
}
void printUint16Ln(uint16 data){
	printUint16(data);
	printLn();
}

//答应heap状态
void printHeapStatus(){
  
#if (OSALMEM_METRICS)
		printStringLn("heap status");
		printString("block max:");
		printUint16Ln(osal_heap_block_max());
		printString("block cnt:");
		printUint16Ln(osal_heap_block_cnt());
		printString("block free:");
		printUint16Ln(osal_heap_block_free());
		printString("mem used:");
		printUint16Ln(osal_heap_mem_used());
#endif
}