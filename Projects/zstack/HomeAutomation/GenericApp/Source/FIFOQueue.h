//////////////////////////////////////////////////////////
// 文件：FIFOQUEUE.h
//////////////////////////////////////////////////////////
#ifndef _FIFOQUEUE_H
#define _FIFOQUEUE_H
#ifdef __cplusplus
extern "C"
{
#endif

#include "ZComDef.h"
#include "OSAL_Clock.h"
#include "OSAL.h"

//采样数据包
typedef struct
{
	UTCTime utcSecs;
	int16  temp;
}sendData_t;


#define ElemType       sendData_t

#ifdef RTR_NWK
#define QueueSize      1 //协调器和路由器队列大小为1
#else
#define QueueSize      60 //fifo队列的大小
#endif
#define QueueFull      0  //fifo满置0
#define QueueEmpty     1  //FIFO空置1
#define QueueOperateOk 2  //队列操作完成 赋值为2

typedef struct 
{
    uint16 front;     //队列头
    uint16 rear;        //队列尾
    uint16 count;       //队列计数
    ElemType dat[QueueSize];
}FifoQueue;
//Queue Initalize
 void QueueInit( FifoQueue *Queue);
// Queue In
 uint8 QueueIn( FifoQueue *Queue,ElemType *sdat);
// Queue Out
 uint8 QueueOut( FifoQueue *Queue,ElemType *sdat);
uint8 QueueRemove(FifoQueue *Queue,uint8 len);
#endif
