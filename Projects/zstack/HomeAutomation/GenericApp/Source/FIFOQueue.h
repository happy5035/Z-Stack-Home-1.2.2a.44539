//////////////////////////////////////////////////////////
// �ļ���FIFOQUEUE.h
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

//�������ݰ�
typedef struct
{
	UTCTime utcSecs;
	int16  temp;
}sendData_t;


#define ElemType       sendData_t

#ifdef RTR_NWK
#define QueueSize      1 //Э������·�������д�СΪ1
#else
#define QueueSize      60 //fifo���еĴ�С
#endif
#define QueueFull      0  //fifo����0
#define QueueEmpty     1  //FIFO����1
#define QueueOperateOk 2  //���в������ ��ֵΪ2

typedef struct 
{
    uint16 front;     //����ͷ
    uint16 rear;        //����β
    uint16 count;       //���м���
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
