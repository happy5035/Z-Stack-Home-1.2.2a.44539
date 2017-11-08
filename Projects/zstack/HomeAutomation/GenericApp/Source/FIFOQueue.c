//////////////////////////////////////////////////////////
// 文件：FIFOQUEUE.C
//////////////////////////////////////////////////////////
#include "FIFOQueue.h"
#include "Utils.h"
//Queue Init
void QueueInit( FifoQueue *Queue)
{
    Queue->front = Queue->rear;//初始化时队列头队列首相连
    Queue->count = 0;   //队列计数为0
}

// Queue In
uint8 QueueIn( FifoQueue *Queue,ElemType *sdat) //数据进入队列
{
    if((Queue->front == Queue->rear) && (Queue->count == QueueSize))
    {                    // full //判断如果队列满了
    	printStringLn("queue full");
        return QueueFull;    //返回队列满的标志
    }else
    {                    // in
//        Queue->dat[Queue->rear] = sdat;
		osal_memcpy(&Queue->dat[Queue->rear],sdat,sizeof(ElemType));
        Queue->rear = (Queue->rear + 1) % QueueSize;
        Queue->count = Queue->count + 1;
        return QueueOperateOk;
    }
}

// Queue Out
uint8 QueueOut( FifoQueue *Queue,ElemType *sdat)
{
    if((Queue->front == Queue->rear) && (Queue->count == 0))
    {                    // empty
        return QueueEmpty;
    }else
    {                    // out
        *sdat = Queue->dat[Queue->front];
//		osal_memcmp(sdat,&Queue->dat[Queue->front],sizeof(ElemType));
        Queue->front = (Queue->front + 1) % QueueSize;
        Queue->count = Queue->count - 1;
        return QueueOperateOk;
    }
}
//移除n个
uint8 QueueRemove(FifoQueue *Queue,uint8 len){
		if(Queue->count <= len){
			Queue->front = Queue->rear;
			Queue->count = 0;
			return QueueEmpty;
		}else{
			Queue->front=(Queue->front +len) % QueueSize;
			Queue->count -=len;
			return QueueOperateOk;
		}
}