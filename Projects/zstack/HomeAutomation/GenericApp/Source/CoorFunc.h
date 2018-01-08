#ifndef COORFUNC_H
#define COORFUNC_H


#include "Constants.h"
#include "OSAL.h"
#include "AF.h"
#include "MT_APP.h"

//coor funciton
void CoorSendSyncClock(afIncomingMSGPacket_t *pkt);
void CoorProcessTempHumData(afIncomingMSGPacket_t *pkt);
void CoorSendCoorStart(void);
void CoorProcessMtSysMsg(mtSysAppMsg_t *pkt);
void CoorProcessEndStatus(afIncomingMSGPacket_t *pkt);
void CoorProcessEndSyncParams(afIncomingMSGPacket_t *pkt);
void CoorSendSyncParams(uint8 paramsVersion,uint16 destAddr);
#endif