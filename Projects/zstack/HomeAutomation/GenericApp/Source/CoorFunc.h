#ifndef COORFUNC_H
#define COORFUNC_H


#include "Constants.h"
#include "OSAL.h"
#include "AF.h"


//coor funciton
void CoorSendSyncClock(afIncomingMSGPacket_t *pkt);
void CoorProcessTempHumData(afIncomingMSGPacket_t *pkt);
void CoorSendCoorStart(void);
void CoorProcessMtSysMsg(afIncomingMSGPacket_t *pkt);
#endif