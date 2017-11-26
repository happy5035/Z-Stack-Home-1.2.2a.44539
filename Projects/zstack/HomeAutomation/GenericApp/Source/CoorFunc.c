/**
*   协调器处理函数
*/
#include "CoorFunc.h"

#include "hal_uart.h"
#include "hal_led.h"
#include "OSAL_Clock.h"

extern byte GenericApp_TransID;
extern endPointDesc_t GenericApp_epDesc;
extern afAddrType_t GenericApp_DstAddr;

//	Local Function
uint8 MasterSetClock(mtSysAppMsg_t *pkt);

/*   C O O R   P R O C E S S   T E M P   H U M   D A T A   */
/*-------------------------------------------------------------------------
    协调器处理终端发送的温度湿度数据包
-------------------------------------------------------------------------*/
void CoorProcessTempHumData(afIncomingMSGPacket_t *pkt){
	MT_BuildAndSendZToolResponse(MT_RSP_CMD_APP, MT_APP_MSG, pkt->cmd.DataLength, pkt->cmd.Data);
}


/*   S E N D   C O O R   S T A R T   */
/*-------------------------------------------------------------------------
    通过串口发送协调器启动
-------------------------------------------------------------------------*/
void CoorSendCoorStart(void){
	uint8 cmd = COOR_START_CMD;
	MT_BuildAndSendZToolResponse(MT_RSP_CMD_APP, MT_APP_MSG, 1, &cmd);
}

/*   P R O C E S S   M T   S Y S   M S G   */
/*-------------------------------------------------------------------------
    处理串口发送的数据命令
-------------------------------------------------------------------------*/
void CoorProcessMtSysMsg(mtSysAppMsg_t *pkt){
	uint8 retValue ;
	retValue = ZSuccess;
	uint8 cmd;
	cmd = *pkt->appData++;
	pkt->appDataLen--;
	switch (cmd){
		case MASTER_SET_CLOCK_CMD:
			retValue = MasterSetClock(pkt);
			break;
		default:
			break;
	}
	MT_BuildAndSendZToolResponse(MT_RSP_CMD_APP, MT_APP_MSG, 1, &retValue);
	
}
/*   C O R   S E N D   C L O C K   */
/*-------------------------------------------------------------------------
    协调器发送同步时间给终端
-------------------------------------------------------------------------*/
void CoorSendSyncClock(afIncomingMSGPacket_t *pkt){
	uint8* packet;
	uint8 len;
	len = sizeof(UTCTime);
	packet = osal_mem_alloc(len);
	if(packet){
		osal_buffer_uint32(packet, osal_getClock());
		GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
		GenericApp_DstAddr.endPoint = GENERICAPP_ENDPOINT;
		GenericApp_DstAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;
		if (AF_DataRequest(&GenericApp_DstAddr, &GenericApp_epDesc, 
			SYNC_TIME_CLUSTERID, 
			len, 
		(byte *) packet, 
			&GenericApp_TransID, 
			AF_DISCV_ROUTE, AF_DEFAULT_RADIUS) == afStatus_SUCCESS)
		{
			// Successfully requested to be sent.
			HalLedBlink(HAL_LED_4, 1, 50, 500);
//			printf("send sync clock success\n");
		}else{
//			printf("send sync clock failed\n");
	    }
	    osal_mem_free(packet);
	}else{
//		printf("alloc send clock packet failed.\n");
	}
}

/*   M A S T E R   S E T   C L O C K   */
/*-------------------------------------------------------------------------
    处理主机发送的同步时间命令
-------------------------------------------------------------------------*/
uint8 MasterSetClock(mtSysAppMsg_t *pkt){
	UTCTime clock;
	clock = osal_build_uint32(pkt->appData, pkt->appDataLen);
	osal_setClock(clock);
	return ZSuccess;
}

