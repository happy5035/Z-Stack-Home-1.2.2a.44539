/**
*   协调器处理函数
*/
#include "CoorFunc.h"

#include "hal_uart.h"
#include "hal_led.h"
#include "OSAL_Clock.h"
#include "user_printf.h"

extern byte GenericApp_TransID;
extern endPointDesc_t GenericApp_epDesc;
extern afAddrType_t GenericApp_DstAddr;
#include "OSAL_Nv.h"


//	Local Function
uint8 MasterSetClock(mtSysAppMsg_t *pkt); 
uint8 MasterSetFreq(mtSysAppMsg_t *pkt);


/*   C O O R   P R O C E S S   T E M P   H U M   D A T A   */
/*-------------------------------------------------------------------------
    协调器处理终端发送的温度湿度数据包
-------------------------------------------------------------------------*/
void CoorProcessTempHumData(afIncomingMSGPacket_t *pkt){
#ifdef MT_TASK
	MT_BuildAndSendZToolResponse(MT_RSP_CMD_APP, MT_APP_MSG, pkt->cmd.DataLength, pkt->cmd.Data);
#endif
}


/*   S E N D   C O O R   S T A R T   */
/*-------------------------------------------------------------------------
    通过串口发送协调器启动
-------------------------------------------------------------------------*/
void CoorSendCoorStart(void){
#ifdef MT_TASK
	uint8 cmd = COOR_START_CMD;
	MT_BuildAndSendZToolResponse(MT_RSP_CMD_APP, MT_APP_MSG, 1, &cmd);
#endif
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
		case MASTER_SET_FREQ_CMD:
			retValue = MasterSetFreq(pkt);
			break;
		default:
			break;
	}
    
#ifdef MT_TASK
	MT_BuildAndSendZToolResponse(MT_RSP_CMD_APP, MT_APP_MSG, 1, &retValue);
#endif
	
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



/*   M A S T E R   S E T   F R E Q   */
/*-------------------------------------------------------------------------
    设置采样温度和湿度频率
    |  0-1  |  2-5  |  6-9  |  10-13  |  14-17  |
    |  addr |  temp |  hum  |  packet |  clock  |  
-------------------------------------------------------------------------*/
uint8 MasterSetFreq(mtSysAppMsg_t *pkt){
	uint8* packet;
	uint8 len;
    len = sizeof(uint32) * 4;
	packet = osal_mem_alloc(len);
	uint8 retValue;
	retValue = FAILURE;
	if(packet){
		uint16 shortAddr;
		shortAddr = osal_build_uint16(pkt->appData);
		osal_memcpy(packet, pkt->appData + 2, len);
		GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
		GenericApp_DstAddr.endPoint = GENERICAPP_ENDPOINT;
		GenericApp_DstAddr.addr.shortAddr = shortAddr;
		if (AF_DataRequest(&GenericApp_DstAddr, &GenericApp_epDesc, 
			SYNC_FREQ_CLUSTERID, 
			len, 
		(byte *) packet, 
			&GenericApp_TransID, 
			AF_DISCV_ROUTE, AF_DEFAULT_RADIUS) == afStatus_SUCCESS)
		{
			// Successfully requested to be sent.
			HalLedBlink(HAL_LED_2, 10, 50, 500);
			retValue = SUCCESS;
//			printf("send sync freq success\n");
		}else{
//			printf("send sync freq failed\n");
	    }
	    osal_mem_free(packet);

	}
	
//	printf("alloc send sync freq  packet failed\n");
	return retValue;
}

void CoorProcessEndStatus(afIncomingMSGPacket_t *pkt){
	uint16 paramsFlag;
	paramsFlag = 0;
	uint8* data = pkt->cmd.Data;
	*data++;
	uint8 len = pkt->cmd.DataLength;
	endStatus_t eStatus;
	osal_memcpy(&eStatus,data,sizeof(endStatus_t));
	paramsFlag |= PARAMS_FLAGS_CLOCK;
	uint8 result;
	uint8* buf;
	buf = osal_mem_alloc(4);
	//读取温度采样频率
	result = osal_nv_read(NV_TEMP_SAMPLE_TIME, 0, 4, buf);
	if(result == NV_OPER_FAILED){
		osal_buffer_uint32(buf, eStatus.tempTime);
		osal_nv_item_init(NV_SYNC_CLOCK_TIME, 4, buf);
	}
	uint32 _tempTime =  osal_build_uint32(buf, 4);
	if(_tempTime != eStatus.tempTime){
		paramsFlag |= PARAMS_FLAGS_TEMP_TIME;
	}
	//读取湿度采样频率
	result = osal_nv_read(NV_HUM_SAMPLE_TIME, 0, 4, buf);
	if(result == NV_OPER_FAILED){
		osal_buffer_uint32(buf, eStatus.humTime);
		osal_nv_item_init(NV_HUM_SAMPLE_TIME, 4, buf);
	}
	//读取数据包发送频率
	result = osal_nv_read(NV_PACKET_SEND_TIME, 0, 4, buf);
	if(result == NV_OPER_FAILED){
		osal_buffer_uint32(buf, eStatus.packetTime);
		osal_nv_item_init(NV_PACKET_SEND_TIME, 4, buf);
	}
	//读取时间同步频率
	result = osal_nv_read(NV_SYNC_CLOCK_TIME, 0, 4, buf);
	if(result == NV_OPER_FAILED){
		osal_buffer_uint32(buf, eStatus.syncClockTime);
		osal_nv_item_init(NV_SYNC_CLOCK_TIME, 4, buf);
	}
	
	
	osal_mem_free(buf);
	
	
#ifdef MT_TASK
	MT_BuildAndSendZToolResponse(MT_RSP_CMD_APP, MT_APP_MSG, pkt->cmd.DataLength, pkt->cmd.Data);
#endif
}


