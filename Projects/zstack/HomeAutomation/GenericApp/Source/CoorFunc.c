/**
*   Э����������
*/
#include "CoorFunc.h"

#include "hal_uart.h"
#include "hal_led.h"
#include "OSAL_Clock.h"
#include "user_printf.h"
#include "AddrMgr.h"

extern byte GenericApp_TransID;
extern endPointDesc_t GenericApp_epDesc;
extern afAddrType_t GenericApp_DstAddr;
#include "OSAL_Nv.h"


//	Local Function
uint8 MasterSetClock(mtSysAppMsg_t *pkt); 
uint8 MasterSetFreq(mtSysAppMsg_t *pkt);
uint8 MasterSetNvConfig(mtSysAppMsg_t *pkt);
uint8 MasterGetAddrCount(mtSysAppMsg_t *pkt);

// Local variable
typedef struct {
	uint8 pv;
	uint8 item_size;
	uint8 item_len;
	uint8* items;
}nvConfigItems_t;

nvConfigItems_t nvConfigItems;


/*   C O O R   P R O C E S S   T E M P   H U M   D A T A   */
/*-------------------------------------------------------------------------
    Э���������ն˷��͵��¶�ʪ�����ݰ�
-------------------------------------------------------------------------*/
void CoorProcessTempHumData(afIncomingMSGPacket_t *pkt){
	CoorSendNVConfig( pkt);
		
#ifdef MT_TASK
	uint8* packet;
	uint16 len;
	len = pkt->cmd.DataLength + 2;
	packet = osal_mem_alloc(len);
	if(packet){
		osal_memcpy(packet,pkt->cmd.Data,pkt->cmd.DataLength);
		*(packet + len - 2) = pkt->rssi;
		*(packet + len - 1) = pkt->LinkQuality;
		MT_BuildAndSendZToolResponse(MT_RSP_CMD_APP, MT_APP_MSG, len, packet);
		osal_mem_free(packet);
	}
	
	
#endif
}


/*   S E N D   C O O R   S T A R T   */
/*-------------------------------------------------------------------------
    ͨ�����ڷ���Э��������
-------------------------------------------------------------------------*/
void CoorSendCoorStart(void){
	
nvConfigItems.items = osal_mem_alloc(1);
#ifdef MT_TASK
	uint8 cmd = COOR_START_CMD;
	MT_BuildAndSendZToolResponse(MT_RSP_CMD_APP, MT_APP_MSG, 1, &cmd);
#endif
}

/*   P R O C E S S   M T   S Y S   M S G   */
/*-------------------------------------------------------------------------
    �����ڷ��͵���������
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
		case MASTER_SET_NV_CONFIG_CMD:
			retValue = MasterSetNvConfig(pkt);
		case MASTER_GET_ADDR_COUNT_CMD:
			retValue = MasterGetAddrCount(pkt);
		default:
			break;
	}
    
#ifdef MT_TASK
	MT_BuildAndSendZToolResponse(MT_RSP_CMD_APP, MT_APP_MSG, 1, &retValue);
#endif
	
}
/*   C O R   S E N D   C L O C K   */
/*-------------------------------------------------------------------------
    Э��������ͬ��ʱ����ն�
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
    �����������͵�ͬ��ʱ������
-------------------------------------------------------------------------*/
uint8 MasterSetClock(mtSysAppMsg_t *pkt){
	UTCTime clock;
	clock = osal_build_uint32(pkt->appData, pkt->appDataLen);
	osal_setClock(clock);
	return ZSuccess;
}



/*   M A S T E R   S E T   F R E Q   */
/*-------------------------------------------------------------------------
    ���ò����¶Ⱥ�ʪ��Ƶ��
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


/*   M A S T E R   S E T   N V   C O N F I G   */
/*-------------------------------------------------------------------------
    ���������ն�NV���ò���
    |  0    |   1   |  2    |  3-len  |
    |  pv   |  size |  len  |  items  |
-------------------------------------------------------------------------*/
uint8 MasterSetNvConfig(mtSysAppMsg_t *pkt){
	uint8 len;
	uint8 retValue;
	retValue = FAILURE;
	len = pkt->appDataLen;
	if(len >=3){
		nvConfigItems.pv = *pkt->appData ++;	
		nvConfigItems.item_size = *pkt->appData ++;	
		nvConfigItems.item_len = *pkt->appData ++;	
		if(nvConfigItems.item_len>0){
			uint8* items = osal_mem_alloc(nvConfigItems.item_len);
			if(items){
				osal_mem_free(nvConfigItems.items);
				osal_memcpy(items, pkt->appData, nvConfigItems.item_len);
				nvConfigItems.items = items;
				osal_nv_write(NV_PARAM_VERSION, 0, 1, &nvConfigItems.pv);
				retValue = SUCCESS;
			}
		}
	}
	return retValue;

}


/*   M A S T E R   G E T   A D D R   C O U N T   */
/*-------------------------------------------------------------------------
    ��ȡ
-------------------------------------------------------------------------*/
uint8 MasterGetAddrCount(mtSysAppMsg_t *pkt){
#ifdef MT_TASK
	
	uint8 len; 
	len = NWK_MAX_ADDRESSES*2 + 2;
	uint8 result;
	uint8 i;
	uint8 count;
	count = 0;
	uint8* msg;
	msg = osal_mem_alloc(len);
	uint8 userType = *(pkt->appData);
	if(msg){
		osal_memset(msg, 0xff, len);
		uint8* _msg = msg+2;
		for(i=0;i<NWK_MAX_ADDRESSES;i++){
			AddrMgrEntry_t addrEntry;
			addrEntry.user = userType;
			addrEntry.index = i;
			result = AddrMgrEntryGet(&addrEntry);
			if(result){
				count++;
				*_msg++ = LO_UINT16(addrEntry.nwkAddr);
				*_msg++ = HI_UINT16(addrEntry.nwkAddr);
			}
		}
		
	*msg = MASTER_GET_ADDR_COUNT_CMD;
	*(msg+1) = count;
	MT_BuildAndSendZToolResponse(MT_RSP_CMD_APP, MT_APP_MSG, len, msg);

	}
	else{
		return FAILURE;
	}
#endif
	return SUCCESS;
}

void CoorSendSyncParams(uint8 paramsVersion,uint16 destAddr){
	uint32 paramsFlag = 0;
	uint8* buf = osal_mem_alloc(4);
	osal_nv_read(NV_PARAM_FLAGS,0,4,buf);
	paramsFlag = osal_build_uint32(buf, 4);
	uint32 _paramsFlag = 0;
//	printf("params flags %d",paramsFlag);
	uint8 *packet;
	uint8 len;
	len = 1 + 4+ 4*5 + 2;// pv+pf+4times+timwWindow
	packet = osal_mem_alloc(len);
	uint8* _packet = packet;
	*_packet++ = paramsVersion;
	_packet = osal_buffer_uint32(_packet, paramsFlag);
	
	osal_buffer_uint32(buf, osal_getClock());
	osal_memcpy(_packet,buf,4);
	_paramsFlag |= PARAMS_FLAGS_CLOCK;
	_packet +=4;
	
	if(paramsFlag & PARAMS_FLAGS_TEMP_TIME){
		osal_nv_read(NV_TEMP_SAMPLE_TIME, 0, 4, _packet);
		_paramsFlag |= PARAMS_FLAGS_TEMP_TIME;
		_packet+=4;
	}else{
		len -=4;
	}
	
	if(paramsFlag & PARAMS_FLAGS_HUM_TIME){
		osal_nv_read(NV_HUM_SAMPLE_TIME, 0, 4, _packet);
		_paramsFlag |= PARAMS_FLAGS_HUM_TIME;
		_packet+=4;
	}else{
		len -=4;
	}
	
	if(paramsFlag & PARAMS_FLAGS_PACKET_TIME){
		osal_nv_read(NV_PACKET_SEND_TIME, 0, 4, _packet);
		_paramsFlag |= PARAMS_FLAGS_PACKET_TIME;
		_packet+=4;
	}else{
		len -=4;
	}
	if(paramsFlag & PARAMS_FLAGS_SYNC_CLOCK_TIME){
		osal_nv_read(NV_SYNC_CLOCK_TIME, 0, 4, _packet);
		_paramsFlag |= PARAMS_FLAGS_SYNC_CLOCK_TIME;
		_packet+=4;
	}else{
		len -=4;
	}
	_paramsFlag |= PARAMS_FLAGS_PACKET_TIME_WINDOW;
	uint16 packetTimeWindow = CoorGetPacketTimeWindow();
	osal_memcpy(_packet, &packetTimeWindow, 2);
	if(len >5){
		GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
		GenericApp_DstAddr.endPoint = GENERICAPP_ENDPOINT;
		GenericApp_DstAddr.addr.shortAddr = destAddr;
		if (AF_DataRequest(&GenericApp_DstAddr, &GenericApp_epDesc, 
			SYNC_PARAM_CLUSTERID, 
			len, 
			(byte *) packet, 
			&GenericApp_TransID, 
			AF_DISCV_ROUTE, AF_DEFAULT_RADIUS) == afStatus_SUCCESS){				
		}else{
		}

		
		
		osal_mem_free(packet);
		osal_mem_free(buf);

	}
}

void CoorProcessEndStatus(afIncomingMSGPacket_t *pkt){
//	uint8* data = pkt->cmd.Data;
//	*data++;
//	endStatus_t eStatus;
//	osal_memcpy(&eStatus,data,sizeof(endStatus_t));
//	uint8* buf = osal_mem_alloc(4);
//	osal_nv_read(NV_PARAM_VERSION, 0, 1, buf);
//	uint8 _paramsVersion = *buf;
//	if(_paramsVersion == eStatus.paramsVersion){
//		printf("same params version %d",_paramsVersion);
//	}else{
//		printf("new params version %d",_paramsVersion);
////		CoorSendSyncParams(_paramsVersion,pkt->srcAddr.addr.shortAddr);
//	}
//	osal_mem_free(buf);
	
#ifdef MT_TASK
	uint8* packet;
	uint16 len;
	len = pkt->cmd.DataLength + 2;
	packet = osal_mem_alloc(len);
	if(packet){
		osal_memcpy(packet,pkt->cmd.Data,pkt->cmd.DataLength);
		*(packet + len - 2) = pkt->rssi;
		*(packet + len - 1) = pkt->LinkQuality;
		MT_BuildAndSendZToolResponse(MT_RSP_CMD_APP, MT_APP_MSG, len, packet);
		osal_mem_free(packet);
	}

#endif
}

/*   C O O R   P R O C E S S   E N D   S Y N C   P A R A M S   */
/*-------------------------------------------------------------------------
    �����ն˽ڵ㷢�͵�ͬ����������
-------------------------------------------------------------------------*/
void CoorProcessEndSyncParams(afIncomingMSGPacket_t *pkt){
	uint8 flag= 0;
	uint8 _paramsVersion ;
	if(pkt->cmd.DataLength == 1){
		if(osal_nv_read(NV_PARAM_VERSION, 0, 1, &_paramsVersion) != NV_OPER_FAILED){
			if(_paramsVersion != *pkt->cmd.Data){
				CoorSendSyncParams(_paramsVersion,pkt->srcAddr.addr.shortAddr);
				flag = 1;
			}
		}
		
	}
	if(flag == 0){
		uint8* packet;
		uint8 len = 1+4*2+2;	//pv+paramsflag+timeWindow
		packet = osal_mem_alloc(len);
		if(packet){
			uint8* _packet = packet;
			*_packet++ = _paramsVersion;
			uint32 paramsFlag = 0;
			paramsFlag |= PARAMS_FLAGS_CLOCK;
			paramsFlag |= PARAMS_FLAGS_PACKET_TIME_WINDOW;
			_packet = osal_memcpy(_packet, &paramsFlag, 4);
			UTCTime time = osal_getClock();
			_packet = osal_memcpy(_packet, &time, 4);
			uint16 packetTimeWindow = CoorGetPacketTimeWindow();
			osal_memcpy(_packet, &packetTimeWindow, 2);
			GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
			GenericApp_DstAddr.endPoint = GENERICAPP_ENDPOINT;
			GenericApp_DstAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;
			if (AF_DataRequest(&GenericApp_DstAddr, &GenericApp_epDesc, 
				SYNC_PARAM_CLUSTERID, 
				len, 
				(byte *) packet, 
				&GenericApp_TransID, 
				AF_DISCV_ROUTE, AF_DEFAULT_RADIUS) == afStatus_SUCCESS){				
			}else{
			}
			osal_mem_free(packet);
		}
		
	}
	
}


/*   C O O R   G E T   P A C K E T   T I M E   W I N D O W   */
/*-------------------------------------------------------------------------
    ��ȡ�ն˽ڵ�ʱ�䴰��ֵ
-------------------------------------------------------------------------*/
uint16 CoorGetPacketTimeWindow(){
	uint16 packetTimeWindow;
	if(osal_nv_read(NV_PACKET_TIME_WINDOW, 0, 2, &packetTimeWindow) == NV_OPER_FAILED){
		packetTimeWindow = PACKET_TIME_WINDOW_DEFAULT;
	}
	uint16 interval;
	if(osal_nv_read(NV_PACKET_TIME_WINDOW_INTERVAL, 0, 2, &interval) == NV_OPER_FAILED){
		interval = PACKET_TIME_WINDOW_INTERVAL_DEFAULT;
	}
	packetTimeWindow += interval;
	uint32 packetTime;
	if(osal_nv_read(NV_PACKET_SEND_TIME, 0, 4, &packetTime) == NV_OPER_FAILED){
		packetTimeWindow = PACKET_TIME_WINDOW_DEFAULT;
	}
	packetTime = packetTime / 1000;
	if(packetTimeWindow >= packetTime){
		packetTimeWindow -= packetTime;
	}
	osal_nv_write(NV_PACKET_TIME_WINDOW, 0, 2, &packetTimeWindow);
	return packetTimeWindow;
}


/*   C O O R   S E N D   N   V   C O N F I G   */
/*-------------------------------------------------------------------------
    ��������NV������ն�
-------------------------------------------------------------------------*/
void CoorSendNVConfig(afIncomingMSGPacket_t *pkt){
	uint8 pv;
	pv = *(pkt->cmd.Data + 13);
	uint8 _pv;
	osal_nv_read(NV_PARAM_VERSION, 0, 1, &_pv);
	if(pv!=_pv){
		
		uint8* packet;
		uint8 len;
		len = nvConfigItems.item_len + 2; // pv + itemSize  + item
		packet = osal_mem_alloc(len);
		if(packet){
			uint8* _packet = packet;
			*_packet++ = _pv;
			*_packet++ = nvConfigItems.item_size; //itemSize
//			*_packet++ = nvConfigItems.item_len; //item_len
			osal_memcpy(_packet, nvConfigItems.items, nvConfigItems.item_len);
			GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
			GenericApp_DstAddr.endPoint = GENERICAPP_ENDPOINT;
			GenericApp_DstAddr.addr.shortAddr = pkt->srcAddr.addr.shortAddr;
			if (AF_DataRequest(&GenericApp_DstAddr, &GenericApp_epDesc, 
					SYNC_NV_CONFIG_CLUSTERID, 
					len, //(byte)osal_strlen( theMessageData ) + 1,
				(byte *) packet, 
					&GenericApp_TransID, 
					AF_DISCV_ROUTE, AF_DEFAULT_RADIUS) == afStatus_SUCCESS)
				{
					
				}else{
				}
			osal_mem_free(packet);
		}

	}
}
/*   C O O R   M   T   U A R T   S E R I A L   M S G   P R O C E S S   */
/*-------------------------------------------------------------------------
    ������λ��Զ��MT���ݰ�
-------------------------------------------------------------------------*/
void CoorMTUartSerialMsgProcess(mtOSALSerialData_t *pBuf){
	uint8* packet = pBuf->msg;
	uint8 len = pBuf->msg[0] + 3 ;
	uint16 destAddr;
	uint8 result = osal_nv_read(NV_REMOTE_URART_DEST_ADDR, 0, 2, &destAddr);
	if(result != NV_OPER_FAILED){
		GenericApp_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
		GenericApp_DstAddr.endPoint = GENERICAPP_ENDPOINT;
		GenericApp_DstAddr.addr.shortAddr = 0xFFFF;
		if (AF_DataRequest(&GenericApp_DstAddr, &GenericApp_epDesc, 
					REMOTE_MT_UART_DATA_CLUSTERID, 
			len, //(byte)osal_strlen( theMessageData ) + 1,
			(byte *) packet, 
				&GenericApp_TransID, 
				AF_DISCV_ROUTE, AF_DEFAULT_RADIUS) == afStatus_SUCCESS)
			{
				
			}else{
				
			}
	}
}

/*   C O O R   P R O C E S S   U A R T   R E S P O N S E   */
/*-------------------------------------------------------------------------
    �����ն˷��ص�Զ��MT��Ӧ
-------------------------------------------------------------------------*/
void CoorProcessUartResponse(afIncomingMSGPacket_t *pkt){
#ifdef ZTOOL_P2
  HalUARTWrite(HAL_UART_PORT_1,pkt->cmd.Data,pkt->cmd.DataLength);
#else
  HalUARTWrite(HAL_UART_PORT_0,pkt->cmd.Data,pkt->cmd.DataLength);
#endif
}

/*   C O O R   P R O C E S S   R O U T E R   S T A T U S   */
/*-------------------------------------------------------------------------
    ����·�������͵�״̬���ݰ�
-------------------------------------------------------------------------*/
void CoorProcessRouterStatus(afIncomingMSGPacket_t *pkt){
	
#ifdef MT_TASK
		uint8* packet;
		uint16 len;
		len = pkt->cmd.DataLength + 2;
		packet = osal_mem_alloc(len);
		if(packet){
			osal_memcpy(packet,pkt->cmd.Data,pkt->cmd.DataLength);
			*(packet + len - 2) = pkt->rssi;
			*(packet + len - 1) = pkt->LinkQuality;
			MT_BuildAndSendZToolResponse(MT_RSP_CMD_APP, MT_APP_MSG, len, packet);
			osal_mem_free(packet);
		}
		
		
#endif


}

/*   C O O R   S E N D   A P P   M S G   */
/*-------------------------------------------------------------------------
    ת��app msg ����λ��
-------------------------------------------------------------------------*/
void CoorSendAppMsg( afIncomingMSGPacket_t *pkt){
#ifdef MT_TASK	
	MT_BuildAndSendZToolResponse(MT_RSP_CMD_APP, MT_APP_MSG, pkt->cmd.DataLength, pkt->cmd.Data);
#endif

}



