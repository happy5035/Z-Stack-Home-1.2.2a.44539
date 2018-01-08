/******************************************************************************
  Filename:       GenericApp.c
  Revised:        $Date: 2014-09-07 13:36:30 -0700 (Sun, 07 Sep 2014) $
  Revision:       $Revision: 40046 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2014 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
******************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful - it is
  intended to be a simple example of an application's structure.

  This application periodically sends a "Hello World" message to
  another "Generic" application (see 'txMsgDelay'). The application
  will also receive "Hello World" packets.

  This application doesn't have a profile, so it handles everything
  directly - by itself.

  Key control:
    SW1:  changes the delay between TX packets
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "user_printf.h"
#include "FIFOQueue.h"
#include "OSAL_PwrMgr.h"
#include "sh20.h"
#include "tmp275.h"
#include "OSAL_Nv.h"

#include "GenericApp.h"
#include "DebugTrace.h"
#include "MT_UART.h"
#include "CoorFunc.h"

#if !defined( WIN32 ) || defined( ZBIT )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
#include "hal_adc.h"

/* RTOS */
#if defined( IAR_ARMCM3_LM )
#include "RTOS_App.h"
#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// This list should be filled with Application specific Cluster IDs.
const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS] =
{
  GENERICAPP_CLUSTERID
};

const SimpleDescriptionFormat_t GenericApp_SimpleDesc =
{
  GENERICAPP_ENDPOINT,              //  int Endpoint;
  GENERICAPP_PROFID,                //  uint16 AppProfId[2];
  GENERICAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  GENERICAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  GENERICAPP_FLAGS,                 //  int   AppFlags:4;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList,  //  byte *pAppInClusterList;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in GenericApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t GenericApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte GenericApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // GenericApp_Init() is called.

devStates_t GenericApp_NwkState;

byte GenericApp_TransID;  // This is the unique message ID (counter)

afAddrType_t GenericApp_DstAddr;
afAddrType_t GenericApp_BroadcastAddr;

uint32  sampleTempTimeDelay = 5000;				//1s
uint32 tempPacketSendTimeDelay = 60000;			//30s
uint8 tempPacketSendRetrayTimes = 0;			//温度数据包重复发送次数
uint8 tempPacketSendPacketTransID;
uint32 syncTimeDealy = (uint32)1000*60;
uint16 tempPacketTimeWindow = 1;

uint32 sampleHumTimeDelay = 10000;// 60s一次采集湿度数据

uint8 sampleTask = 0x00;

uint32 requestSyncClockDelay = 600000; //10分钟同步一次时间

uint8 paramsVersion = 0;

//发送数据包
typedef struct
{
	uint16  netAddr;
	uint8 	extAddr[Z_EXTADDR_LEN];
	uint16 	vdd;
	uint8   paramsVersion;
	UTCTime tempStartTime;
	uint16	sampleFreq;
	uint8 	tempNumbers;
	UTCTime humStartTime;
	uint16	humFreq;
	uint8 	humNumbers;
}tempPacket_t;
//存储温度数据
FifoQueue tempQueue;
uint8 extAddr[Z_EXTADDR_LEN];
tempPacket_t packetHead; 
//存储湿度数据
static FifoQueue humQueue;
uint32 startSampleTime;


typedef struct{
	osal_event_hdr_t event;
	uint8 status;
}tempMeasureMsg_t;
uint8 tempStatus = 0;
uint32 lastSampleTempClock; //上次采集温度时间
uint32 lastSampleHumClock; //上次采集湿度时间


typedef struct{
	uint16  netAddr;
	uint8 	extAddr[Z_EXTADDR_LEN];
	uint16 	vdd;
	uint8	paramsVersion;
	UTCTime	clock;
	uint32	tempTime;
	uint32	humTime;
	uint32	packetTime;
	uint32	syncClockTime; 
}endStatus_t;
endStatus_t endStatus;

typedef enum{
	startProcessInit,
	startProcessReport,
	startProcessSync,
	startProcessStartTimer,
	startProcessOver
}startProcessStatus_e;
startProcessStatus_e startProcessStatus;


byte reportPacketID;
//uint8 reportStatus; 
typedef enum{
	reportInit,
	reportSend,
	reportConfirm,
	reportSucess
}reportStatus_e;
reportStatus_e reportStatus;
uint32 reportConfirmTimeOut = END_REPORT_COMFIRM_TIMEOUT;
uint8 reportReSendTimes = END_REPORT_RE_SEND_TIMES;

//uint8	syncParamsStatus = SYNC_PARAMS_STATUS_INIT;
uint32	syncParamsTimeout = SYNC_PARAMS_CONFIRM_TIMEOUT;
uint8	syncParamsTimes = SYNC_PARAMS_STATUS_TIMES;



/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void GenericApp_HandleKeys( byte shift, byte keys );
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );

//sample and send  temp humi
static void EndSampleTempHandler(void);
static void EndSampleHumHandler(void);
static void EndTempPacketSendHandler(void);
static void EndTempSampleCfg(void);
static void EndSampleTask(void);
static int16 EndReadTemp(void);
static int16 EndReadHum(void);
static uint16 EndReadVcc(void);
static uint8* EndBuildTempSendPacket(uint8*,uint8*,uint8*);

// sync time
static void EndSetClock(afIncomingMSGPacket_t *pkt);
static void EndRequestSyncClock(void);

//sync freq
static void EndSetFreq(afIncomingMSGPacket_t *pkt);

static void EndStartProcess(void);
static void EndReportStatus(void);
static void EndSyncParams(afIncomingMSGPacket_t *pkt);
static void EndReadNvParams(void);
static void EndSyncParamsProcess(uint8 status);


/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void GenericApp_Init( uint8 task_id )
{
  GenericApp_TaskID = task_id;
  GenericApp_NwkState = DEV_INIT;
  GenericApp_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  GenericApp_DstAddr.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_DstAddr.addr.shortAddr = 0x00;
  
	//广播地址
	GenericApp_BroadcastAddr.addrMode = (afAddrMode_t)
	AddrBroadcast;
	GenericApp_BroadcastAddr.endPoint = GENERICAPP_ENDPOINT;
	GenericApp_BroadcastAddr.addr.shortAddr = 0xFFFF;
  

  // Fill out the endpoint description.
  GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_epDesc.task_id = &GenericApp_TaskID;
  GenericApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
  GenericApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &GenericApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( GenericApp_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "GenericApp", HAL_LCD_LINE_1 );
#endif

//  ZDO_RegisterForZDOMsg( GenericApp_TaskID, End_Device_Bind_rsp );
//  ZDO_RegisterForZDOMsg( GenericApp_TaskID, Match_Desc_rsp );
	//初始化模块extAddr
	ZMacGetReq( ZMacExtAddr, extAddr );
    //温度数据队列初始化
    QueueInit(&tempQueue);
	QueueInit(&humQueue);

//	ZDO_RegisterForZDOMsg(GenericApp_TaskID, Device_annce);
//    EndTempSampleCfg();
}

/*********************************************************************
 * @fn      GenericApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 GenericApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          GenericApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          GenericApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;

          sentEP = afDataConfirm->endpoint;
          (void)sentEP;  // This info not used now
          sentTransID = afDataConfirm->transID;
          (void)sentTransID;  // This info not used now

          sentStatus = afDataConfirm->hdr.status;
          // Action taken when confirmation is received.
          #ifndef RTR_NWK
          printf("%d data confirm,status: 0x%x\n", sentTransID,sentStatus);
		  #endif
		  if(sentTransID == reportPacketID){
			reportStatus = reportConfirm;
			EndReportStatus();
		  }
          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
          }
          break;

        case AF_INCOMING_MSG_CMD:
          GenericApp_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          GenericApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (GenericApp_NwkState == DEV_ZB_COORD) ||
               (GenericApp_NwkState == DEV_ROUTER) ||
               (GenericApp_NwkState == DEV_END_DEVICE) )
          {
          	HalLedSet(HAL_LED_ALL, HAL_LED_MODE_OFF);
          	if(GenericApp_NwkState == DEV_ZB_COORD){
				printf("coordinator start...\n");
				HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
				CoorSendCoorStart();
//				osal_setClock(0x21AAEBCB);  //2017/11/24 16:40:00
			}
			if(GenericApp_NwkState == DEV_ROUTER){
				printf("router start...\n");
				HalLedSet(HAL_LED_2, HAL_LED_MODE_ON);
			}
			if(GenericApp_NwkState == DEV_END_DEVICE){
				startProcessStatus = startProcessInit;
				EndStartProcess();
				
			}
            
          }
          break;
		case MT_SYS_APP_MSG:
			CoorProcessMtSysMsg((mtSysAppMsg_t *)MSGpkt);
			break;
        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
	if(events & SAMPLE_TEMP_EVT){
//		sampleTask |= SAMPLE_TEMP_START_TASK | SAMPLE_TEMP_READY_TASK;
//		EndSampleTask();
		EndReadTemp();
		
		return events ^ SAMPLE_TEMP_EVT;
	}
	
	if(events & SAMPLE_HUM_EVT){
//		sampleTask |= SAMPLE_HUM_START_TASK | SAMPLE_HUM_READY_TASK;
//		EndSampleTask();
		return events ^ SAMPLE_HUM_EVT;
	}
	
	if(events & SAMPLE_TASK_EVT){
		EndSampleTask();
		return events ^ SAMPLE_TASK_EVT;
	}


	
	if(events & TEMP_PACKET_SEND_EVT){
//		uint16 random;
//		random = (uint16)(((float)osal_rand() / 65535) * tempPacketTimeWindow * tempPacketSendTimeDelay) ;
//		printf("random window:%u\n",random);
		EndTempPacketSendHandler();
//		osal_start_timerEx(GenericApp_TaskID, TEMP_PACKET_SEND_EVT, tempPacketSendTimeDelay + random);
		return events ^ TEMP_PACKET_SEND_EVT;
	}

	if(events & REQUEST_SYNC_CLOCK_EVT){
		EndRequestSyncClock();
		return events^ REQUEST_SYNC_CLOCK_EVT;
	}

	if(events & END_REPORT_CONFIRM_TIMEOUT_EVT){
		if (reportStatus != reportSend){
			osal_stop_timerEx(GenericApp_TaskID, END_REPORT_CONFIRM_TIMEOUT_EVT);
		}else{	
			reportReSendTimes --;
			if(reportConfirm <=0  && reportConfirmTimeOut < 60000){
				reportConfirmTimeOut =2 * reportConfirmTimeOut;
				reportReSendTimes = END_REPORT_RE_SEND_TIMES;
			}
		}	
		EndReportStatus();
		return events^ END_REPORT_CONFIRM_TIMEOUT_EVT;
	}

	if(events & END_SYNC_PARAMS_TIEMOUT_EVT){
		EndSyncParamsProcess(SYNC_PARAMS_STATUS_TIMEOUT);
		return events ^ END_SYNC_PARAMS_TIEMOUT_EVT;
	}

	
//	if(events & SAMPLE_TEMP_READY_EVT){
////		EndSampleTask();
//		uint16 res;
//		res = TMP275_ReadTemp();
//		float real;
//		real = res * 0.0625 ;
//		int16 result;
//		result = (int16) (real * 100);
//		printf("tmp:%d.%02d",result/100,result % 100);
//		osal_start_timerEx(GenericApp_TaskID, SAMPLE_TEMP_EVT, sampleTempTimeDelay);
//		return events ^ SAMPLE_TEMP_READY_EVT;
//	}

  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      GenericApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined( BLINK_LEDS )
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            GenericApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            GenericApp_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}


static void GenericApp_HandleKeys( uint8 shift, uint8 keys )
{
  
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
    case GENERICAPP_CLUSTERID:
      break;
    case TEMP_HUM_DATA_CLUSTERID:
		HalLedSet ( HAL_LED_4, HAL_LED_MODE_BLINK );  // Blink an LED
		CoorProcessTempHumData(pkt);
   		break;
	case SYNC_TIME_CLUSTERID:
		EndSetClock(pkt);
		break;
	case REQUEST_SYNC_CLOCK_CLUSTERID:
		CoorSendSyncClock(pkt);
		break;
	case SYNC_FREQ_CLUSTERID:
		EndSetFreq(pkt);
		break;
	case END_STATUS_CLUSTERID:
		CoorProcessEndStatus(pkt);
		break;
	case SYNC_PARAM_CLUSTERID:
		EndSyncParams(pkt);
		break;
	default :
		break;
  }
}
/*   U P D A T E   N V   P A R A M S   */
/*-------------------------------------------------------------------------
    读取flash的参数并更新
-------------------------------------------------------------------------*/
static void EndReadNvParams(){
	uint8 result;
	uint8* buf;
	buf = osal_mem_alloc(4);
	//读取参数版本
	result = osal_nv_read(NV_PARAM_VERSION, 0, 1, buf);
	if(result == NV_OPER_FAILED){
		*buf = paramsVersion;
		osal_nv_item_init(NV_PARAM_VERSION, 1, buf);
	}else{
		paramsVersion = *buf;
	}
	//读取温度采样频率
	result = osal_nv_read(NV_TEMP_SAMPLE_TIME, 0, 4, buf);
	if(result == NV_OPER_FAILED){
		osal_buffer_uint32(buf, sampleTempTimeDelay);
		osal_nv_item_init(NV_TEMP_SAMPLE_TIME, 4, buf);
	}else{
		sampleTempTimeDelay = osal_build_uint32(buf, 4);
		
	}
	//读取湿度采样频率
	result = osal_nv_read(NV_HUM_SAMPLE_TIME, 0, 4, buf);
	if(result == NV_OPER_FAILED){
		osal_buffer_uint32(buf, sampleHumTimeDelay);
		osal_nv_item_init(NV_HUM_SAMPLE_TIME, 4, buf);
	}else{
		sampleHumTimeDelay = osal_build_uint32(buf, 4);
	}
	//读取数据包发送频率
	result = osal_nv_read(NV_PACKET_SEND_TIME, 0, 4, buf);
	if(result == NV_OPER_FAILED){
		osal_buffer_uint32(buf, tempPacketSendTimeDelay);
		osal_nv_item_init(NV_PACKET_SEND_TIME, 4, buf);
	}else{
		tempPacketSendTimeDelay = osal_build_uint32(buf, 4);
	}
	//读取时间同步频率
	result = osal_nv_read(NV_SYNC_CLOCK_TIME, 0, 4, buf);
	if(result == NV_OPER_FAILED){
		osal_buffer_uint32(buf, requestSyncClockDelay);
		osal_nv_item_init(NV_SYNC_CLOCK_TIME, 4, buf);
	}else{
		requestSyncClockDelay = osal_build_uint32(buf, 4);
	}
	osal_mem_free(buf);
}

static void EndStartProcess(){
	if(startProcessStatus == startProcessInit){
		printf("end device start...\n");
		HalLedSet(HAL_LED_3, HAL_LED_MODE_ON);
		reportStatus = reportInit;
		startProcessStatus = startProcessReport;
		EndReadNvParams();
	}
	if(startProcessStatus == startProcessReport){
		EndReportStatus();
	}
	if(startProcessStatus == startProcessSync){
		EndSyncParamsProcess(SYNC_PARAMS_STATUS_INIT);
		startProcessStatus = startProcessStartTimer;
	}
	
	if(startProcessStatus == startProcessStartTimer){
		
		//开启采样和发送定时器
		//EndRequestSyncClock();
		osal_start_reload_timer(GenericApp_TaskID, SAMPLE_TEMP_EVT, sampleTempTimeDelay);
	//	osal_start_reload_timer(GenericApp_TaskID, SAMPLE_HUM_EVT, sampleHumTimeDelay);
		osal_start_reload_timer(GenericApp_TaskID, TEMP_PACKET_SEND_EVT, tempPacketSendTimeDelay);
		EndTempSampleCfg();
		osal_start_reload_timer(GenericApp_TaskID, REQUEST_SYNC_CLOCK_EVT, requestSyncClockDelay);
		osal_pwrmgr_device(PWRMGR_BATTERY);
		startProcessStatus = startProcessOver;
	}
	if(startProcessStatus == startProcessOver){
		startProcessStatus = startProcessInit;
		printf("start process over\n");
	}
	
    


}

/*   E N D   R E P O R T   S T A T U S   */
/*-------------------------------------------------------------------------
    终端报告设备状态
-------------------------------------------------------------------------*/
static void EndReportStatus(void){
	if(reportStatus == reportInit){
		endStatus.netAddr = NLME_GetShortAddr();
			osal_memcpy(endStatus.extAddr,extAddr,Z_EXTADDR_LEN);
			endStatus.vdd = EndReadVcc();
			endStatus.clock = osal_getClock();
			endStatus.paramsVersion = paramsVersion;
			uint8* buf ;
			buf = osal_mem_alloc(4);
			uint8 result;
			//读取温度采样频率
			result = osal_nv_read(NV_TEMP_SAMPLE_TIME, 0, 4, buf);
			if(result == NV_OPER_FAILED){
				osal_buffer_uint32(buf, sampleTempTimeDelay);
				result = osal_nv_item_init(NV_TEMP_SAMPLE_TIME, 4, buf);
				endStatus.tempTime = sampleTempTimeDelay;
				if(result !=NV_OPER_FAILED ){
					printf("init nv temp time success");
				}
					
				else{
					printf("init nv temp time failed");
				}
			}else{
				endStatus.tempTime =  osal_build_uint32(buf, 4);
			}
			//读取湿度采样频率
			result = osal_nv_read(NV_HUM_SAMPLE_TIME, 0, 4, buf);
			if(result == NV_OPER_FAILED){
				osal_buffer_uint32(buf, sampleHumTimeDelay);
				result = osal_nv_item_init(NV_HUM_SAMPLE_TIME, 4, buf);
				endStatus.humTime = sampleHumTimeDelay;
				if(result !=NV_OPER_FAILED ){
					printf("init nv hum time success");
				}
					
				else{
					printf("init nv hum time failed");
				}
			}else{
				endStatus.humTime =  osal_build_uint32(buf, 4);
			}
			
			//读取数据包发送频率
			result = osal_nv_read(NV_PACKET_SEND_TIME, 0, 4, buf);
			if(result == NV_OPER_FAILED){
				osal_buffer_uint32(buf, tempPacketSendTimeDelay);
				result = osal_nv_item_init(NV_PACKET_SEND_TIME, 4, buf);
				endStatus.packetTime = tempPacketSendTimeDelay;
				printf("init nv sample time");
				if(result !=NV_OPER_FAILED ){
					printf("init nv packet time success");
				}
					
				else{
					printf("init nv packet time failed");
				}
			}else{
				endStatus.packetTime =	osal_build_uint32(buf, 4);
			}
			
			//读取同步时钟发送频率
			result = osal_nv_read(NV_SYNC_CLOCK_TIME, 0, 4, buf);
			if(result == NV_OPER_FAILED){
				osal_buffer_uint32(buf, requestSyncClockDelay);
				result = osal_nv_item_init(NV_SYNC_CLOCK_TIME, 4, buf);
				endStatus.syncClockTime = requestSyncClockDelay;
				printf("init nv sample time");
				if(result !=NV_OPER_FAILED ){
					printf("init nv packet time success");
				}
					
				else{
					printf("init nv packet time failed");
				}
			}else{
				endStatus.syncClockTime =  osal_build_uint32(buf, 4);
			}
			reportStatus = reportSend;
			osal_mem_free(buf);

	}
	if( reportStatus == reportSend){
		uint8* packet;
		uint8 len;
		len = sizeof(endStatus_t)+1;
		packet = osal_mem_alloc(len);
		*packet = END_REPORT_STATUS_CMD;
		osal_memcpy(packet+1,&endStatus,len-1);
		if(packet){
			GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
			GenericApp_DstAddr.endPoint = GENERICAPP_ENDPOINT;
			GenericApp_DstAddr.addr.shortAddr = 0x00;
			reportPacketID = GenericApp_TransID;
			if (AF_DataRequest(&GenericApp_DstAddr, &GenericApp_epDesc, 
					END_STATUS_CLUSTERID, 
					len, //(byte)osal_strlen( theMessageData ) + 1,
				(byte *) packet, 
					&GenericApp_TransID, 
					AF_DISCV_ROUTE | AF_ACK_REQUEST, AF_DEFAULT_RADIUS) == afStatus_SUCCESS)
				{
					// Successfully requested to be sent.
					HalLedBlink(HAL_LED_4, 1, 50, 500);
					printf("send status success\n");
				}else{
					printf("send status failed\n");
				}
		}
		osal_start_reload_timer(GenericApp_TaskID, END_REPORT_CONFIRM_TIMEOUT_EVT, reportConfirmTimeOut);

	}
	if(reportStatus == reportConfirm){
		osal_stop_timerEx(GenericApp_TaskID, END_REPORT_CONFIRM_TIMEOUT_EVT);
		reportConfirmTimeOut = END_REPORT_COMFIRM_TIMEOUT;
		reportReSendTimes = END_REPORT_RE_SEND_TIMES;
		reportStatus = reportSucess;	
	}
	if(reportStatus == reportSucess){
		printf("report success");
		reportStatus = reportInit;
		startProcessStatus = startProcessSync;
		EndStartProcess();
	}
	
	
}


/*   E N D   S Y N C   P A R A M S   P R O C E S S   */
/*-------------------------------------------------------------------------
    终端节点发送同步参数请求并同步参数函数，在同步失败之后能重新发送同步请求
-------------------------------------------------------------------------*/
static void EndSyncParamsProcess(uint8 status){
	if(status == SYNC_PARAMS_STATUS_INIT){
		//初始状态
		status = SYNC_PARAMS_STATUS_SEND;
	}
	if(status == SYNC_PARAMS_STATUS_SEND){
		//发送参数同步请求
		GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
		GenericApp_DstAddr.endPoint = GENERICAPP_ENDPOINT;
		GenericApp_DstAddr.addr.shortAddr = 0x00;
		if (AF_DataRequest(&GenericApp_DstAddr, &GenericApp_epDesc, 
					END_SYNC_PARAMS_CLUSTERID, 
					1, //(byte)osal_strlen( theMessageData ) + 1,
				(byte *) &paramsVersion, 
					&GenericApp_TransID, 
					AF_DISCV_ROUTE | AF_ACK_REQUEST, AF_DEFAULT_RADIUS) == afStatus_SUCCESS)
		{
			// Successfully requested to be sent.
			printf("send sync params success\n");
		}else{
			printf("send sync params failed\n");
		}
		//syncParamsStatus = SYNC_PARAMS_STATUS_RECEIVE;
		osal_start_timerEx(GenericApp_TaskID, END_SYNC_PARAMS_TIEMOUT_EVT, syncParamsTimeout);
	}
	if(status == SYNC_PARAMS_STATUS_TIMEOUT){
		//在规定时间内未收到同步参数命令
		syncParamsTimes -- ;
		if(syncParamsTimes <= 0 ){
			syncParamsTimeout = 2 * syncParamsTimeout;
			syncParamsTimes = SYNC_PARAMS_STATUS_TIMES;
		}
		osal_start_timerEx(GenericApp_TaskID, END_SYNC_PARAMS_TIEMOUT_EVT, syncParamsTimeout);
//		syncParamsStatus = SYNC_PARAMS_STATUS_SEND;
		EndSyncParamsProcess(SYNC_PARAMS_STATUS_SEND);
		printf("sync params status timeout %d \n",syncParamsTimeout);
	}
	if(status == SYNC_PARAMS_STATUS_RECEIVE){
		//已经收到同步参数命令
		
		osal_stop_timerEx(GenericApp_TaskID, END_SYNC_PARAMS_TIEMOUT_EVT);
		syncParamsTimeout = SYNC_PARAMS_CONFIRM_TIMEOUT;
		syncParamsTimes = SYNC_PARAMS_STATUS_TIMES;
		status = SYNC_PARAMS_STATUS_END;
	}
	if(status == SYNC_PARAMS_STATUS_END){
		printf("sync params end\n");
//		syncParamsStatus = SYNC_PARAMS_STATUS_END;
	}
}


/*   E N D   S Y N C   P A R A M S   */
/*-------------------------------------------------------------------------
    终端节点同步协调器设置参数
-------------------------------------------------------------------------*/
static void EndSyncParams(afIncomingMSGPacket_t* pkt){
	uint8* data = pkt->cmd.Data;
	uint8 len = pkt->cmd.DataLength;
	if(len >=5){
		uint8 _paramsVersion  = *data++;
		paramsVersion = _paramsVersion;
		osal_nv_write(NV_PARAM_VERSION, 0, 1, &paramsVersion);
		uint32 paramsFlags = osal_build_uint32(data, 4);
		data +=4;
		if(paramsFlags!=0){
			if(paramsFlags & PARAMS_FLAGS_CLOCK){
				osal_setClock(osal_build_uint32(data, 4));
				printf("new clock :%d\n",osal_getClock());
				data+=4;
			}
			uint8* buf;
			buf = osal_mem_alloc(4);
			if(paramsFlags & PARAMS_FLAGS_TEMP_TIME){
				uint32 _sampleTempTimeDelay = osal_build_uint32(data, 4);
				if(_sampleTempTimeDelay != sampleTempTimeDelay){
					sampleTempTimeDelay = _sampleTempTimeDelay;
					osal_start_reload_timer(GenericApp_TaskID, SAMPLE_TEMP_EVT, sampleTempTimeDelay);
					osal_buffer_uint32( buf, sampleTempTimeDelay);
					osal_nv_write(NV_TEMP_SAMPLE_TIME, 0, 4,  buf);
					printf("new temp time:%d\n",sampleTempTimeDelay);
				}
				data+=4;
			}
			if(paramsFlags & PARAMS_FLAGS_HUM_TIME){
				uint32 _sampleHumTimeDelay = osal_build_uint32(data, 4);
				if(_sampleHumTimeDelay != sampleHumTimeDelay){
					sampleHumTimeDelay = _sampleHumTimeDelay;
//					osal_start_reload_timer(GenericApp_TaskID, SAMPLE_HUM_EVT, sampleHumTimeDelay);
					osal_buffer_uint32( buf, sampleHumTimeDelay);
					osal_nv_write(PARAMS_FLAGS_HUM_TIME, 0, 4,  buf);
					printf("new hum time:%d\n",sampleHumTimeDelay);
				}
				data+=4;
			}
			if(paramsFlags & PARAMS_FLAGS_PACKET_TIME){
				uint32 _tempPacketSendTimeDelay = osal_build_uint32(data, 4);
				if(_tempPacketSendTimeDelay != tempPacketSendTimeDelay){
					tempPacketSendTimeDelay = _tempPacketSendTimeDelay;
					osal_start_reload_timer(GenericApp_TaskID, TEMP_PACKET_SEND_EVT, tempPacketSendTimeDelay);
					osal_buffer_uint32( buf, tempPacketSendTimeDelay);
					osal_nv_write(PARAMS_FLAGS_PACKET_TIME, 0, 4,  buf);
					printf("new packet time:%d\n",tempPacketSendTimeDelay);
				}
				data+=4;
			}
			if(paramsFlags & PARAMS_FLAGS_SYNC_CLOCK_TIME){
				uint32 _requestSyncClockDelay = osal_build_uint32(data, 4);
				if(_requestSyncClockDelay != requestSyncClockDelay){
					requestSyncClockDelay = _requestSyncClockDelay;
					osal_start_reload_timer(GenericApp_TaskID, REQUEST_SYNC_CLOCK_EVT, requestSyncClockDelay);
					osal_buffer_uint32( buf, requestSyncClockDelay);
					osal_nv_write(PARAMS_FLAGS_SYNC_CLOCK_TIME, 0, 4,  buf);
					printf("new sync clock time:%d\n",requestSyncClockDelay);
				}
				data+=4;
			}

			osal_mem_free(buf);
			
			
		}
		
	}
//	syncParamsStatus = SYNC_PARAMS_STATUS_RECEIVE;
	EndSyncParamsProcess(SYNC_PARAMS_STATUS_RECEIVE);
}


/*   E N D   S E T   C L O C K   */
/*-------------------------------------------------------------------------
    设置终端时间
-------------------------------------------------------------------------*/
static void EndSetClock(afIncomingMSGPacket_t *pkt){
	uint8 dataLength;
	uint8* data;
	UTCTime clock;
	dataLength = pkt->cmd.DataLength;
	data = pkt->cmd.Data;
	clock = osal_build_uint32(data, dataLength);
	osal_setClock(clock);
	printf("sync:%02x%02x%02x%02x\n"
	,BREAK_UINT32(clock, 3)
	,BREAK_UINT32(clock, 2)
	,BREAK_UINT32(clock, 1)
	,BREAK_UINT32(clock, 0)
	);
}
/*   E N D   S E T   F R E Q   */
/*-------------------------------------------------------------------------
    设置终端温湿度采样频率
-------------------------------------------------------------------------*/
static void EndSetFreq(afIncomingMSGPacket_t *pkt){
	uint32 tempFreq;
	uint32 humFreq;
	uint32 packetFreq;
	uint32 clockFreq;
	tempFreq = osal_build_uint32(pkt->cmd.Data,4);
	humFreq = osal_build_uint32(pkt->cmd.Data + 4,4);
	packetFreq = osal_build_uint32(pkt->cmd.Data + 8,4);
	clockFreq = osal_build_uint32(pkt->cmd.Data + 12,4);
	
//				EndTempSampleCfg();
	if(tempFreq >= 1000){
		sampleTempTimeDelay = tempFreq;
		osal_start_timerEx(GenericApp_TaskID, SAMPLE_TEMP_EVT, sampleTempTimeDelay);
	}
	if(humFreq >= 10000){
		sampleHumTimeDelay = humFreq;
        osal_start_reload_timer(GenericApp_TaskID, SAMPLE_HUM_EVT, sampleHumTimeDelay);
	}
	if(packetFreq >= 10000){
		tempPacketSendTimeDelay = packetFreq;
		osal_start_reload_timer(GenericApp_TaskID, TEMP_PACKET_SEND_EVT, tempPacketSendTimeDelay);
	}
	if(clockFreq >= 10000){
		requestSyncClockDelay = clockFreq;
		osal_start_reload_timer(GenericApp_TaskID, REQUEST_SYNC_CLOCK_EVT, requestSyncClockDelay);
	}
	printf("sync temp freq:%d,hum:%d,packet:%d,clock:%d\n",tempFreq,humFreq,packetFreq,clockFreq);
}

/*   E N D   R E Q U E S T   S Y N C   C L O C K   */
/*-------------------------------------------------------------------------
    发送请求同步时间
-------------------------------------------------------------------------*/
static void EndRequestSyncClock(){
	uint8* packet;
	uint8 len;
	len = sizeof(UTCTime) + 1;
	packet = osal_mem_alloc(len);
	if(packet){
		*packet = REQUEST_SYNC_CLOCK_CMD;
		UTCTime clock;
		clock = osal_getClock();
		osal_buffer_uint32(packet+1, clock);
		  GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
		  GenericApp_DstAddr.endPoint = GENERICAPP_ENDPOINT;
		  GenericApp_DstAddr.addr.shortAddr = 0x00;
		if (AF_DataRequest(&GenericApp_DstAddr, &GenericApp_epDesc, 
			REQUEST_SYNC_CLOCK_CLUSTERID, 
			len, 
		(byte *) packet, 
			&GenericApp_TransID, 
			AF_DISCV_ROUTE, AF_DEFAULT_RADIUS) == afStatus_SUCCESS)
		{
			// Successfully requested to be sent.
			HalLedBlink(HAL_LED_4, 1, 50, 500);
			printf("send request sync clock success\n");
		}else{
			printf("send request sync clock failed\n");
	    }
	    osal_mem_free(packet);
    }else{
		printf("build sync clock packet failed\n");
	}
}

/*   E N D S A M P L E   T A S K   */
/*-------------------------------------------------------------------------
    采集任务
-------------------------------------------------------------------------*/
static void EndSampleTask(void){
	while(sampleTask){
		if(sampleTask & SAMPLE_TEMP_START_TASK){
			Set_Resolution(RESOLUTION_T11);
			SHT2X_StartMeasureNHM(TEMP_MEASURE_N_MASTER);
			sampleTask ^= SAMPLE_TEMP_START_TASK;
			break;
		}
		if(sampleTask & SAMPLE_TEMP_READY_TASK){
			EndSampleTempHandler();
			sampleTask ^= SAMPLE_TEMP_READY_TASK;
			break;
		}
		if(sampleTask & SAMPLE_HUM_START_TASK){
			Set_Resolution(RESOLUTION_T11);
			SHT2X_StartMeasureNHM(HUMI_MEASURE_N_MASTER);
			sampleTask ^= SAMPLE_HUM_START_TASK;
			break;
		}
		if(sampleTask & SAMPLE_HUM_READY_TASK){
			EndSampleHumHandler();
			sampleTask ^= SAMPLE_HUM_READY_TASK;
			break;
		}
		//错误的状态
		sampleTask = 0;
	}
	if(sampleTask){
		osal_start_timerEx(GenericApp_TaskID, SAMPLE_TASK_EVT, SAMPLE_TASK_WAITE_TIMEOUT);
	}
}


/*   S A M P L E   T E M P   H A N D L E R   */
/*-------------------------------------------------------------------------
    温度采集处理函数
-------------------------------------------------------------------------*/
static void EndSampleTempHandler(void){

	uint16 temp;
	sendData_t dataPacket;
	temp = EndReadTemp();
	dataPacket.utcSecs = lastSampleTempClock;
	lastSampleTempClock = osal_getClock();
	dataPacket.data = temp;
	uint8 res;
	res = QueueIn(&tempQueue,&dataPacket);
	if(res == QueueFull){
		printf("temp queue full");
//		osal_stop_timerEx(uint8 task_id, uint16 event_id)
	}
	osal_start_timerEx(GenericApp_TaskID, SAMPLE_TEMP_EVT, sampleTempTimeDelay);
	
}

/*   S A M P L E   H U M   H A N D L E R   */
/*-------------------------------------------------------------------------
    湿度采集处理函数
-------------------------------------------------------------------------*/
static void EndSampleHumHandler(void){
	uint16 hum;
	sendData_t dataPacket;
	hum = EndReadHum();
	dataPacket.data = hum;
	dataPacket.utcSecs = lastSampleHumClock;
	lastSampleHumClock = osal_getClock();
	uint8 res = QueueIn(&humQueue, &dataPacket);
	
	if(res == QueueFull){
		printf("hum queue full");
//		osal_stop_timerEx(uint8 task_id, uint16 event_id)
	}
}

static void EndTempPacketSendHandler(void){
	
	uint8* packet;
	uint8 temp_len;
	uint8 hum_len;
	uint8 total_len;
    packet = EndBuildTempSendPacket(&total_len,&temp_len,&hum_len);
	if(packet){
	  GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
	  GenericApp_DstAddr.endPoint = GENERICAPP_ENDPOINT;
	  GenericApp_DstAddr.addr.shortAddr = 0x00;
		if (AF_DataRequest(&GenericApp_DstAddr, &GenericApp_epDesc, 
			TEMP_HUM_DATA_CLUSTERID, 
			total_len, //(byte)osal_strlen( theMessageData ) + 1,
		(byte *) packet, 
			&GenericApp_TransID, 
			AF_DISCV_ROUTE, AF_DEFAULT_RADIUS) == afStatus_SUCCESS)
		{
			// Successfully requested to be sent.
			HalLedBlink(HAL_LED_4, 1, 50, 500);
			printf("send temp success\n");
			QueueRemove(&tempQueue,temp_len);
			QueueRemove(&humQueue, hum_len);
		}else{
			printf("send temp failed\n");
	    }
	    osal_mem_free(packet);
    }else{
		printf("build packet failed\n");
	}
}

/**
*	温度传感器配置
*/
static void EndTempSampleCfg(void){
//	TR0 				= 0X01; 					/*这里我让AD和温度传感器相连*/
//	ATEST				= 0X01; 					/*启动温度传感器*/
//	tempStatus = TEMP_MEAUSRE_START_STATUS;
	startSampleTime = osal_getClock();
	if(TMP275_startMeasure()){ 
		printf("start measure success\n");
	}else{
		printf("start measure failed\n"); 
	}

}

static int16 EndReadTemp(){
    int16 temp;
	temp = TMP275_ReadTemp();
	sendData_t dataPacket;
	dataPacket.utcSecs = startSampleTime;
	startSampleTime = osal_getClock();
	dataPacket.data = temp;
	uint8 res;
	res = QueueIn(&tempQueue,&dataPacket);
//	if(res == QueueFull){
//		printf("temp queue full");
//	}
//	printf("temp:%d.%02d\n",temp/100,temp%100);

	if(TMP275_startMeasure()){ 
//		printf("start measure success\n");
	}else{
//		printf("start measure failed\n"); 
	}

	return res;

}

static int16 EndReadHum(void){
	int16 res;
	res = 0xFFFF;
	if(SHT2X_MeasureReady(HUMI_MEASURE_N_MASTER)){
		res = SHT2X_ReadMeasure(HUMI_MEASURE_N_MASTER);
	}

	printf("hum:%d.%02d\n",res/100,res %100);
	return res;
}

static uint16 EndReadVcc(void){
	HalAdcSetReference(HAL_ADC_REF_125V);
	uint16 adcValue;
	float vcc;
	adcValue =  HalAdcRead(HAL_ADC_CHANNEL_VDD, HAL_ADC_RESOLUTION_10);
	if(adcValue > 512 ){
		printf("read vcc negtive\n");
		return 0;
	}
	vcc = 3 * 1.15 * adcValue / 511;
	return (uint16)(vcc * 100);
}
/**
*	构造发送数据包，返回数据包制作并修改数据包长度
*/
static uint8* EndBuildTempSendPacket(uint8* total_len,uint8 *temp_len,uint8* hum_len){
	uint8* packet;
	sendData_t* temp_start ;
	sendData_t* hum_start ;

	packetHead.vdd = EndReadVcc();

	// temp header
	*temp_len = tempQueue.count;
	if(*temp_len > TEMP_PACKET_SEND_SIZE){
		*temp_len = TEMP_PACKET_SEND_SIZE;
	}
	temp_start = &tempQueue.dat[tempQueue.front];
	packetHead.netAddr = NLME_GetShortAddr();
	osal_memcpy(packetHead.extAddr, extAddr, Z_EXTADDR_LEN);
	packetHead.tempStartTime = temp_start->utcSecs;
	packetHead.tempNumbers = *temp_len;
	packetHead.sampleFreq = sampleTempTimeDelay;
	printf("vcc:%d\n",packetHead.vdd);
	printf("temp packet header:\n");
	printf("sampleFreq:%d\n",packetHead.sampleFreq);
	printf("numbers:%d\n",packetHead.tempNumbers);
	
	// Humidity header
	*hum_len = humQueue.count;
	if(*hum_len > HUM_PACKET_SEND_SIZE){
		*hum_len = HUM_PACKET_SEND_SIZE;

	}
	hum_start = &humQueue.dat[humQueue.front];
	packetHead.humStartTime = hum_start->utcSecs;
	packetHead.humFreq = sampleHumTimeDelay;
	packetHead.humNumbers = *hum_len;
	packetHead.paramsVersion = paramsVersion;
	printf("vcc:%d\n",packetHead.vdd);
	printf("temp packet header:\n");
	printf("sampleFreq:%d\n",packetHead.sampleFreq);
	printf("paramsVersion:%d\n",packetHead.paramsVersion);
	printf("numbers:%d\n",packetHead.tempNumbers);

	
	//复制数据到发送数据包
	*total_len = sizeof(tempPacket_t) + sizeof(int16) * (*temp_len) + sizeof(16) * (*hum_len) + 1;
	packet = osal_mem_alloc(* total_len);
	uint8* _packet;
	_packet = packet;
	if(packet){
		*_packet ++ = TEMP_PACKET_SEND_CMD;
		osal_memcpy(_packet,&packetHead,sizeof(tempPacket_t));
		_packet += sizeof(tempPacket_t);
		int i;
		for (i = 0; i < *temp_len; i++){
			*_packet ++ = LO_UINT16(temp_start->data);
			*_packet ++ = HI_UINT16(temp_start->data);
			if(QueueOut(&tempQueue, temp_start) == QueueEmpty){
				printf("tempQueue empty\n");
				return NULL;
			}
		}
		
		for (i = 0; i < *hum_len; i++){
			*_packet ++ = LO_UINT16(hum_start->data);
			*_packet ++ = HI_UINT16(hum_start->data);
			
			if(QueueOut(&humQueue, hum_start) == QueueEmpty){
				printf("humQueue empty\n");
				return NULL;
			}
		}
		return packet;
	}else{
		return NULL;
	}
	
}




