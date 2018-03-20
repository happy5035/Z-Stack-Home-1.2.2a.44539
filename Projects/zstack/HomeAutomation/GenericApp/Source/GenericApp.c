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
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#include "OSAL_PwrMgr.h"
#include "OSAL_Nv.h"
#include "ZDSecMgr.h"

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
uint8 tempPacketSendRetrayTimes = 0;			//�¶����ݰ��ظ����ʹ���
uint8 tempPacketSendPacketTransID;
uint32 syncTimeDealy = (uint32)1000*60;
uint16 tempPacketTimeWindow = 1;

uint32 sampleHumTimeDelay = 10000;// 60sһ�βɼ�ʪ������

uint8 sampleTask = 0x00;

uint32 requestSyncClockDelay = 600000; //10����ͬ��һ��ʱ��

uint8 paramsVersion = 1;

uint8 extAddr[Z_EXTADDR_LEN];


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void GenericApp_HandleKeys( byte shift, byte keys );
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
static void ReadNvParams(void);




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
  // If 11the hardware is other parts of the device add it in main().

  GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  GenericApp_DstAddr.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_DstAddr.addr.shortAddr = 0x00;
	//�㲥��ַ
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
	//��ʼ��ģ��extAddr
	ZMacGetReq( ZMacExtAddr, extAddr );

	ZDO_RegisterForZDOMsg(GenericApp_TaskID, Device_annce);
//    EndTempSampleCfg();

	MT_UartRegistGenericAppTaskId(GenericApp_TaskID);
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
				ReadNvParams();
				CoorSendCoorStart();
//				osal_setClock(0x21AAEBCB);  //2017/11/24 16:40:00

			}
			if(GenericApp_NwkState == DEV_ROUTER){
				printf("router start...\n");
				HalLedSet(HAL_LED_2, HAL_LED_MODE_ON);
			}
			if(GenericApp_NwkState == DEV_END_DEVICE){


			}

          }
          break;
		case MT_SYS_APP_MSG:
			CoorProcessMtSysMsg((mtSysAppMsg_t *)MSGpkt);
			break;
		case CMD_SERIAL_MSG:
			CoorMTUartSerialMsgProcess((mtOSALSerialData_t *)MSGpkt);
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


	if(events & COOR_TEST_TIMEOUT_EVT){

		return events ^ COOR_TEST_TIMEOUT_EVT;
	}


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

	case Device_annce:
		{
		ZDO_DeviceAnnce_t devAnnc;
		ZDO_ParseDeviceAnnce(inMsg, &devAnnc);
		if(0 == (devAnnc.capabilities & 0x0F)){
			ZDSecMgrAddrClear(devAnnc.extAddr);
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
	case END_STATUS_CLUSTERID:
		CoorProcessEndStatus(pkt);
		break;
	case REMOTE_MT_UART_RESPONSE_CLUSTERID:
		CoorProcessUartResponse(pkt);
		break;
	case ROUTER_STATUS_CLUSTERID:
		CoorProcessRouterStatus(pkt);
		break;
	case SEND_APP_MSG_CLUSTERID:
		CoorSendAppMsg(pkt);
		break;
	default :
		break;
  }
}

static void ReadNvParams(){
	uint8 result;
	uint8* buf;
	buf = osal_mem_alloc(4);

	result = osal_nv_read(NV_PARAM_VERSION, 0, 1, buf);
	if(result == NV_OPER_FAILED){
		*buf = paramsVersion;
		osal_nv_item_init(NV_PARAM_VERSION, 1, buf);
	}else{
		paramsVersion = *buf;
	}
	osal_nv_write(NV_PARAM_VERSION, 0, 1, &paramsVersion);
	result = osal_nv_read(NV_PARAM_FLAGS, 0, 4, buf);
	if(result == NV_OPER_FAILED){
		osal_buffer_uint32(buf, 0);
		osal_nv_item_init(NV_PARAM_FLAGS, 4, buf);
	}

	result = osal_nv_read(NV_TEMP_SAMPLE_TIME, 0, 4, buf);
	if(result == NV_OPER_FAILED){
		osal_buffer_uint32(buf, sampleTempTimeDelay);
		osal_nv_item_init(NV_TEMP_SAMPLE_TIME, 4, buf);
	}else{
		sampleTempTimeDelay = osal_build_uint32(buf, 4);
	}

	result = osal_nv_read(NV_HUM_SAMPLE_TIME, 0, 4, buf);
	if(result == NV_OPER_FAILED){
		osal_buffer_uint32(buf, sampleHumTimeDelay);
		osal_nv_item_init(NV_HUM_SAMPLE_TIME, 4, buf);
	}else{
		sampleHumTimeDelay = osal_build_uint32(buf, 4);
	}

	result = osal_nv_read(NV_PACKET_SEND_TIME, 0, 4, buf);
	if(result == NV_OPER_FAILED){
		osal_buffer_uint32(buf, tempPacketSendTimeDelay);
		osal_nv_item_init(NV_PACKET_SEND_TIME, 4, buf);
	}else{
		tempPacketSendTimeDelay = osal_build_uint32(buf, 4);
	}

	result = osal_nv_read(NV_SYNC_CLOCK_TIME, 0, 4, buf);
	if(result == NV_OPER_FAILED){
		osal_buffer_uint32(buf, requestSyncClockDelay);
		osal_nv_item_init(NV_SYNC_CLOCK_TIME, 4, buf);
	}else{
		requestSyncClockDelay = osal_build_uint32(buf, 4);
	}

	result = osal_nv_read(NV_REMOTE_URART_DEST_ADDR, 0, 2, buf);
	if(result == NV_OPER_FAILED){
		osal_buffer_uint32(buf, 0);
		osal_nv_item_init(NV_REMOTE_URART_DEST_ADDR, 2, buf);
	}
	result = osal_nv_read(NV_PACKET_TIME_WINDOW, 0, 2, buf);
	if(result == NV_OPER_FAILED){
		osal_memset(buf, 0, 4);
		osal_nv_item_init(NV_PACKET_TIME_WINDOW, 2,  buf);
	}
	result = osal_nv_read(NV_PACKET_TIME_WINDOW_INTERVAL, 0, 2, buf);
	if(result == NV_OPER_FAILED){
		uint16 packetTimeWindowInterval = PACKET_TIME_WINDOW_INTERVAL_DEFAULT;
		osal_nv_item_init(NV_PACKET_TIME_WINDOW_INTERVAL, 2,  &packetTimeWindowInterval);
	}
	osal_mem_free(buf);
}
