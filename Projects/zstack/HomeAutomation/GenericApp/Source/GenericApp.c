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
#include "FIFOQueue.h"
#include "OSAL_PwrMgr.h"

#include "GenericApp.h"
#include "DebugTrace.h"

#if !defined( WIN32 ) || defined( ZBIT )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"

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

// Number of recieved messages
static uint16 rxMsgCount;

// Time interval between sending messages
static uint32   txMsgDelay = GENERICAPP_SEND_MSG_TIMEOUT;
static uint16   voltageAtTemp19 = 0;
static bool     bCalibrate = TRUE;
uint32  sampleTempTimeDelay = 5000;				//1s
uint32 tempPacketSendTimeDelay = 10000;			//30s
uint8 tempPacketSendRetrayTimes = 0;			//�¶����ݰ��ظ����ʹ���
uint8 tempPacketSendPacketTransID;
uint32 syncTimeDealy = (uint32)1000*60;

//�洢�¶�����
FifoQueue tempQueue;
uint8 extAddr[Z_EXTADDR_LEN];

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void GenericApp_HandleKeys( byte shift, byte keys );
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
static void SampleTimeHandler(void);
static void TempPacketSendHandler(void);
static void TempSampleCfg(void);
static uint16 readTemp(void);


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

//    TempSampleCfg();
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
          printf("%d data confirm,status: 0x%x\n", sentTransID,sentStatus);
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
			}
			if(GenericApp_NwkState == DEV_ROUTER){
				printf("router start...\n");
				HalLedSet(HAL_LED_2, HAL_LED_MODE_ON);
			}
			if(GenericApp_NwkState == DEV_END_DEVICE){
				printf("end device start...\n");
				HalLedSet(HAL_LED_3, HAL_LED_MODE_ON);
				//���������ͷ��Ͷ�ʱ��
				osal_start_reload_timer(GenericApp_TaskID, SAMPLE_TEMP_EVT, sampleTempTimeDelay);
				osal_start_reload_timer(GenericApp_TaskID, TEMP_PACKET_SEND_EVT, tempPacketSendTimeDelay);
                
				TempSampleCfg();
                //�¶����ݶ��г�ʼ��
                QueueInit(&tempQueue);
				osal_pwrmgr_device(PWRMGR_BATTERY);
			}
            
          }
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
		SampleTimeHandler();
		return events ^ SAMPLE_TEMP_EVT;
	}
	if(events & TEMP_PACKET_SEND_EVT){
		TempPacketSendHandler();
		return events ^ TEMP_PACKET_SEND_EVT;
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
  }
}

/*********************************************************************
 * @fn      GenericApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void GenericApp_HandleKeys( uint8 shift, uint8 keys )
{
  zAddrType_t dstAddr;

  // Shift is used to make each button/switch dual purpose.
  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
#if defined( SWITCH1_BIND )
      // We can use SW1 to simulate SW2 for devices that only have one switch,
      keys |= HAL_KEY_SW_2;
#elif defined( SWITCH1_MATCH )
      // or use SW1 to simulate SW4 for devices that only have one switch
      keys |= HAL_KEY_SW_4;
#else
      // Normally, SW1 changes the rate that messages are sent
      if ( txMsgDelay > 100 )
      {
        // Cut the message TX delay in half
        txMsgDelay /= 2;
      }
      else
      {
        // Reset to the default
        txMsgDelay = GENERICAPP_SEND_MSG_TIMEOUT;
      }
#endif
    }

    if ( keys & HAL_KEY_SW_2 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

      // Initiate an End Device Bind Request for the mandatory endpoint
      dstAddr.addrMode = Addr16Bit;
      dstAddr.addr.shortAddr = 0x0000; // Coordinator
      ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
                            GenericApp_epDesc.endPoint,
                            GENERICAPP_PROFID,
                            GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                            GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                            FALSE );
    }

    if ( keys & HAL_KEY_SW_3 )
    {
    }

    if ( keys & HAL_KEY_SW_4 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
      // Initiate a Match Description Request (Service Discovery)
      dstAddr.addrMode = AddrBroadcast;
      dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
      ZDP_MatchDescReq( &dstAddr, NWK_BROADCAST_SHORTADDR,
                        GENERICAPP_PROFID,
                        GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                        GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
                        FALSE );
    }
  }
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
      rxMsgCount += 1;  // Count this message
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_BLINK );  // Blink an LED
#if defined( LCD_SUPPORTED )
      HalLcdWriteString( (char*)pkt->cmd.Data, HAL_LCD_LINE_1 );
      HalLcdWriteStringValue( "Rcvd:", rxMsgCount, 10, HAL_LCD_LINE_2 );
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
  }
}


static void SampleTimeHandler(void){

	uint16 temp;
	sendData_t dataPacket;
	temp = readTemp();
	dataPacket.utcSecs = osal_getClock();
	dataPacket.temp = temp;
	uint8 res;
	res = QueueIn(&tempQueue,&dataPacket);
	if(res == QueueFull){
		printf("temp queue full");
//		osal_stop_timerEx(uint8 task_id, uint16 event_id)
	}
	printf("%2d.%02d\n",temp/100,temp%100);
	
}
static void TempPacketSendHandler(void){
	uint8 len;
	len = tempQueue.count;
	if(len > TEMP_PACKET_SEND_SIZE){
		len = TEMP_PACKET_SEND_SIZE;
	}
	sendData_t* start ;
	uint8* packet;
	start = &tempQueue.dat[tempQueue.front];
    packet = osal_mem_alloc(len*sizeof(sendData_t) + Z_EXTADDR_LEN );
	if(packet){
		osal_memcpy(packet,extAddr,Z_EXTADDR_LEN);
		osal_memcpy(packet+Z_EXTADDR_LEN,start,len*sizeof(sendData_t));
		if (AF_DataRequest(&GenericApp_DstAddr, &GenericApp_epDesc, 
			GENERICAPP_CLUSTERID, 
			len * sizeof(sendData_t) + Z_EXTADDR_LEN , //(byte)osal_strlen( theMessageData ) + 1,
		(byte *) packet, 
			&GenericApp_TransID, 
			AF_DISCV_ROUTE|AF_ACK_REQUEST, AF_DEFAULT_RADIUS) == afStatus_SUCCESS)
		{
			// Successfully requested to be sent.
			HalLedBlink(HAL_LED_4, 1, 50, 500);
			printf("send temp success\n");
			QueueRemove(&tempQueue,len);
		}else{
			printf("send temp failed\n");
	    }
        
	    osal_mem_free(packet);
    }else{
		printf("alloc packet failed\n");
	}
}

/**
*	�¶ȴ���������
*/
static void TempSampleCfg(void){
	TR0 				= 0X01; 					/*��������AD���¶ȴ���������*/
	ATEST				= 0X01; 					/*�����¶ȴ�����*/

}

static uint16 readTemp(){
    uint8 i;
	uint16 adcValue;
	uint16 value;
	adcValue = 0;
	
	for (i = 0; i < 4; i++){
		ADCCON3 |=0x0E;
		ADCCON3 &= 0x3f;
		ADCCON3 |=0x30;
		ADCCON1 |= 0x30;
		ADCCON1 |= 0x40;
		while(!(ADCCON1 & 0X80));
		value = ADCL>>2;
		value |=(((uint16)ADCH)<<6);
		adcValue +=value;
	}
	value = adcValue >>2;
	if(bCalibrate){
		voltageAtTemp19 = value;
		bCalibrate = FALSE;
	}
    float re;
    re = (value - voltageAtTemp19) / 4.0 + 10;
	return	(uint16)(re * 100); 

}


