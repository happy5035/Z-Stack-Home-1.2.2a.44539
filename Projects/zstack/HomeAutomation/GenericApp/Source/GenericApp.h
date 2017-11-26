/**************************************************************************************************
  Filename:       GenericApp.h
  Revised:        $Date: 2012-02-12 16:04:42 -0800 (Sun, 12 Feb 2012) $
  Revision:       $Revision: 29217 $

  Description:    This file contains the Generic Application definitions.


  Copyright 2004-2012 Texas Instruments Incorporated. All rights reserved.

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
**************************************************************************************************/

#ifndef GENERICAPP_H
#define GENERICAPP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "MT_RPC.h"
#include "MT.h"

/*********************************************************************
 * CONSTANTS
 */

// These constants are only for example and should be changed to the
// device's needs
#define GENERICAPP_ENDPOINT           10

#define GENERICAPP_PROFID             0x0F04
#define GENERICAPP_DEVICEID           0x0001
#define GENERICAPP_DEVICE_VERSION     0
#define GENERICAPP_FLAGS              0

#define GENERICAPP_MAX_CLUSTERS       1
#define GENERICAPP_CLUSTERID          0xf1
#define SYNC_TIME_CLUSTERID			  0xf2
#define REQUEST_SYNC_CLOCK_CLUSTERID  0xf3
#define TEMP_HUM_DATA_CLUSTERID  	  0xf4




#define TEMP_PACKET_SEND_SIZE     		30
#define HUM_PACKET_SEND_SIZE     		30



// Send Message Timeout
#define GENERICAPP_SEND_MSG_TIMEOUT   5000     // Every 5 seconds
// Application Events (OSAL) - These are bit weighted definitions.
#define GENERICAPP_SEND_MSG_EVT       0x0001
#define UART_RX_CB_EVT                0x0002 	
#define SAMPLE_TEMP_EVT               0x0004
#define SAMPLE_HUM_EVT				  0x0008
#define SAMPLE_TASK_EVT	  	  		  0x0010
#define TEMP_PACKET_SEND_EVT		  0x0020
#define REQUEST_SYNC_CLOCK_EVT		  0x0040

//�ɼ�����
#define SAMPLE_TASK_WAITE_TIMEOUT     100  		// �ȴ�����ʱ��ms
#define SAMPLE_TEMP_START_TASK 		  0x01
#define SAMPLE_TEMP_READY_TASK 		  0x02
#define SAMPLE_HUM_START_TASK 		  0x04
#define SAMPLE_HUM_READY_TASK 		  0x08

// aps cmd
#define TEMP_PACKET_SEND_CMD   	      0x00
#define REQUEST_SYNC_CLOCK_CMD		  0x01
#define SYNC_TIME_CMD				  0x02
#define COOR_START_CMD				  0x03

//uart constans
// �����ʽΪ
//		| SOP | Data Length | CMD1 | CMD2 |  Data  | FCS |
//		|  1  |      1      |  1   |  1   |  0-Len |  1  |

#define UART_CMD_LEN				  2 		//uart �����ֽ���
#define UART_LEN_TOKEN_LEN	          1			// �����ֽ���	
#define UART_HEADER_LEN				  3			//һ�������ֽڣ����������ֽ�
#define UART_MIN_LEN				  5 		// ��С���ݳ��� 
#define MT_RSP_CMD_APP				  ((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_APP)



#if defined( IAR_ARMCM3_LM )
#define GENERICAPP_RTOS_MSG_EVT       0x0002
#endif  

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the Generic Application
 */
extern void GenericApp_Init( byte task_id );

/*
 * Task Event Processor for the Generic Application
 */
extern UINT16 GenericApp_ProcessEvent( byte task_id, UINT16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GENERICAPP_H */
