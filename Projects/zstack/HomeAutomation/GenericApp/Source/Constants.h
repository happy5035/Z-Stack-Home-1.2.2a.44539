#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "ZComDef.h"
#include "MT_RPC.h"
#include "MT.h"


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
#define SYNC_FREQ_CLUSTERID	  	      0xF5
#define SYNC_PARAM_CLUSTERID		  0xF6



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
#define TEMP_PACKET_SEND_CMD   	      0xF0
#define REQUEST_SYNC_CLOCK_CMD		  0xF1
#define SYNC_TIME_CMD				  0xF2
#define COOR_START_CMD				  0xF3
#define MASTER_SET_CLOCK_CMD		  0xF4 
#define MASTER_SET_FREQ_CMD	          0xF5

//uart constans
// �����ʽΪ
//		| SOP | Data Length | CMD1 | CMD2 |  Data  | FCS |
//		|  1  |      1      |  1   |  1   |  0-Len |  1  |

#define UART_CMD_LEN				  2 		//uart �����ֽ���
#define UART_LEN_TOKEN_LEN	          1			// �����ֽ���	
#define UART_HEADER_LEN				  3			//һ�������ֽڣ����������ֽ�
#define UART_MIN_LEN				  5 		// ��С���ݳ��� 
#define MT_RSP_CMD_APP				  ((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_APP)




#endif