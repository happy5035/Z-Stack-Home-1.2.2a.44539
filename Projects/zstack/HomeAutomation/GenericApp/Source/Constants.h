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

#define GENERICAPP_MAX_CLUSTERS       	1
#define GENERICAPP_CLUSTERID          	0xf1
#define SYNC_TIME_CLUSTERID			  	0xf2
#define REQUEST_SYNC_CLOCK_CLUSTERID  	0xf3
#define TEMP_HUM_DATA_CLUSTERID  	  	0xf4
#define SYNC_FREQ_CLUSTERID	  	      	0xF5
#define SYNC_PARAM_CLUSTERID		  	0xF6
#define END_STATUS_CLUSTERID		  	0xF7
#define END_SYNC_PARAMS_CLUSTERID	  	0xF8
#define SYNC_NV_CONFIG_CLUSTERID	  	0xF9
#define SYNC_NV_CONFIG_RESULT_CLUSTERID	0xFA
#define REMOTE_UART_DATA_CLUSTERID		0xFB
#define REMOTE_UART_RESPONCE_CLUSTERID	0xFC



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
#define END_REPORT_CONFIRM_TIMEOUT_EVT 0x0080
#define END_SYNC_PARAMS_TIEMOUT_EVT	  0x0100
#define END_RESET_TIEMOUT_EVT		  0x0200
#define END_START_PROCESS_EVT		  0x0400
#define END_DATA_REQUEST_EVT		  0x0800

#define SAMPLE_TEMP_TIME_DELAY_DEFAULT			30000
#define TEMP_PACKET_SEND_TIME_DELAY_DEFAULT		300000
#define SAMPLE_HUM_TIME_DELAY_DEFAULT			30000
#define REQUEST_SYNC_CLOCK_DELAY_DEFAULT		1800000
#define PACKET_TIME_WINDOW_DEFAULT				1
#define DATA_REQUEST_TIMEOUT_DEFAULT			10000

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
#define END_REPORT_STATUS_CMD		  0xF6

//uart constans
// �����ʽΪ
//		| SOP | Data Length | CMD1 | CMD2 |  Data  | FCS |
//		|  1  |      1      |  1   |  1   |  0-Len |  1  |

#define UART_CMD_LEN				  2 		//uart �����ֽ���
#define UART_LEN_TOKEN_LEN	          1			// �����ֽ���	
#define UART_HEADER_LEN				  3			//һ�������ֽڣ����������ֽ�
#define UART_MIN_LEN				  5 		// ��С���ݳ��� 
#define MT_RSP_CMD_APP				  ((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_APP)


//����nv�洢λ��
#define NV_PARAM_VERSION				0x0401
#define NV_TEMP_SAMPLE_TIME				0x0402
#define NV_HUM_SAMPLE_TIME				0x0403
#define NV_PACKET_SEND_TIME				0x0404
#define NV_SYNC_CLOCK_TIME				0x0405
#define NV_PACKET_TIME_WINDOW			0x0406

#define END_REPORT_RE_SEND_TIMES		3
#define END_REPORT_COMFIRM_TIMEOUT		3000

//����ͬ����־
#define PARAMS_FLAGS_CLOCK				0x0001
#define PARAMS_FLAGS_TEMP_TIME			0x0002
#define PARAMS_FLAGS_HUM_TIME			0x0004
#define PARAMS_FLAGS_PACKET_TIME		0x0008
#define PARAMS_FLAGS_SYNC_CLOCK_TIME	0x0010
#define PARAMS_FLAGS_PACKET_TIME_WINDOW	0x0020

//ͬ������״̬��־
#define SYNC_PARAMS_STATUS_INIT			0x0001
#define SYNC_PARAMS_STATUS_SEND			0x0002
#define SYNC_PARAMS_STATUS_TIMEOUT		0x0003
#define SYNC_PARAMS_STATUS_RECEIVE		0x0004
#define SYNC_PARAMS_STATUS_END			0x0005

#define SYNC_PARAMS_STATUS_TIMES		3
#define SYNC_PARAMS_CONFIRM_TIMEOUT		3000
#endif