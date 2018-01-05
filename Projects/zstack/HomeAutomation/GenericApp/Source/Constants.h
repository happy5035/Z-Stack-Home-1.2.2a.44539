#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "ZComDef.h"
#include "MT_RPC.h"
#include "MT.h"
#include "OSAL_Clock.h"


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
#define END_STATUS_CLUSTERID		  0xF7



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


//采集任务
#define SAMPLE_TASK_WAITE_TIMEOUT     100  		// 等待测量时间ms
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
// 命令格式为
//		| SOP | Data Length | CMD1 | CMD2 |  Data  | FCS |
//		|  1  |      1      |  1   |  1   |  0-Len |  1  |

#define UART_CMD_LEN				  2 		//uart 命令字节数
#define UART_LEN_TOKEN_LEN	          1			// 长度字节数	
#define UART_HEADER_LEN				  3			//一个长度字节，两个命令字节
#define UART_MIN_LEN				  5 		// 最小数据长度 
#define MT_RSP_CMD_APP				  ((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_APP)


//参数nv存储位置
#define NV_PARAM_VERSION				0x0401
#define NV_TEMP_SAMPLE_TIME				0x0402
#define NV_HUM_SAMPLE_TIME				0x0403
#define NV_PACKET_SEND_TIME				0x0404
#define NV_SYNC_CLOCK_TIME				0x0405

#define END_REPORT_RE_SEND_TIMES		3

//参数同步标志
#define PARAMS_FLAGS_CLOCK				0x0001
#define PARAMS_FLAGS_TEMP_TIME			0x0002
#define PARAMS_FLAGS_HUM_TIME			0x0004
#define PARAMS_FLAGS_PACKET_TIME		0x0008
#define PARAMS_FLAGS_SYNC_CLOCK_TIME	0x0010


typedef struct{
	uint16  netAddr;
	uint8 	extAddr[Z_EXTADDR_LEN];
	uint16 	vdd;
	UTCTime	clock;
	uint32	tempTime;
	uint32	humTime;
	uint32	packetTime;
	uint32	syncClockTime; 
}endStatus_t;
#endif
