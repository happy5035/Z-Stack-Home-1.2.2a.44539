#include "MY_Uart.h"
#include "ZComDef.h"
#include "hal_led.h"
#include "GenericApp.h"

extern uint8 GenericApp_TaskID;
uint8  rxlen;     
uint8* rbuf;   
uint8 buflen;
void MY_UartInit ()
{
  halUARTCfg_t uartConfig;

  /* Initialize APP ID */
//  App_TaskID = 0;

  /* UART Configuration */
  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = MT_UART_DEFAULT_BAUDRATE; 
  uartConfig.flowControl          = MT_UART_DEFAULT_OVERFLOW;
  uartConfig.flowControlThreshold = MT_UART_DEFAULT_THRESHOLD;
  uartConfig.rx.maxBufSize        = MT_UART_DEFAULT_MAX_RX_BUFF;
  uartConfig.tx.maxBufSize        = MT_UART_DEFAULT_MAX_TX_BUFF;
  uartConfig.idleTimeout          = MT_UART_DEFAULT_IDLE_TIMEOUT;
  uartConfig.intEnable            = TRUE;
  uartConfig.callBackFunc         = MY_UartCallBack;

  /* Start UART */
  HalUARTOpen (MT_UART_DEFAULT_PORT, &uartConfig);
  /* Silence IAR compiler warning */
  (void)uartConfig;
  
}
/*
*   ´®¿Ú»Øµôº¯Êý
*/

void MY_UartCallBack( uint8 port, uint8 event )
{
  rxlen=Hal_UART_RxBufLen(port);  
  rbuf=osal_mem_alloc(rxlen+1);  
  rbuf[0]=rxlen;
  HalUARTRead ( port, rbuf+1, rxlen); 
  buflen=rxlen+1;
  if(rxlen==0)
     osal_mem_free( rbuf );  
  else
  osal_set_event(GenericApp_TaskID,UART_RX_CB_EVT);
}
