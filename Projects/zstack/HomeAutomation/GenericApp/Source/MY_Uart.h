#ifndef MY_UART_H
#define MY_UART_H
#include "Onboard.h"

#define MT_UART_DEFAULT_BAUDRATE         HAL_UART_BR_38400
#define MT_UART_DEFAULT_THRESHOLD        MT_UART_THRESHOLD
#define MT_UART_DEFAULT_MAX_RX_BUFF      MT_UART_RX_BUFF_MAX
#define MT_UART_DEFAULT_MAX_TX_BUFF      MT_UART_TX_BUFF_MAX
#define MT_UART_DEFAULT_IDLE_TIMEOUT     MT_UART_IDLE_TIMEOUT
#define MT_UART_DEFAULT_OVERFLOW         FALSE
#define MT_UART_DEFAULT_PORT             HAL_UART_PORT_1

void MY_UartInit(void);
void MY_UartCallBack ( uint8 port, uint8 event );

#endif