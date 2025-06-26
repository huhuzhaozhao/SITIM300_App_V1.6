
#ifndef __UART_X_H__
#define __UART_X_H__

#include "main.h"
#include "string.h"

typedef enum 
{
    WAIT_HEAD1,
    WAIT_HEAD2,
    WAIT_CMD,
    WAIT_DATA
} recv_state;

extern uint8_t uart_rx_byte;
extern uint32_t ul_CmdData;
#endif /* __UART_X_H__ */

