#ifndef UART_XMOS_H
#define UART_XMOS_H

#include <stdint.h>

void UART_Init(void);
uint8_t UART_ReadByte(uint8_t *data);
uint8_t UART_XMOS_GetSignal(void);


extern volatile uint8_t last_signal;
extern volatile uint8_t new_signal_received;


#endif