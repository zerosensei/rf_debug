#ifndef __UART_TASK_H
#define __UART_TASK_H

#include <stdint.h>

#define UART_TIMED_EVT          (1<<0)
#define UART_START_EVT          (1<<1)

extern uint8_t uart_task_id;


void uart_task_init(void);

#endif /* __UART_TASK_H */
