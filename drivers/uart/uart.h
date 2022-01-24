#ifndef __UART_H
#define __UART_H

#include <stdbool.h>
#include <stdint.h>

void uart0_init(void);
bool set_uart0_bps(uint32_t bps);
bool is_uart0_got_data(void);


#endif /* __UART_H */
