#ifndef GNG_TARE_TEST_BOARD_H
#define GNG_TARE_TEST_BOARD_H
#include "hardware/uart.h"
#define SCALE_UART uart0
#define SCALE_UART_TX 0
#define SCALE_UART_RX 1
#define GPIO_FUNC_UART 2
#define UART_PARITY_NONE 0
void gpio_set_function(unsigned pin, unsigned function);
#endif
