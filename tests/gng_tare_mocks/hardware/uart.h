#ifndef GNG_TARE_TEST_UART_H
#define GNG_TARE_TEST_UART_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
typedef void *uart_inst_t;
#define uart0 ((uart_inst_t)0)
bool uart_is_readable(uart_inst_t uart);
char uart_getc(uart_inst_t uart);
void uart_tx_wait_blocking(uart_inst_t uart);
uint32_t uart_init(uart_inst_t uart, uint32_t baudrate);
uint32_t uart_set_baudrate(uart_inst_t uart, uint32_t baudrate);
void uart_set_format(uart_inst_t uart, unsigned data_bits, unsigned stop_bits, int parity);
void uart_write_blocking(uart_inst_t uart, const uint8_t *data, size_t length);
#endif
