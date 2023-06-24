#include "ubp_config.h"
#include "hardware/pio.h"

#ifndef _PIO_UART_LOGGER
#define _PIO_UART_LOGGER

void start_pio_uart_logger(PIO pio, uint32_t pin, uint32_t bit_rate);

#endif
