#include "hardware/pio.h"

#ifndef _WS2812_H_
#define _WS2812_H_

void ws2812_pio_init(PIO pio);
void ws2812_put_pixel(uint8_t r, uint8_t g, uint8_t b);

#endif
