#include "config.h"
#include "hardware/pio.h"
#include "FreeRTOS.h"
#include "task.h"

#ifndef _WS2812_H_
#define _WS2812_H_

#ifndef WS2812_PIN
#ifdef PICO_DEFAULT_WS2812_PIN
#define WS2812_PIN PICO_DEFAULT_WS2812_PIN
#else
// default to pin 2 if the board doesn't have a default WS2812 pin defined
#define WS2812_PIN 16
#endif
#endif

extern TaskHandle_t g_ws2812_task_handle;
typedef enum _RGB_LED_STATE
{
	RGB_LED_STATE_STARTUP,
	
	RGB_LED_STATE_COLOR_MASK        = 0xFF,

	RGB_LED_STATE_SET_PERSISTENT    = 0x100,
	RGB_LED_STATE_CLR_PERSISTENT    = 0x200,
	RGB_LED_STATE_PERSISTENT_MASK   = (RGB_LED_STATE_SET_PERSISTENT | RGB_LED_STATE_CLR_PERSISTENT),
}RGB_LED_STATE;

typedef struct _LED_COLOR_T
{
	uint8_t r;
	uint8_t g; 
	uint8_t b;
	uint8_t Delay100ms;
}LED_COLOR_T;

void ws2812_pio_init(PIO pio);
void ws2812_start_task(void);

void __always_inline ws2812_set_rgb_state(RGB_LED_STATE state)
{
	if (g_ws2812_task_handle) 
		xTaskNotify(g_ws2812_task_handle, state, eSetValueWithOverwrite);
}

#endif
