#include "config.h"
#include "hardware/pio.h"
#include "FreeRTOS.h"
#include "task.h"

#ifndef _WS2812_H_
#define _WS2812_H_

typedef enum _RGB_LED_STATE
{
	RGB_LED_STATE_STARTUP,
	RGB_LED_STATE_JTAG,
	RGB_LED_STATE_PROG_B1_R1,
	RGB_LED_STATE_PROG_B0_R1,
	RGB_LED_STATE_PROG_B1_R0,
	RGB_LED_STATE_PROG_B0_R0,
	RGB_LED_STATE_MSC_START,
	RGB_LED_STATE_END,
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

#if (defined(WS2812_PIN))

extern TaskHandle_t g_ws2812_task_handle;
extern RGB_LED_STATE g_last_led_val;

void ws2812_pio_init(PIO pio);
void ws2812_start_task(void);

void __always_inline ws2812_set_rgb_state(RGB_LED_STATE state)
{
	if (g_ws2812_task_handle && g_last_led_val != state) 
		xTaskNotify(g_ws2812_task_handle, state, eSetValueWithOverwrite);
}
void __always_inline ws2812_set_rgb_state_isr(RGB_LED_STATE state)
{
	if (g_ws2812_task_handle && g_last_led_val != state) 
	{
		BaseType_t higherPriorityTaskWoken;
		xTaskNotifyFromISR(g_ws2812_task_handle, state, eSetValueWithOverwrite, &higherPriorityTaskWoken);
		portYIELD_FROM_ISR(higherPriorityTaskWoken);
	}
}
#else
#define ws2812_pio_init(pio)
#define ws2812_start_task()
#define ws2812_set_rgb_state(state)
#define ws2812_set_rgb_state_isr(state)
#endif
#endif
