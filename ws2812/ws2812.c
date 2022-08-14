/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stdio.h>
#include <stdlib.h>

#include "ws2812.h"
#include "config.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#if (defined(WS2812_PIN))
#define IS_RGBW true

static LED_COLOR_T _ledStateStartup[] = {
	{ 100, 0, 0, 10 },		// RED
	{ 0, 0, 0, 10 },		// BLACK
	{ 0, 0, 0, 0 }
};
static LED_COLOR_T _ledStateJTAG[] = {
	{ 0, 100, 0, 2 },		// GREEN
	{ 0, 0, 125, 2 },		// BLUE
	{ 0, 0, 0, 0 }
};

static LED_COLOR_T _ledStatePROG_B1R1[] = {
	{ 0, 100, 0, 10 },		// GREEN
	{ 0, 0, 0, 0 }
};

static LED_COLOR_T _ledStatePROG_B1R0[] = {
	{ 100, 0, 0, 10 },		// RED
	{ 0, 0, 0, 0 }
};

static LED_COLOR_T _ledStatePROG_B0R1[] = {
	{ 0, 0, 125, 10 },		// BLUE
	{ 0, 0, 0, 0 }
};

static LED_COLOR_T _ledStatePROG_B0R0[] = {
	{ 100, 50, 0, 10 },	// ORANGE
	{ 0, 0, 0, 0 }
};

static LED_COLOR_T _ledStateMSC_Start[] = {
	{ 75, 0, 100, 10 },	// PURPLE
	{ 0, 0, 0, 0 }
};

static LED_COLOR_T _ledStateMSC_End[] = {
	{ 0, 100, 0, 10 },		// GREEN
	{ 0, 0, 0, 0 }
};

TaskHandle_t g_ws2812_task_handle = NULL;


static PIO _pio_ws2812 = NULL;
static int _sm_ws2812 = 0;

static inline void put_pixel(uint32_t pixel_grb)
{
	pio_sm_put_blocking(_pio_ws2812, _sm_ws2812, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) 
{
	return ((uint32_t) (r) << 8) | ((uint32_t) (g) << 16) | (uint32_t) (b);
}

void ws2812_pio_init(PIO pio)
{
	_pio_ws2812 = pio;
	_sm_ws2812 = pio_claim_unused_sm(pio, true);
	uint offset = pio_add_program(pio, &ws2812_program);

	ws2812_program_init(_pio_ws2812, _sm_ws2812, offset, WS2812_PIN, 800000, IS_RGBW);

}

static void ws2812_put_pixel(uint8_t r, uint8_t g, uint8_t b)
{
	put_pixel(urgb_u32(r, g, b));
}

static void ws2812_task(void * pvParameters)
{
	uint32_t notificationValue;
	TickType_t ticksToWait;
	LED_COLOR_T* nextLedColor = _ledStateStartup;
	LED_COLOR_T* headLedColor = _ledStateStartup;
	TickType_t nextLedServiceTickTime;
	RGB_LED_STATE currentState = RGB_LED_STATE_STARTUP;
	RGB_LED_STATE lastNonPersistentState = RGB_LED_STATE_STARTUP;
	
	nextLedServiceTickTime =  xTaskGetTickCount() + pdMS_TO_TICKS(100);
	
	for (;;)
	{
		TickType_t tickTime = xTaskGetTickCount();
		TickType_t nextServiceTime = nextLedServiceTickTime;
		
		if (tickTime >= nextServiceTime)
		{
			ticksToWait = 1;
		}
		else
		{
			ticksToWait = nextServiceTime - tickTime;
		}
		BaseType_t change_state = xTaskNotifyWait(0, 0xFFFFFFFF, &notificationValue, ticksToWait);
		tickTime = xTaskGetTickCount();
		if (change_state == pdTRUE)
		{
			bool allowColorChange = true;
			if (notificationValue & (RGB_LED_STATE_COLOR_MASK | RGB_LED_STATE_PERSISTENT_MASK))
			{
				
				RGB_LED_STATE newState = notificationValue & (RGB_LED_STATE_COLOR_MASK | RGB_LED_STATE_PERSISTENT_MASK);
				if ((currentState & RGB_LED_STATE_SET_PERSISTENT) && !(newState & RGB_LED_STATE_PERSISTENT_MASK))
				{
					// previous state is persistent, new state does not have a persistence mask so just store color until persistence is removed
					lastNonPersistentState = newState;
					allowColorChange = false;
				}
				else if ((currentState & RGB_LED_STATE_SET_PERSISTENT) && (newState & RGB_LED_STATE_CLR_PERSISTENT))
				{
					// persistent state removed, revert to last non persistent color
					newState = lastNonPersistentState;
				}
				else if (newState & RGB_LED_STATE_SET_PERSISTENT)
				{
					// set a persistent state. store the current state so we revert back to it when this persistent state is cleared
					lastNonPersistentState = currentState;
				}
				if (allowColorChange)
				{
					nextLedServiceTickTime = tickTime;
					switch (newState & RGB_LED_STATE_COLOR_MASK)
					{
					case RGB_LED_STATE_STARTUP:
						headLedColor = _ledStateStartup;
						nextLedColor = headLedColor;
						break;
					case RGB_LED_STATE_JTAG:
						headLedColor = _ledStateJTAG;
						nextLedColor = headLedColor;
						break;
					case RGB_LED_STATE_PROG_B1_R1:
						headLedColor = _ledStatePROG_B1R1;
						nextLedColor = headLedColor;
						break;
					case RGB_LED_STATE_PROG_B0_R1:
						headLedColor = _ledStatePROG_B0R1;
						nextLedColor = headLedColor;
						break;
					case RGB_LED_STATE_PROG_B1_R0:
						headLedColor = _ledStatePROG_B1R0;
						nextLedColor = headLedColor;
						break;
					case RGB_LED_STATE_PROG_B0_R0:
						headLedColor = _ledStatePROG_B0R0;
						nextLedColor = headLedColor;
						break;
					case RGB_LED_STATE_MSC_START:
						headLedColor = _ledStateMSC_Start;
						nextLedColor = headLedColor;
						break;
					case RGB_LED_STATE_MSC_END:
						headLedColor = _ledStateMSC_End;
						nextLedColor = headLedColor;
						break;
					default :
						break;
					}
					
					currentState = newState;
				}
			}
		}
		
		// Service the RGB led
		if (tickTime >= nextLedServiceTickTime)
		{
			if (nextLedColor->Delay100ms == 0)
				nextLedColor = headLedColor;
			
			nextLedServiceTickTime = tickTime + pdMS_TO_TICKS((uint32_t)nextLedColor->Delay100ms * 100);
			ws2812_put_pixel(nextLedColor->r, nextLedColor->g, nextLedColor->b);
			nextLedColor++;

		}
	}
}

void ws2812_start_task(void)
{
	xTaskCreateAffinitySet(ws2812_task, "ws2812_task", STACK_SIZE_FROM_BYTES(2 * 1024), NULL, tskIDLE_PRIORITY + 1, CORE_AFFINITY_WS2812_TASK, &g_ws2812_task_handle);
}

#endif
