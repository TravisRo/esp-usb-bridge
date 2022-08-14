
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "bsp/board.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "hardware/vreg.h"
#include "pico.h"
#include "pico/stdlib.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#if ( configNUM_CORES > 1 )
#include "pico/multicore.h"
#endif

#include "ws2812.h"
#include "usb_descriptors.h"

#include "jtag.h"
#include "serial.h"
#include "msc.h"

#if (CFG_TUSB_OS != OPT_OS_FREERTOS)
#error "TinyUSB is not using freertos!"
#endif


static HeapStats_t heap_stats;
static size_t minEverBytesRemaining = 0xFFFFFFFF;

static void tusb_device_task(void *pvParameters)
{
	while (1)
	{
		vPortGetHeapStats(&heap_stats);
		if (heap_stats.xMinimumEverFreeBytesRemaining < minEverBytesRemaining)
		{
			minEverBytesRemaining = heap_stats.xMinimumEverFreeBytesRemaining;
			//printf("xMinimumEverFreeBytesRemaining:%u\r\n", heap_stats.xMinimumEverFreeBytesRemaining);
		}
		//ws2812_put_pixel(100, 0, 0);
		//tud_task();
		//ws2812_put_pixel(0, 100, 0);
		tud_task();
	}
	vTaskDelete(NULL);
}

int main(void)
{
	vreg_set_voltage(VREG_VOLTAGE_1_15);
	set_sys_clock_khz(260000, true);

	board_init();
	init_serial_no();

	// init device stack on configured roothub port
	tud_init(BOARD_TUD_RHPORT);

#if (defined(WS2812_PIN))
	ws2812_pio_init(pio0);
	ws2812_start_task();
#endif

	xTaskCreateAffinitySet(tusb_device_task, "tusb_device_task", STACK_SIZE_FROM_BYTES(4 * 1024), NULL, 5, CORE_AFFINITY_USB_TASK, NULL);
	xTaskCreateAffinitySet(msc_task, "msc_task", STACK_SIZE_FROM_BYTES(4 * 1024), NULL, 5, CORE_AFFINITY_MSC_TASK, NULL);
	xTaskCreateAffinitySet(start_serial_task, "start_serial_task", STACK_SIZE_FROM_BYTES(4 * 1024), NULL, 5, CORE_AFFINITY_SERIAL_TASK, NULL);
	xTaskCreateAffinitySet(jtag_task, "jtag_task", STACK_SIZE_FROM_BYTES(4 * 1024), NULL, 5, CORE_AFFINITY_JTAG_TASK, NULL);
	
	
	vTaskStartScheduler();

	for (;;);

	return 0;
}

