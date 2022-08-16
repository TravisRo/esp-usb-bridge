
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
#include "pico/stdio.h"

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

#include "pio_uart_logger/pio_uart_logger.h"

#if (CFG_TUSB_OS != OPT_OS_FREERTOS)
#error "TinyUSB is not using freertos!"
#endif

static void tusb_device_task(void *pvParameters)
{
	while (1)
	{
		tud_task();
	}
	vTaskDelete(NULL);
}

int main(void)
{
	
#if RP2040_OVERCLOCK_ENABLED
	vreg_set_voltage(VREG_VOLTAGE_1_15);
	set_sys_clock_khz(260000, true);
#endif
	
	/* board_init is a tinyusb function which mainly just enables UART0 for
	 * stdio. We are going to use the PIO logger so UART0 can be used for
	 * the serial programming interface for RP2040 boards that dont provide
	 * access to UART1 (Seeed XIAO RP2040 for example)
	board_init();
	*/
	init_serial_no();

#if (LOGGING_ENABLED())
	start_pio_uart_logger(pio0, LOGGER_UART_TX_PIN, LOGGER_UART_BITRATE);
#endif
	
	// init device stack on configured roothub port
	tud_init(BOARD_TUD_RHPORT);

	ws2812_pio_init(pio0);
	ws2812_start_task();

	xTaskCreateAffinitySet(tusb_device_task, "tusb_device_task", STACK_SIZE_FROM_BYTES(4 * 1024), NULL, 5, CORE_AFFINITY_USB_TASK, NULL);
	xTaskCreateAffinitySet(msc_task, "msc_task", STACK_SIZE_FROM_BYTES(4 * 1024), NULL, 5, CORE_AFFINITY_MSC_TASK, NULL);
	xTaskCreateAffinitySet(start_serial_task, "start_serial_task", STACK_SIZE_FROM_BYTES(4 * 1024), NULL, 5, CORE_AFFINITY_SERIAL_TASK, NULL);
	xTaskCreateAffinitySet(jtag_task, "jtag_task", STACK_SIZE_FROM_BYTES(4 * 1024), NULL, 5, CORE_AFFINITY_JTAG_TASK, NULL);
	
	
	vTaskStartScheduler();

	for (;;);

	return 0;
}

