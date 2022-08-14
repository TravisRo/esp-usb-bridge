#pragma once

#include <assert.h>
#include "bsp/board.h"

#ifdef __cplusplus
extern "C" {
#endif

// PICO_DEFAULT_WS2812_PIN is generally defined in the boards header file for those that have it.
// If you have a custom board with a WS2812 you can define WS2812_PIN here.
#ifdef PICO_DEFAULT_WS2812_PIN
#define WS2812_PIN PICO_DEFAULT_WS2812_PIN
#endif
	
// pins connected to esp32 target
#define GPIO_BOOT (6)
#define GPIO_RST (7)
#define GPIO_TXD (4)
#define GPIO_RXD (5)

#define CONFIG_BRIDGE_GPIO_TCK (29)
#define CONFIG_BRIDGE_GPIO_TDI (27)
#define CONFIG_BRIDGE_GPIO_TMS (28)
#define CONFIG_BRIDGE_GPIO_TDO (14)

_Static_assert(CONFIG_BRIDGE_GPIO_TMS == CONFIG_BRIDGE_GPIO_TDI+1, "TDI and TMS pins must be sequential! EG: If TDI=27 then TMS must be 28");

#define SLAVE_UART_NUM          uart1
#define SLAVE_UART_BUF_SIZE     (2 * 1024)
#define SLAVE_UART_DEFAULT_BAUD 115200

#define USB_SEND_RINGBUFFER_SIZE SLAVE_UART_BUF_SIZE

#define CONFIG_BRIDGE_MSC_VOLUME_LABEL "ESPPROG_MSC"

#define GET_BYTE(n, b)          (((n) >> ((b) * 8)) & 0xFF)

#define CORE_AFFINITY_USB_TASK (2)
#define CORE_AFFINITY_WS2812_TASK (2)
#define CORE_AFFINITY_JTAG_TASK (1)
#define CORE_AFFINITY_SERIAL_TASK (1)
#define CORE_AFFINITY_MSC_TASK (1)

/**
 * @brief Chip models
 */
typedef enum
{
	CHIP_ESP32 = 1,		//!< ESP32
	CHIP_ESP32S2 = 2,	//!< ESP32-S2
	CHIP_ESP32S3 = 9,	//!< ESP32-S3
	CHIP_ESP32C3 = 5,	//!< ESP32-C3
	CHIP_ESP32H2 = 6,	//!< ESP32-H2
} esp_chip_model_t;

// espressif uses a uint8_t StackType_t in there freertos port.  Most everyone else uses a uint32_t
#define STACK_SIZE_FROM_BYTES(stackSizeInBytes) ((stackSizeInBytes)/sizeof(StackType_t))

#define eub_abort() do{ printf("eub_abort!!\r\n"); }while(1)
#ifdef __cplusplus
}
#endif
