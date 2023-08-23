#ifndef _UBP_CONFIG_H
#define _UBP_CONFIG_H

#include <assert.h>
#include "bsp/board.h"

#define FWVER_MAJOR (0)
#define FWVER_MINOR (3)

/* 
 * RP2040's can generally be over-clocked up to 260mhz without any problems. This
 * brings our FLASH frequency up to 130mhz which is right at the maximum speed for
 * W25Q flash ICs (unless it's a really really old one)
 */
#define RP2040_OVERCLOCK_ENABLED (0)

/*
 * PICO_DEFAULT_WS2812_PIN is generally defined in the boards header file for those that have it.
 * You can specify your board at the top of the parent CMakeList.txt file. 
 * EG: set(PICO_BOARD tzt_rgb_usbc_rp2040)
 * 
 * The board header files are located in the pico-sdk in the folder pico-sdk/src/boards/include/boards
 * 
 * If you have a custom board with a WS2812 you can define WS2812_PIN here.
 */

#if defined(PICO_DEFAULT_WS2812_PIN) && !defined(WS2812_PIN)
#define WS2812_PIN PICO_DEFAULT_WS2812_PIN
#endif
	
/* 
 * Serial Programming Interface PIN assignments
 * NOTE: These can also be set with a project define or from the
 * make command line.
 */ 

#ifndef GPIO_BOOT
#define GPIO_BOOT (6)
#endif

#ifndef GPIO_RST
#define GPIO_RST (7)
#endif

#ifndef GPIO_TXD
#define GPIO_TXD (4)
#endif

#ifndef GPIO_RXD
#define GPIO_RXD (5)
#endif
////////////////////////////////////////////////////////////////////

/* 
 * Serial Programming Interface UART
 * NOTE: These can also be set with a project define or from the
 * make command line.
 */ 

/* Some dev boards don't route out uart1 pins in which case you can
 * use uart0 and change GPIO_TXD, GPIO_RXD accordingly.
 */
#ifndef PROG_UART
#define PROG_UART			uart1
#endif

#ifndef PROG_UART_BUF_SIZE
#define PROG_UART_BUF_SIZE	(2 * 1024)
#endif

#ifndef PROG_UART_BITRATE
#define PROG_UART_BITRATE	115200
#endif


/* 
 * JTAG debugging Interface PIN assignments
 * NOTE: These can also be set with a project define or from the
 * make command line.
 */ 
#ifndef GPIO_TCK
#define GPIO_TCK (19)
#endif

#ifndef GPIO_TDI
#define GPIO_TDI (17)
#endif

#ifndef GPIO_TMS
#define GPIO_TMS (18)
#endif

#ifndef GPIO_TDO
#define GPIO_TDO (16)
#endif

////////////////////////////////////////////////////////////////////

_Static_assert(GPIO_TMS == GPIO_TDI+1, "TDI and TMS pins must be sequential! EG: If TDI=27 then TMS must be 28");

#define CONFIG_BRIDGE_MSC_VOLUME_LABEL "ESPPROG_MSC"

#define GET_BYTE(n, b)          (((n) >> ((b) * 8)) & 0xFF)

#define CORE_AFFINITY_USB_TASK (1)
#define CORE_AFFINITY_WS2812_TASK (2)
#define CORE_AFFINITY_LOGGER_TASK (2)
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

#define eub_abort() do{ printf("eub_abort!!\r\n"); for(;;); }while(1)

// Log level defines (higher is more logging)
#define ESP_LOG_NONE	(-1)
#define ESP_LOG_ERROR	(0)
#define ESP_LOG_WARN	(1)
#define ESP_LOG_INFO	(2)
#define ESP_LOG_DEBUG	(3)
#define ESP_LOG_VERBOSE	(4)

/*
 * PIO UART logger UART TX pin.
 * 
 * This can be ANY unused pin since it doesn't use an actual UART peripheral.
 * NOTE: These can also be set with a project define or from the
 * make command line.
 */

#ifndef LOGGER_UART_TX_PIN
#define LOGGER_UART_TX_PIN	(0)
#endif

/* 
 * NOTE: Since by default we are running the logger uart rate really fast (1.5M),
 * we are using 2 stop bits. If you lower the baudrate, you may want to decrease
 * this to 1 for slightly faster logging.
 */
#ifndef LOGGER_UART_STOPBITS
#define LOGGER_UART_STOPBITS	(2)
#endif

/* 
 * NOTE: This may be to high for some serial ICs!
 * I use a CH340G for logging and 1.5M works better than 921600.
 * Presumably, the CH340G osc can hit 1.5M with 0% error.
 */
#ifndef LOGGER_UART_BITRATE
#define LOGGER_UART_BITRATE	(1500000)
#endif

/*
 * Only show warning and errors by default
 */
#ifndef LOG_LEVEL
#define LOG_LEVEL (ESP_LOG_NONE)
#endif
////////////////////////////////////////////////////////////////////

#define LOGGING_ENABLED() (LOG_LEVEL > ESP_LOG_NONE)

#ifndef MSC_ENABLED
#define MSC_ENABLED 0
#endif

#ifndef JTAG_ENABLED
#define JTAG_ENABLED 1
#endif

#ifndef GPIO_ADC_0
#define GPIO_ADC_0 (26)
#define GPIO_ADC_0_RPLUS (10000)
#define GPIO_ADC_0_RMINUS (3300)
#endif 

#ifndef GPIO_ADC_1
#define GPIO_ADC_1 (27)
#define GPIO_ADC_1_RPLUS (10000)
#define GPIO_ADC_1_RMINUS (10000)
#endif 

#ifndef GPIO_ADC_2
#define GPIO_ADC_2 (28)
#define GPIO_ADC_2_RPLUS (10000)
#define GPIO_ADC_2_RMINUS (3300)
#endif 

#ifndef GPIO_ADCVPP
#define GPIO_ADCVPP (GPIO_ADC_0)
#endif 


#endif
