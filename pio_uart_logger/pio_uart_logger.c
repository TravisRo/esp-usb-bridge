#include "pio_uart_logger.h"
#include "pico/stdio_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "hardware/dma.h"
#include "uart_tx.pio.h"
#include "semphr.h"
#include "pico/stdio/driver.h"

#define PIO_DMA_IRQ (DMA_IRQ_1)


typedef struct _logger_t
{
	TaskHandle_t task_handle;
	StreamBufferHandle_t stream_handle;
	bool is_ready;
	uint32_t dma_chan;
	PIO pio;
	uint32_t sm;
	uint32_t pin;
	uint32_t bit_rate;
	SemaphoreHandle_t dma_idle_sem;
	StaticSemaphore_t dma_idle_sem_def;
}logger_t;

static logger_t logger;

static void __not_in_flash_func(pio_uart_log_func)(const char *buf, int len)
{
	if (!logger.is_ready) return;
	
	if (portCHECK_IF_IN_ISR())
	{
		BaseType_t higherPriorityTaskWoken;
		portENTER_CRITICAL();
		xStreamBufferSendFromISR(logger.stream_handle, buf, len, &higherPriorityTaskWoken);
		portEXIT_CRITICAL();
		portYIELD_FROM_ISR(higherPriorityTaskWoken);
	}
	else
	{
		portENTER_CRITICAL();
		xStreamBufferSend(logger.stream_handle, buf, len, 0);
		portEXIT_CRITICAL();
	}
}

static void __not_in_flash_func(pio_uart_log_dma_handler)(void)
{
	if (dma_irqn_get_channel_status(PIO_DMA_IRQ, logger.dma_chan))
	{
		BaseType_t higherPriorityTaskWoken;
		dma_irqn_acknowledge_channel(PIO_DMA_IRQ, logger.dma_chan);
		xSemaphoreGiveFromISR(logger.dma_idle_sem, &higherPriorityTaskWoken);
		portYIELD_FROM_ISR(higherPriorityTaskWoken);
	}
}


static void stdio_dma_init(void)
{
	// Configure a channel to write the same word (32 bits) repeatedly to PIO0
// SM0's TX FIFO, paced by the data request signal from that peripheral.
	logger.dma_chan = dma_claim_unused_channel(true);
	dma_channel_config c = dma_channel_get_default_config(logger.dma_chan);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
	channel_config_set_read_increment(&c, true);
	channel_config_set_write_increment(&c, false);
	channel_config_set_dreq(&c, pio_get_dreq(logger.pio, logger.sm, true));

	dma_channel_configure(
		logger.dma_chan,
		&c,
		&logger.pio->txf[logger.sm],		// Write address (only need to set this once)
		NULL,								// Don't provide a read address yet
		1,									// Write the same value many times, then halt and interrupt
		false								// Don't start yet
		);

	// Configure the processor to run pio_uart_log_dma_handler() when IO_DMA_IRQ is asserted
	// NOTE: There is some overhead with shared handlers but code is more manageable with them.
	irq_add_shared_handler(PIO_DMA_IRQ, pio_uart_log_dma_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
	//irq_set_exclusive_handler(PIO_DMA_IRQ, pio_uart_log_dma_handler);
	
	// enable dma interrupt for PIO_DMA_IRQ
	irq_set_enabled(PIO_DMA_IRQ, true);
	
	// Clear the interrupt request.
	dma_irqn_acknowledge_channel(PIO_DMA_IRQ, logger.dma_chan);
	
	// enable DMA interrupt for this channel
	dma_irqn_set_channel_enabled(PIO_DMA_IRQ, logger.dma_chan, true);
}

static void stdio_pio_init(void)
{
	static stdio_driver_t logger_drv = 
	{ 
		.out_chars = pio_uart_log_func,
		.out_flush = NULL,
		.in_chars = NULL,
		.next = NULL,
#if PICO_STDIO_ENABLE_CRLF_SUPPORT
		.last_ended_with_cr = false,
		.crlf_enabled = false
#endif
	};
	
	uint offset = pio_add_program(logger.pio, &uart_tx_program);
	uart_tx_program_init(logger.pio, logger.sm, offset, logger.pin, logger.bit_rate);

	stdio_set_driver_enabled(&logger_drv, true);
}

static void pio_uart_logger_task(void* p)
{
	static uint8_t temp_buffer[2][1024];
	stdio_pio_init();
	stdio_dma_init();
	uint buf_index = 0;
	size_t minSpaceAvail = UINT32_MAX;
	logger.is_ready = true;
	for (;;)
	{
		size_t len = xStreamBufferReceive(logger.stream_handle, temp_buffer[buf_index], sizeof(temp_buffer[0]), portMAX_DELAY /* pdMS_TO_TICKS(5000) */);
		if (len)
		{
			size_t spaceAvail = xStreamBufferSpacesAvailable(logger.stream_handle);
			if (spaceAvail < minSpaceAvail)
				minSpaceAvail = spaceAvail;
			
			xSemaphoreTake(logger.dma_idle_sem, portMAX_DELAY);
			dma_channel_set_read_addr(logger.dma_chan, temp_buffer[buf_index], false);
			dma_channel_set_trans_count(logger.dma_chan, len, true);
			buf_index ^= 1;
		}
		else
		{
			len = sprintf((char*)temp_buffer[buf_index], "Min Log Space = %u\r\n", minSpaceAvail);
			
			xSemaphoreTake(logger.dma_idle_sem, portMAX_DELAY);
			dma_channel_set_read_addr(logger.dma_chan, temp_buffer[buf_index], false);
			dma_channel_set_trans_count(logger.dma_chan, len, true);
			buf_index ^= 1;
			
		}
	}
}

void start_pio_uart_logger(PIO pio, uint32_t pin, uint32_t bit_rate)
{
	memset(&logger, 0, sizeof(logger));
	logger.pio = pio;
	logger.pin = pin;
	logger.bit_rate = bit_rate;
	
	logger.sm = pio_claim_unused_sm(logger.pio, true);
	
	logger.dma_idle_sem = xSemaphoreCreateBinaryStatic(&logger.dma_idle_sem_def);
	xSemaphoreGive(logger.dma_idle_sem);
	
	// disable the default uart driver for stdio.  We will use our PIO uart logger
	stdio_set_driver_enabled(&stdio_uart, false);

	logger.stream_handle = xStreamBufferCreate(8192, 1);
	xTaskCreateAffinitySet(pio_uart_logger_task, "logger_task", STACK_SIZE_FROM_BYTES(4 * 1024), NULL, tskIDLE_PRIORITY+1, CORE_AFFINITY_LOGGER_TASK, &logger.task_handle);
}
