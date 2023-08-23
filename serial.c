// Copyright 2020-2021 Espressif Systems (Shanghai) CO LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <pico/stdlib.h>
#include <stdlib.h>
#include "ubp_config.h"
#include "esp_log.h"
#include "esp_err.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "pico.h"
#include "timers.h"
#include "serial.h"
#include "jtag.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/dma.h"
#include "tusb.h"
#include "components/esp_loader/port/rp2040_port.h"
#include "stream_buffer.h"
#include "ws2812.h"
#include "hardware/adc.h"
#include <hardware/flash.h>
#include <hardware/watchdog.h>
#include "pico/float.h"
#include "pico/bootrom.h"

static const char *TAG = "bridge_serial";

static alarm_id_t state_change_timer = -1;

static volatile bool serial_init_finished = false;
static volatile bool serial_read_enabled = false;

TaskHandle_t cdc_to_uart_task_handle;
TaskHandle_t uart_to_cdc_task_handle;
TaskHandle_t flash_write_lock_task_handle;

StreamBufferHandle_t uart_to_cdc_stream_handle;
StaticStreamBuffer_t uart_to_cdc_stream_context;

StaticSemaphore_t sem_flash_lock_def;
SemaphoreHandle_t sem_flash_lock_handle;

volatile bool other_core_locked = false;
volatile bool flash_in_progress = false;

static uint32_t last_bit_rate = PROG_UART_BITRATE;

typedef struct _uart_dma_tx_t
{
	int channel;
	StaticSemaphore_t sem_ready_def;
	SemaphoreHandle_t sem_ready_handle;
	uint8_t buf[CFG_TUD_CDC_EP_BUFSIZE];
}uart_dma_tx_t;

typedef union _cal_value_t
{
	float val;
	struct
	{
		uint8_t b0;
		uint8_t b1;
		uint8_t b2;
		uint8_t b3;
	};
}cal_value_t;

typedef enum _adc_cal_type
{
	ADC_CAL_VREF,
	ADC_CAL_A0,
	ADC_CAL_A1,
	ADC_CAL_A2,
	
	ADC_CAL_MAX
}adc_cal_type;

static uart_dma_tx_t uart_dma_tx;
uint8_t uart_to_cdc_stream_buffer[PROG_UART_BUF_SIZE];

static float adc_calibration_cache[ADC_CAL_MAX] = { 3.3F, 1.0F, 1.0F, 1.0F };

static bool is_in_command_mode(void)
{
	return (last_bit_rate > 1200 && last_bit_rate <= 2400);
}
static inline void set_esp_pin(uint pin, bool val)
{
	if (val)
	{
		// set to input, enable pullup
		gpio_put(pin, true);
		gpio_set_dir(pin, true);
		gpio_set_pulls(pin, true, false);
		gpio_set_dir(pin, false);
	}
	else
	{
		// set low, set to output
		gpio_put(pin, false);
		gpio_set_dir(pin, true);
	}
}

static void dma_uart_tx_start(const uint8_t* buf, uint32_t len)
{
	dma_channel_set_read_addr(uart_dma_tx.channel, buf, false);
	dma_channel_set_trans_count(uart_dma_tx.channel, len, true);
}

static void __not_in_flash_func(dma_handler_uart_tx)(void)
{
	BaseType_t higherPriorityTaskWoken;

	// Clear the interrupt request.
	if (dma_channel_get_irq0_status(uart_dma_tx.channel))
	{
		dma_channel_acknowledge_irq0(uart_dma_tx.channel);
		xSemaphoreGiveFromISR(uart_dma_tx.sem_ready_handle, &higherPriorityTaskWoken);
		portYIELD_FROM_ISR(higherPriorityTaskWoken);
	}

}

static void dma_init_uart_tx(void)
{
	uart_dma_tx.sem_ready_handle = xSemaphoreCreateBinaryStatic(&uart_dma_tx.sem_ready_def);
	xSemaphoreGive(uart_dma_tx.sem_ready_handle);

	uart_dma_tx.channel = dma_claim_unused_channel(true);
	dma_channel_config c = dma_channel_get_default_config(uart_dma_tx.channel);

	channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
	channel_config_set_read_increment(&c, true);
	channel_config_set_write_increment(&c, false);
	channel_config_set_dreq(&c, uart_get_dreq(PROG_UART, true));

	dma_channel_configure(
		uart_dma_tx.channel,
		&c,
		&uart_get_hw(PROG_UART)->dr, // Write address (only need to set this once)
		NULL,                        // Don't provide a read address yet
		0,                           // Write the same value many times, then halt and interrupt
		false                        // Don't start yet
		);


	// Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
	irq_add_shared_handler(DMA_IRQ_0, dma_handler_uart_tx, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
	//irq_set_exclusive_handler(DMA_IRQ_0, dma_handler_uart_tx);
	// Tell the DMA to raise IRQ line 0 when the channel finishes a block
	irq_set_enabled(DMA_IRQ_0, true);

	// Clear the interrupt request.
	dma_channel_acknowledge_irq0(uart_dma_tx.channel);
	dma_channel_set_irq0_enabled(uart_dma_tx.channel, true);
}

static void __not_in_flash_func(uart_rx_isr)(void)
{
	BaseType_t higherPriorityTaskWoken = pdFALSE;
	static uint8_t temp_buffer[16];
	static uint32_t temp_buffer_len;
	temp_buffer_len = 0;
	while (uart_is_readable(PROG_UART))
	{

		temp_buffer[temp_buffer_len++] = uart_getc(PROG_UART);
		if (temp_buffer_len == sizeof(temp_buffer))
		{
			if (!is_in_command_mode())
			{
				xStreamBufferSendFromISR(uart_to_cdc_stream_handle, temp_buffer, temp_buffer_len, &higherPriorityTaskWoken);
			}
			temp_buffer_len = 0;
		}
	}
	if (temp_buffer_len > 0 && uart_to_cdc_stream_handle)
	{
		if (!is_in_command_mode())
		{
			xStreamBufferSendFromISR(uart_to_cdc_stream_handle, temp_buffer, temp_buffer_len, &higherPriorityTaskWoken);
		}
	}

	portYIELD_FROM_ISR(higherPriorityTaskWoken);
}
static int32_t seq_pos = 0;
static int32_t seq_data_pos = 0; 
static int32_t seq_data_len = 0; 
static uint8_t seq_last_cmd = 0;
;const uint8_t CMD_SEQ_START[] = { 0xE4, 0xEA, 0x82, 0x11, 0xCF, 0x56, 0x41, 0x76, 0xBC, 0x97, 0x6A, 0x4D, 0x3C, 0x8D, 0xED, 0x82 };

static float calc_resistor_divider(float* volts_in, uint* r_plus, uint* r_minus, float* volts_out)
{
	if (volts_in == NULL)
	{
		return (*volts_out)*(((float)*r_plus) + ((float)*r_minus)) / ((float)*r_minus);
	}
	else if (r_plus == NULL)
	{
		return (*volts_in)*((float)*r_minus) / (*volts_out) - ((float)*r_minus);
	}
	else if (r_minus == NULL)
	{
		return (*volts_out)*((float)*r_plus) / ((*volts_in) - (*volts_out)) ;
	}
	else if (volts_out == NULL)
	{
		return (*volts_in)*((float)*r_minus) / (((float)*r_plus) + ((float)*r_minus)) ;
	}
}

static char adcReportBuffer[128];

void cmd_report_adc(uint gpio, int avg)
{
	int i;
	int len;
	uint adc_cnt = 0;
	float volts_input;
	float volts_ref;
	float volts_adc;
	uint r_minus;
	uint r_plus;
	adc_select_input(gpio - GPIO_ADC_0);
	(void)adc_read();
	
	for (i = 0; i < avg; i++)
	{
		adc_cnt += adc_read();
	}
	adc_cnt /= avg;
				
	//gpio_init(gpio);
	//gpio_put(gpio, false);
	//gpio_set_dir(gpio, true);
	
	volts_ref = adc_calibration_cache[ADC_CAL_VREF];
	volts_adc = volts_ref * ((float)adc_cnt / 4095.0F);
	
	switch (gpio)
	{
	case GPIO_ADC_0:
		r_plus = GPIO_ADC_0_RPLUS;
		r_minus = GPIO_ADC_0_RMINUS;
		break;
	case GPIO_ADC_1:
		r_plus = GPIO_ADC_1_RPLUS;
		r_minus = GPIO_ADC_1_RMINUS;
		break;
	case GPIO_ADC_2:
		r_plus = GPIO_ADC_2_RPLUS;
		r_minus = GPIO_ADC_2_RMINUS;
		break;
	default:
		return;
	}
	
	volts_input = calc_resistor_divider(NULL, &r_plus, &r_minus, &volts_adc);
	volts_input *= adc_calibration_cache[1 + (gpio - GPIO_ADC_0)];
	if (isnanf(volts_input))
		volts_input = 0.0F;
	if (isnanf(volts_adc))
		volts_adc = 0.0F;
	
	len = sprintf(adcReportBuffer, "ADC%u CNT=%u,R+=%u,R-=%u,vRef=%f,vOut=%f,vIn=%f\r\n", gpio - GPIO_ADC_0, adc_cnt, r_plus, r_minus, volts_ref, volts_adc, volts_input);
	xStreamBufferSend(uart_to_cdc_stream_handle, adcReportBuffer, len, portMAX_DELAY);
	
}
#define FLASH_TARGET_OFFSET (1792*1024)                                                         //++ Starting Flash Storage location after 1.8MB ( of the 2MB )
const uint8_t *flash_target_contents = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET); 

static void load_adc_calibrations()
{
	memcpy(adc_calibration_cache, flash_target_contents, sizeof(adc_calibration_cache));
	
	
	if (isnanf(adc_calibration_cache[ADC_CAL_VREF]) || adc_calibration_cache[ADC_CAL_VREF] < 3.0F || adc_calibration_cache[ADC_CAL_VREF] > 3.6F)
		adc_calibration_cache[ADC_CAL_VREF] = 3.3F;

	if (isnanf(adc_calibration_cache[ADC_CAL_A0]) || adc_calibration_cache[ADC_CAL_A0] < 0.8F || adc_calibration_cache[ADC_CAL_A0] > 1.2F)
		adc_calibration_cache[ADC_CAL_A0] = 1.0F;
	
	if (isnanf(adc_calibration_cache[ADC_CAL_A1]) || adc_calibration_cache[ADC_CAL_A1] < 0.8F || adc_calibration_cache[ADC_CAL_A1] > 1.2F)
		adc_calibration_cache[ADC_CAL_A1] = 1.0F;
	
	if (isnanf(adc_calibration_cache[ADC_CAL_A2]) || adc_calibration_cache[ADC_CAL_A2] < 0.8F || adc_calibration_cache[ADC_CAL_A2] > 1.2F)
		adc_calibration_cache[ADC_CAL_A2] = 1.0F;
}

static void report_adc_calibration(adc_cal_type calType)
{
	if (calType >= ADC_CAL_MAX) return;
	int len = sprintf(adcReportBuffer, "ADCCAL%u=%f\r\n", calType, adc_calibration_cache[calType]);
	xStreamBufferSend(uart_to_cdc_stream_handle, adcReportBuffer, len, portMAX_DELAY);
	
}
static void report_to_adc(const char* str)
{
	int len = strlen(str);
	xStreamBufferSend(uart_to_cdc_stream_handle, str, len, portMAX_DELAY);
}

static void write_adc_calibration(adc_cal_type calType, float val)
{
	if (calType >= ADC_CAL_MAX) return;
	adc_calibration_cache[calType] = val;
}

#if DISABLED
static void __not_in_flash_func (save_adc_calibrations)(void)
{
	uint8_t * write_mem = malloc(FLASH_PAGE_SIZE);
	if (write_mem == NULL) return;
	memset(write_mem + sizeof(adc_calibration_cache), 0, FLASH_PAGE_SIZE - sizeof(adc_calibration_cache));
	memcpy(write_mem, adc_calibration_cache, sizeof(adc_calibration_cache));
	
	vTaskDelay(pdMS_TO_TICKS(500));
	flash_in_progress = true;
	xSemaphoreGive(sem_flash_lock_handle);
	while (!other_core_locked) ;
	taskENTER_CRITICAL();
	flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
	flash_range_program(FLASH_TARGET_OFFSET, write_mem, FLASH_PAGE_SIZE);
	taskEXIT_CRITICAL();
	flash_in_progress = false;
	free(write_mem);
	watchdog_reboot(0,0,1);
}
#endif

static void save_adc_calibrations(void)
{
	reset_usb_boot(1 << PICO_DEFAULT_LED_PIN, 0);
}

static void cdc_seq_filter(uint8_t* buf, uint32_t len)
{
	uint32_t pos_buf;
	uint8_t seq_cmd;
	static uint8_t seq_data[4];
	cal_value_t cal_value;
	for (pos_buf = 0; pos_buf < len; pos_buf++)
	{
		if (seq_pos == sizeof(CMD_SEQ_START))
		{
			if (seq_data_len)
				seq_cmd = seq_last_cmd;
			else
			{
				seq_cmd = buf[pos_buf];
				seq_last_cmd = seq_cmd;
			}

			// Match special sequence!
			switch (seq_cmd)
			{
			case 0:
				cmd_report_adc(GPIO_ADC_0, 64);
				seq_pos = 0;
				seq_data_len = 0;
				break;
			case 1:
				cmd_report_adc(GPIO_ADC_1, 64);
				seq_pos = 0;
				seq_data_len = 0;
				break;
			case 2:
				cmd_report_adc(GPIO_ADC_2, 64);
				seq_pos = 0;
				seq_data_len = 0;
				break;
			case ADC_CAL_VREF + 3:	// VREF Calibration
			case ADC_CAL_A0 + 3:	// ADC0 Multiplier
			case ADC_CAL_A1 + 3:	// ADC1 Multiplier
			case ADC_CAL_A2 + 3:	// ADC2 Multiplier
				if (seq_data_len == 0)
				{
					seq_data_len = -1;
				}
				else if (seq_data_len == -1)
				{
					if (buf[pos_buf] > sizeof(seq_data))
					{
						seq_pos = 0;
						seq_data_len = 0;
						
					}
					else
					{
						seq_data_len = buf[pos_buf];
						seq_data_pos = 0;
					}

				}
				else
				{
					seq_data[seq_data_pos++] = buf[pos_buf];
					if (seq_data_pos == seq_data_len)
					{
						if (seq_data_len == 1)
						{
							// read calibration value
							report_adc_calibration(seq_cmd - 3);
						}
						else if (seq_data_len == 4)
						{
							// write calibration value
							cal_value.b0 = seq_data[0];
							cal_value.b1 = seq_data[1];
							cal_value.b2 = seq_data[2];
							cal_value.b3 = seq_data[3];
							write_adc_calibration(seq_cmd - 3, cal_value.val);	
							report_adc_calibration(seq_cmd - 3);
						}

						seq_pos = 0;
						seq_data_len = 0;
					}
				}
				break;
			case 0xFF:
				save_adc_calibrations();
				load_adc_calibrations();
				report_to_adc("ADC CALIBRATIONS SAVED!\r\n");
				break;
			default:
				seq_pos = 0;
				seq_data_len = 0;
				break;
			}

		}
		else
		{
			if (buf[pos_buf] == CMD_SEQ_START[seq_pos])
				seq_pos++;
			else
			{
				seq_pos = 0;
				seq_data_len = 0;
			}
		}

	}
}

static void cdc_to_uart_task(void* param)
{
	(void)(param);
	uint32_t len;
	for (;;)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (!tud_inited() || !tud_ready()) continue;
		while (tud_cdc_available())
		{
			if (is_in_command_mode())
			{
				len = tud_cdc_read(uart_dma_tx.buf, sizeof(uart_dma_tx.buf));

				// check for special command sequences
				cdc_seq_filter(uart_dma_tx.buf, len);
			}
			else
			{
				xSemaphoreTake(uart_dma_tx.sem_ready_handle, portMAX_DELAY);
				len = tud_cdc_read(uart_dma_tx.buf, sizeof(uart_dma_tx.buf));
				dma_uart_tx_start(uart_dma_tx.buf, len);
			}
		}
	}
}


static void __not_in_flash_func(flash_write_lock_task)(void* param)
{
	for (;;)
	{
		xSemaphoreTake(sem_flash_lock_handle, portMAX_DELAY);
		other_core_locked = true;
		taskENTER_CRITICAL();
		while (flash_in_progress) ;
		taskEXIT_CRITICAL();
		other_core_locked = false;		
	}
}

static void uart_to_cdc_task(void* param)
{
	uint8_t transfer_buffer[64];
	uint32_t length;
	for (;;)
	{
		length = xStreamBufferReceive(uart_to_cdc_stream_handle, transfer_buffer, sizeof(transfer_buffer), portMAX_DELAY);
		if (!tud_inited() || !tud_ready()) continue;
		uint8_t* p_buf = transfer_buffer;
		while (length > 0)
		{
			if (!tud_cdc_write_available())
			{
				vTaskDelay(1);
				continue;
			}
			uint32_t sent = tud_cdc_write(p_buf, length);
			length -= sent;
			p_buf += sent;
		}
		tud_cdc_write_flush();
	}
}

void tud_cdc_rx_cb(uint8_t itf)
{
	if (cdc_to_uart_task_handle)
		xTaskNotifyGive(cdc_to_uart_task_handle);
}


void tud_cdc_line_coding_cb(const uint8_t itf, cdc_line_coding_t const *p_line_coding)
{
	serial_set_baudrate(p_line_coding->bit_rate);
}

static int64_t state_change_timer_cb(alarm_id_t id, void *user_data)
{
	ESP_LOGI(TAG, "BOOT = 1, RST = 1");
	set_esp_pin(GPIO_BOOT, true);
	set_esp_pin(GPIO_RST, true);
	state_change_timer = -1;
	ws2812_set_rgb_state_isr(RGB_LED_STATE_PROG_B1_R1);

	return 0;
}

void tud_cdc_line_state_cb(const uint8_t itf, const bool dtr, const bool rts)
{
	if (!serial_init_finished)
	{
		// This is a callback function which can be invoked without running start_serial_task()
		ESP_LOGW(TAG, "Tasks for the serial interface hasn't been initialized!");
		return;
	}

	// The following transformation of DTR & RTS signals to BOOT & RST is done based on auto reset circutry shown in
	// schematics of ESP boards.

	// defaults for ((dtr && rts) || (!dtr && !rts))
	bool rst = true;
	bool boot = true;

	if (!dtr && rts)
	{
		rst = false;
		boot = true;
	}
	else if (dtr && !rts)
	{
		rst = true;
		boot = false;
	}
	if (state_change_timer !=-1)
	{
		cancel_alarm(state_change_timer);
		state_change_timer = -1;
	}

	if (dtr & rts)
	{
		// The assignment of BOOT=1 and RST=1 is postponed and it is done only if no other state change occurs in time
		// period set by the timer.
		// This is a patch for Esptool. Esptool generates DTR=0 & RTS=1 followed by DTR=1 & RTS=0. However, a callback
		// with DTR = 1 & RTS = 1 is received between. This would prevent to put the target chip into download mode.
		state_change_timer = add_alarm_in_us(10 * 1000 /*us*/, state_change_timer_cb, NULL, true);
	}
	else
	{
		ESP_LOGI(TAG, "DTR = %d, RTS = %d -> BOOT = %d, RST = %d", dtr, rts, boot, rst);

		if (boot && rst)
			ws2812_set_rgb_state(RGB_LED_STATE_PROG_B1_R1);
		else if (!boot && rst)
			ws2812_set_rgb_state(RGB_LED_STATE_PROG_B0_R1);
		else if (boot && !rst)
			ws2812_set_rgb_state(RGB_LED_STATE_PROG_B1_R0);
		else if (!boot && !rst)
			ws2812_set_rgb_state(RGB_LED_STATE_PROG_B0_R0);

		set_esp_pin(GPIO_BOOT, boot);
		set_esp_pin(GPIO_RST, rst);

#ifdef DISABLED
		// On ESP32, TDI jtag signal is on GPIO12, which is also a strapping pin that determines flash voltage.
		// If TDI is high when ESP32 is released from external reset, the flash voltage is set to 1.8V, and the chip will fail to boot.
		// As a solution, MTDI signal forced to be low when RST is about to go high.
		static bool tdi_bootstrapping = false;
		if (jtag_get_target_model() == CHIP_ESP32 && !tdi_bootstrapping && boot && !rst)
		{
			jtag_task_suspend();
			tdi_bootstrapping = true;
			gpio_put(GPIO_TDO, 0);
			ESP_LOGW(TAG, "jtag task suspended");
		}
		if (tdi_bootstrapping && boot && rst)
		{
			sleep_us(1000);
			jtag_task_resume();
			tdi_bootstrapping = false;
			ESP_LOGW(TAG, "jtag task resumed");
		}
#endif
	}
}

static int32_t uart_read_for_loader(uint8_t* buf, uint32_t len, uint32_t timeout_ms)
{
	int total_transferred = 0;
	absolute_time_t timeout_time = make_timeout_time_ms(timeout_ms);
	while (len > 0 && !time_reached(timeout_time))
	{
		absolute_time_t cur_time = get_absolute_time();

		int64_t diff_time = absolute_time_diff_us(cur_time, timeout_time);
		if (diff_time <= 0) break;
		size_t rcv_length = xStreamBufferReceive(uart_to_cdc_stream_handle, buf, len, pdMS_TO_TICKS(us_to_ms(diff_time)));
		buf += rcv_length;
		total_transferred += rcv_length;
		len -= rcv_length;
	}

	return total_transferred;
}

static int32_t uart_write_for_loader(const uint8_t* buf, uint32_t len, uint32_t timeout_ms)
{
	xSemaphoreTake(uart_dma_tx.sem_ready_handle, portMAX_DELAY);
	dma_uart_tx_start(buf, len);
	return len;
}

static uint32_t uart_set_baudrate_for_loader(uint32_t baudrate)
{
	return uart_set_baudrate(PROG_UART, baudrate);
}

static void gpio_set_boot_pin_for_loader(bool val)
{
	set_esp_pin(GPIO_BOOT, val);
}

static void gpio_set_rst_pin_for_loader(bool val)
{
	set_esp_pin(GPIO_RST, val);
}

void start_serial_task(void *pvParameters)
{

	uart_to_cdc_stream_handle = xStreamBufferGenericCreateStatic(sizeof(uart_to_cdc_stream_buffer), 1, pdFALSE, uart_to_cdc_stream_buffer, &uart_to_cdc_stream_context);


	// Set up our UART with a basic baud rate.
	uart_init(PROG_UART, PROG_UART_BITRATE);

	// Set the TX and RX pins by using the function select on the GPIO
	// Set datasheet for more information on function select
	gpio_set_function(GPIO_TXD, GPIO_FUNC_UART);
	gpio_set_function(GPIO_RXD, GPIO_FUNC_UART);

	// Set UART flow control CTS/RTS, we don't want these, so turn them off
	uart_set_hw_flow(PROG_UART, false, false);

	// Set our data format
	uart_set_format(PROG_UART, 8, 1, UART_PARITY_NONE);

	// Turn on FIFO's
	uart_set_fifo_enabled(PROG_UART, true);

	// Set up a RX interrupt
	// We need to set up the handler first
	// Select correct interrupt for the UART we are using
	int UART_IRQ = PROG_UART == uart0 ? UART0_IRQ : UART1_IRQ;

	// And set up and enable the interrupt handlers
	irq_set_exclusive_handler(UART_IRQ, uart_rx_isr);


	// Now enable the UART to send interrupts - RX only
	uart_set_irq_enables(PROG_UART, true, false);

	irq_set_enabled(UART_IRQ, true);

	// Initialize the boot and reset pins that connect to the target esp32
	gpio_init_mask((1 << GPIO_BOOT) | (1 << GPIO_RST));

	//gpio_set_drive_strength(GPIO_BOOT, GPIO_DRIVE_STRENGTH_2MA);
	//gpio_set_drive_strength(GPIO_RST, GPIO_DRIVE_STRENGTH_2MA);

	set_esp_pin(GPIO_BOOT, true);
	set_esp_pin(GPIO_RST, true);

	serial_init_finished = true;
	serial_read_enabled = true;
	dma_init_uart_tx();

	loader_rp2040_config_t loader_cfg =
	{
		.read_uart = uart_read_for_loader,
		.write_uart = uart_write_for_loader,
		.set_baud_rate = uart_set_baudrate_for_loader,
		.set_boot_pin = gpio_set_boot_pin_for_loader,
		.set_rst_pin = gpio_set_rst_pin_for_loader,

	};
	loader_port_rp2040_init(&loader_cfg);

	load_adc_calibrations();
	adc_init();
	adc_gpio_init(GPIO_ADC_0);
	adc_gpio_init(GPIO_ADC_1);
	adc_gpio_init(GPIO_ADC_2);

	sem_flash_lock_handle = xSemaphoreCreateBinaryStatic(&sem_flash_lock_def);
	xSemaphoreTake(sem_flash_lock_handle, 0);
	xTaskCreateAffinitySet(uart_to_cdc_task, "uart_to_cdc", STACK_SIZE_FROM_BYTES(8 * 1024), NULL, 5, CORE_AFFINITY_SERIAL_TASK, (TaskHandle_t *)&uart_to_cdc_task_handle);
	xTaskCreateAffinitySet(cdc_to_uart_task, "cdc_to_uart", STACK_SIZE_FROM_BYTES(8 * 1024), NULL, 5, CORE_AFFINITY_SERIAL_TASK, (TaskHandle_t *)&cdc_to_uart_task_handle);
	
	//xTaskCreateAffinitySet(flash_write_lock_task, "flash_write_lock", STACK_SIZE_FROM_BYTES(2 * 1024), NULL, configMAX_PRIORITIES - 1, 2, (TaskHandle_t *)&flash_write_lock_task_handle);

	vTaskDelete(NULL);
}

void serial_set(const bool enable)
{
	if (serial_read_enabled != enable)
	{
		if (enable)
		{
			vTaskResume(uart_to_cdc_task_handle);
			vTaskResume(cdc_to_uart_task_handle);
			serial_read_enabled = true;
			ws2812_set_rgb_state(RGB_LED_STATE_END);
		}
		else
		{
			serial_read_enabled = false;
			vTaskSuspend(uart_to_cdc_task_handle);
			vTaskSuspend(cdc_to_uart_task_handle);
			ws2812_set_rgb_state(RGB_LED_STATE_MSC_START);
		}
	}
}

bool serial_set_baudrate(uint32_t bit_rate)
{
	if (last_bit_rate != bit_rate)
	{
		last_bit_rate = bit_rate;
		if (!is_in_command_mode())
		{
			uart_set_baudrate(PROG_UART, bit_rate);
		}
	}

	return true;
}
