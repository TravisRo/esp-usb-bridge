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
#include "esp_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "jtag.h"
#include "tusb.h"
#include "config.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "generated/jtag.pio.h"
#include "hardware/dma.h"
#include "stream_buffer.h"
#include "ws2812.h"

#define MAKE_DAT(tdo, tms, tdi) ((tdo << 2)|(tms << 1)|(tdi << 0))
#define IS_TDO(dat) (dat & 0b100) ? true : false

#define USB_RCVBUF_SIZE             4096

// TODO: We shouldn't have to buffer up to 32k of data!  This indicates a design problem on the host side
// and will require fixing whatever is wrong in the openocd-esp32 usb-jtag implementation.
#define USB_SNDBUF_SIZE             (32*1024)

#define ROUND_UP_BITS(x)            ((x + 7) & (~7))



/* esp usb serial protocol specific definitions */
#define JTAG_PROTO_MAX_BITS      (512)
#define JTAG_PROTO_CAPS_VER 1     /*Version field. */
typedef struct __attribute__((packed))
{
	uint8_t proto_ver;        /*Protocol version. Expects JTAG_PROTO_CAPS_VER for now. */
	uint8_t length;           /*of this plus any following descriptors */
} jtag_proto_caps_hdr_t;

#define JTAG_PROTO_CAPS_SPEED_APB_TYPE 1
typedef struct __attribute__((packed))
{
	uint8_t type;
	uint8_t length;
} jtag_gen_hdr_t;

typedef struct __attribute__((packed))
{
	uint8_t type;             /*Type, always JTAG_PROTO_CAPS_SPEED_APB_TYPE */
	uint8_t length;           /*Length of this */
	uint16_t apb_speed_10khz; /*ABP bus speed, in 10KHz increments. Base speed is half
	                           * this. */
	uint16_t div_min;         /*minimum divisor (to base speed), inclusive */
	uint16_t div_max;         /*maximum divisor (to base speed), inclusive */
} jtag_proto_caps_speed_apb_t;

typedef struct
{
	jtag_proto_caps_hdr_t proto_hdr;
	jtag_proto_caps_speed_apb_t caps_apb;
} jtag_proto_caps_t;

typedef struct _jtag_context_t
{
	PIO pio;
	uint sm_tx;
	uint sm_rx;
	uint offset_tx;
	uint offset_rx;

	uint pio_rx_dma_channel;

	uint pio_rx_bits_cached;

	uint tdo_bits_total;
	uint tdo_bits_sent;

	float pio_clkdiv;
}jtag_context_t;

#define VEND_JTAG_SETDIV        0
#define VEND_JTAG_SETIO         1
#define VEND_JTAG_GETTDO        2
#define VEND_JTAG_SET_CHIPID    3

#define JTAG_BASE_FREQ_HZ (20000000)
#define TCK_FREQ(khz) ((khz * 2) / 10)
#define TCK_FREQ_HZ_FROM_DIV(div)((float)JTAG_BASE_FREQ_HZ/(div))

static const jtag_proto_caps_t jtag_proto_caps = 
{
	{ .proto_ver = JTAG_PROTO_CAPS_VER, .length = sizeof(jtag_proto_caps_hdr_t) + sizeof(jtag_proto_caps_speed_apb_t) },
	{ .type = JTAG_PROTO_CAPS_SPEED_APB_TYPE, .length = sizeof(jtag_proto_caps_speed_apb_t), .apb_speed_10khz = TCK_FREQ(JTAG_BASE_FREQ_HZ / 1000), .div_min = 1, .div_max = 200 }
};

static jtag_context_t jtag_ctx = { 0 };

typedef struct _str_buffer_t
{
	StreamBufferHandle_t handle;
	StaticStreamBuffer_t def;

	SemaphoreHandle_t sem_can_transfer_handle;
	StaticSemaphore_t sem_can_transfer_def;
}str_buffer_t;

uint8_t usb_recv_buf_storage[USB_RCVBUF_SIZE + 1];
uint8_t usb_send_buf_storage[USB_SNDBUF_SIZE + 1];

static str_buffer_t usb_recv_buf;
static str_buffer_t usb_send_buf;

static uint8_t s_tdo_bytes[1024];
static esp_chip_model_t s_target_model;
static TaskHandle_t s_task_handle = NULL;

static const char* USB_CTRL_TAG = "jtag-usbctl";
static const char* USB_RX_TAG = "jtag-rx";
static const char* USB_TX_TAG = "jtag-tx";
static const char* JTAG_TASK_TAG = "jtag";

static void jtag_pio_dma_init(void)
{
#if JTAG_RX_PUSH_THRESHOLD == 32
#define PIO_RX_DMA_SIZE DMA_SIZE_32
#elif JTAG_RX_PUSH_THRESHOLD == 16
#define PIO_RX_DMA_SIZE DMA_SIZE_16
#elif JTAG_RX_PUSH_THRESHOLD == 8
#define PIO_RX_DMA_SIZE DMA_SIZE_8
#else
#error "_JTAG_RX_PUSH_THRESHOLD must be 8, 16, or 32"
#endif

	// setup PIO jtag RX dma for TDO line
	dma_channel_config c = dma_channel_get_default_config(jtag_ctx.pio_rx_dma_channel);
	channel_config_set_transfer_data_size(&c, PIO_RX_DMA_SIZE);
	channel_config_set_write_increment(&c, true);
	channel_config_set_read_increment(&c, false);
	channel_config_set_dreq(&c, pio_get_dreq(jtag_ctx.pio, jtag_ctx.sm_rx, false));
	dma_channel_configure(
		jtag_ctx.pio_rx_dma_channel,
		&c,
		NULL,                                                                               // Write address
		((uint8_t*)&jtag_ctx.pio->rxf[jtag_ctx.sm_rx])+((32 - JTAG_RX_PUSH_THRESHOLD) / 8), // Read address
		0,                                                                                  // Transfer count
		false                                                                               // Don't start yet
		);

	dma_channel_acknowledge_irq0(jtag_ctx.pio_rx_dma_channel);
	dma_channel_set_irq0_enabled(jtag_ctx.pio_rx_dma_channel, true);
}

static inline void jtag_pio_dma_start_read(void* buf, uint trans_count)
{
	dma_channel_set_trans_count(jtag_ctx.pio_rx_dma_channel, trans_count, false);
	dma_channel_set_write_addr(jtag_ctx.pio_rx_dma_channel, buf, true);
}

static void __not_in_flash_func(jtag_pio_dma_handler)(void)
{
	BaseType_t higherPriorityTaskWoken = 0;

	if (dma_channel_get_irq0_status(jtag_ctx.pio_rx_dma_channel))
	{
		dma_channel_acknowledge_irq0(jtag_ctx.pio_rx_dma_channel);
		xTaskNotifyFromISR(s_task_handle, JTAG_PIO_DMA_RX_COMPLETE_EVENT, eSetBits, &higherPriorityTaskWoken);
		portYIELD_FROM_ISR(higherPriorityTaskWoken);
	}
}

bool tud_vendor_control_xfer_cb(const uint8_t rhport, const uint8_t stage, tusb_control_request_t const *request)
{
	// nothing to with DATA & ACK stage
	if (stage != CONTROL_STAGE_SETUP)
	{
		return true;
	}

	switch (request->bmRequestType_bit.type)
	{
	case TUSB_REQ_TYPE_VENDOR:
		ESP_LOGI(USB_CTRL_TAG,
		         "bRequest: (%d) wValue: (%d) wIndex: (%d)",
		         request->bRequest,
		         request->wValue,
		         request->wIndex);

		switch (request->bRequest)
		{
		case VEND_JTAG_SETDIV:
			if (s_task_handle == NULL)
			{
				ESP_LOGE(USB_CTRL_TAG, "can't set JTAG clock until jtag_task is fully initialized!");
				return false;
			}
			pio_set_sm_mask_enabled(jtag_ctx.pio, (1u << jtag_ctx.sm_tx) | (1u << jtag_ctx.sm_rx), false);
			jtag_ctx.pio_clkdiv = clock_get_hz(clk_sys) / (TCK_FREQ_HZ_FROM_DIV(request->wValue) * jtag_simple_cycles_per_bit);
			pio_sm_set_clkdiv(jtag_ctx.pio, jtag_ctx.sm_tx, jtag_ctx.pio_clkdiv);
			pio_sm_set_clkdiv(jtag_ctx.pio, jtag_ctx.sm_rx, jtag_ctx.pio_clkdiv);
			pio_set_sm_mask_enabled(jtag_ctx.pio, (1u << jtag_ctx.sm_tx) | (1u << jtag_ctx.sm_rx), true);
			printf("clk_div=%.02f\r\n", jtag_ctx.pio_clkdiv);
			break;
		case VEND_JTAG_SETIO:
			// TODO: process the commands
			break;
		case VEND_JTAG_GETTDO: {
			uint8_t buf = gpio_get(GPIO_TDO);
			return tud_control_xfer(rhport, request, (void *)&buf, 1);
		}
		break;
		case VEND_JTAG_SET_CHIPID:
			s_target_model = request->wValue;
		}

		// response with status OK
		return tud_control_status(rhport, request);
	}

	return false;
}

static void init_jtag_pio(void)
{
	jtag_ctx.pio = pio0;
	jtag_ctx.offset_tx = pio_add_program(jtag_ctx.pio, &jtag_simple_program);
	jtag_ctx.offset_rx = pio_add_program(jtag_ctx.pio, &jtag_tdo_slave_program);
	jtag_ctx.sm_tx = pio_claim_unused_sm(jtag_ctx.pio, true);
	jtag_ctx.sm_rx = pio_claim_unused_sm(jtag_ctx.pio, true);

	jtag_ctx.offset_rx = pio_add_program(jtag_ctx.pio, &jtag_tdo_slave_program);
	jtag_simple_program_init(jtag_ctx.pio, jtag_ctx.sm_tx, jtag_ctx.offset_tx, jtag_ctx.sm_rx, jtag_ctx.offset_rx, GPIO_TDI, GPIO_TDO, GPIO_TCK, 1000000ul);
}

// Invoked when received new data
void tud_vendor_rx_cb(uint8_t itf)
{
	xSemaphoreGive(usb_recv_buf.sem_can_transfer_handle);
}

// Invoked when last rx transfer finished
void tud_vendor_tx_cb(uint8_t itf, uint32_t sent_bytes)
{
	xSemaphoreGive(usb_send_buf.sem_can_transfer_handle);
}

static void usb_reader_task(void *pvParameters)
{
	uint8_t buf[CFG_TUD_VENDOR_EPSIZE];
	for (;;)
	{
		if (tud_vendor_n_available(0))
		{
			uint32_t r;
			while ((r = tud_vendor_n_read(0, buf, sizeof(buf))) > 0)
			{
				if (xStreamBufferSend(usb_recv_buf.handle, buf, r, pdMS_TO_TICKS(1000)) != r)
				{
					ESP_LOGE(USB_RX_TAG,
					         "Cannot write to usb_rcvbuf ringbuffer (free %d of %d)!",
					         xStreamBufferSpacesAvailable(usb_recv_buf.handle),
					         USB_RCVBUF_SIZE);
					eub_abort();
				}
			}

			if (xStreamBufferSpacesAvailable(usb_recv_buf.handle) < 0.25 * USB_RCVBUF_SIZE)
			{
				ESP_LOGW(USB_RX_TAG, "Ringbuffer is getting full!");
				vTaskDelay(pdMS_TO_TICKS(1000));
			}
		}
		else
		{
			xSemaphoreTake(usb_recv_buf.sem_can_transfer_handle, portMAX_DELAY);
		}
	}
	vTaskDelete(NULL);
}

static void usb_writer_task(void *pvParameters)
{
	uint8_t local_buf[CFG_TUD_VENDOR_EPSIZE];
	for (;;)
	{

		size_t n = xStreamBufferReceive(usb_send_buf.handle, local_buf, sizeof(local_buf), portMAX_DELAY);
		if (!tud_mounted())
		{
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;
		}
		uint8_t *buf = local_buf;
		ESP_LOGD(USB_TX_TAG, "%d bytes", n);
		for (int transferred = 0, to_send = n; transferred < n;)
		{
			int space;
			while ((space = tud_vendor_n_write_available(0)) == 0)
			{
				xSemaphoreTake(usb_send_buf.sem_can_transfer_handle, portMAX_DELAY);
			}
			
			const int sent = tud_vendor_n_write(0, local_buf + transferred, MIN(space, to_send));
			transferred += sent;
			to_send -= sent;
			// there seems to be no flush for vendor class
			tud_vendor_n_flush(0);
		}
	}
	vTaskDelete(NULL);
}

static int usb_send(const uint8_t *buf, const int size)
{
	if (xStreamBufferSend(usb_send_buf.handle, buf, size, pdMS_TO_TICKS(1000)) != size)
	{
		ESP_LOGE(USB_TX_TAG,
		         "Out of space! (free %d of %d)!",
		         xStreamBufferSpacesAvailable(usb_send_buf.handle),
		         USB_SNDBUF_SIZE);
		return 0;
	}
	return size;
}

inline static void jtag_transfer(uint8_t dat, uint repeat_cnt)
{
	if (repeat_cnt == 0) return;

	uint pio_tx_cmd = (dat << 12) | ((repeat_cnt - 1) << 1);
	uint32_t notify_value;
	if (IS_TDO(dat))
	{
		uint dma_trans_count = (repeat_cnt + jtag_ctx.pio_rx_bits_cached) / JTAG_RX_PUSH_THRESHOLD;
		jtag_ctx.pio_rx_bits_cached = (repeat_cnt + jtag_ctx.pio_rx_bits_cached) % JTAG_RX_PUSH_THRESHOLD;

		if (dma_trans_count)
		{
			if (dma_channel_is_busy(jtag_ctx.pio_rx_dma_channel))
			{
				xTaskNotifyWait(0xFFFFFFFF, 0, &notify_value, 0);
				if (dma_channel_is_busy(jtag_ctx.pio_rx_dma_channel))
					xTaskNotifyWait(0, JTAG_PIO_DMA_RX_COMPLETE_EVENT, &notify_value, portMAX_DELAY);
			}
			jtag_pio_dma_start_read(&s_tdo_bytes[jtag_ctx.tdo_bits_total / 8], dma_trans_count);
			jtag_ctx.tdo_bits_total += (dma_trans_count * JTAG_RX_PUSH_THRESHOLD);
			pio_sm_put_blocking(jtag_ctx.pio, jtag_ctx.sm_tx, pio_tx_cmd);
		}
		else
		{
			pio_sm_put_blocking(jtag_ctx.pio, jtag_ctx.sm_tx, pio_tx_cmd);
		}
	}
	else
	{
		pio_sm_put_blocking(jtag_ctx.pio, jtag_ctx.sm_tx, pio_tx_cmd);
	}

}

inline static void jtag_flush(void)
{
	uint32_t notify_value;
	if (dma_channel_is_busy(jtag_ctx.pio_rx_dma_channel))
	{
		xTaskNotifyWait(0xFFFFFFFF, 0, &notify_value, 0);
		if (dma_channel_is_busy(jtag_ctx.pio_rx_dma_channel))
			xTaskNotifyWait(0, JTAG_PIO_DMA_RX_COMPLETE_EVENT, &notify_value, portMAX_DELAY);
	}

	if (jtag_ctx.pio_rx_bits_cached)
	{
		uint flush_count = (JTAG_RX_PUSH_THRESHOLD - jtag_ctx.pio_rx_bits_cached);
		uint pio_tx_cmd =  ((flush_count - 1) << 1) | 0x01;

		pio_sm_put_blocking(jtag_ctx.pio, jtag_ctx.sm_tx, pio_tx_cmd);
		uint tdo = pio_sm_get_blocking(jtag_ctx.pio, jtag_ctx.sm_rx);
		tdo >>= (32 - JTAG_RX_PUSH_THRESHOLD);
		while (jtag_ctx.pio_rx_bits_cached > 0)
		{
			if ((jtag_ctx.tdo_bits_total & (~(JTAG_RX_PUSH_THRESHOLD - 1))) == 0)
				s_tdo_bytes[(jtag_ctx.tdo_bits_total) / 8] = 0;

			uint8_t bitmask = 1u << ((jtag_ctx.tdo_bits_total) % 8);
			if (tdo & 1)
				s_tdo_bytes[(jtag_ctx.tdo_bits_total) / 8] |= bitmask;
			else
				s_tdo_bytes[(jtag_ctx.tdo_bits_total) / 8] &= ~bitmask;
			jtag_ctx.tdo_bits_total++;
			tdo >>= 1;
			jtag_ctx.pio_rx_bits_cached--;
		}
	}
}

int jtag_get_proto_caps(uint16_t *dest)
{
	memcpy(dest, (uint16_t *)&jtag_proto_caps, sizeof(jtag_proto_caps));
	return sizeof(jtag_proto_caps);
}

int jtag_get_target_model(void)
{
	return s_target_model;
}

void jtag_task_suspend(void)
{
	if (s_task_handle)
	{
		vTaskSuspend(s_task_handle);
	}
}

void jtag_task_resume(void)
{
	if (s_task_handle)
	{
		vTaskResume(s_task_handle);
	}
}

void __not_in_flash_func(jtag_task)(void *pvParameters)
{
	static uint8_t nibbles[64];

	jtag_ctx.pio_rx_dma_channel = dma_claim_unused_channel(true);
	jtag_ctx.pio_rx_bits_cached = 0;
	jtag_ctx.tdo_bits_sent = 0;
	jtag_ctx.tdo_bits_total = 0;

	jtag_ctx.pio_rx_dma_channel = dma_claim_unused_channel(true);
	init_jtag_pio();
	jtag_pio_dma_init();
	irq_add_shared_handler(DMA_IRQ_0, jtag_pio_dma_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
	irq_set_enabled(DMA_IRQ_0, true);
	s_task_handle = xTaskGetCurrentTaskHandle();

	memset(s_tdo_bytes, 0x00, sizeof(s_tdo_bytes));

	// create stream buffers
	usb_recv_buf.handle = xStreamBufferGenericCreateStatic(sizeof(usb_recv_buf_storage), 1, pdFALSE, usb_recv_buf_storage, &usb_recv_buf.def);
	usb_send_buf.handle = xStreamBufferGenericCreateStatic(sizeof(usb_send_buf_storage), 1, pdFALSE, usb_send_buf_storage, &usb_send_buf.def);

	// create transfer semapthores
	usb_recv_buf.sem_can_transfer_handle = xSemaphoreCreateBinaryStatic(&usb_recv_buf.sem_can_transfer_def);
	usb_send_buf.sem_can_transfer_handle = xSemaphoreCreateBinaryStatic(&usb_send_buf.sem_can_transfer_def);

	xSemaphoreTake(usb_recv_buf.sem_can_transfer_handle, 0);
	xSemaphoreTake(usb_send_buf.sem_can_transfer_handle, 0);


	if (xTaskCreateAffinitySet(usb_reader_task, "usb_reader_task", STACK_SIZE_FROM_BYTES(4 * 1024), NULL, uxTaskPriorityGet(NULL) - 1, CORE_AFFINITY_JTAG_TASK, NULL) != pdPASS)
	{
		ESP_LOGE(JTAG_TASK_TAG, "Cannot create USB reader task!");
		eub_abort();
	}

	if (xTaskCreateAffinitySet(usb_writer_task, "usb_send_task", STACK_SIZE_FROM_BYTES(4 * 1024), NULL, uxTaskPriorityGet(NULL) + 1, CORE_AFFINITY_JTAG_TASK, NULL) != pdPASS)
	{
		ESP_LOGE(JTAG_TASK_TAG, "Cannot create USB send task!");
		eub_abort();
	}

	enum e_cmds
	{
		CMD_CLK_0 = 0,
		CMD_CLK_1,
		CMD_CLK_2,
		CMD_CLK_3,
		CMD_CLK_4,
		CMD_CLK_5,
		CMD_CLK_6,
		CMD_CLK_7,
		CMD_SRST0,
		CMD_SRST1,
		CMD_FLUSH,
		CMD_RSV,
		CMD_REP0,
		CMD_REP1,
		CMD_REP2,
		CMD_REP3
	};

	uint8_t pin_levels[] =
	{
		MAKE_DAT(0, 0, 0),               //CMD_CLK_0
		MAKE_DAT(0, 0, 1),               //CMD_CLK_1
		MAKE_DAT(0, 1, 0),               //..
		MAKE_DAT(0, 1, 1),
		MAKE_DAT(1, 0, 0),
		MAKE_DAT(1, 0, 1),
		MAKE_DAT(1, 1, 0),
		MAKE_DAT(1, 1, 1),               //CMD_CLK_7
		MAKE_DAT(0, 1, 0),               //CMD_SRST0
		MAKE_DAT(0, 1, 0),               //CMD_SRST1
	};
	size_t cnt = 0;
	int prev_cmd = CMD_SRST0, rep_cnt = 0;

	while (1)
	{
		bool was_reset = false;
		cnt = xStreamBufferReceive(usb_recv_buf.handle, nibbles, sizeof(nibbles), portMAX_DELAY);

		for (size_t n = 0; n < cnt * 2; n++)
		{
			const int cmd = (n & 1) ? (nibbles[n / 2] & 0x0F) : (nibbles[n / 2] >> 4);
			int cmd_exec = cmd, cmd_rpt_cnt = 1;

			switch (cmd)
			{
			case CMD_REP0:
			case CMD_REP1:
			case CMD_REP2:
			case CMD_REP3:
				//(r1*2+r0)<<(2*n)
				cmd_rpt_cnt = (cmd - CMD_REP0) << (2 * rep_cnt++);
				cmd_exec = prev_cmd;
				break;
			case CMD_SRST0:          // JTAG Tap reset command is not expected from host but still we are ready
				cmd_rpt_cnt = 8; // 8 TMS=1 is more than enough to return the TAP state to RESET
				was_reset = true;
				break;
			case CMD_SRST1:          // FIXME: system reset may cause an issue during openocd examination
				cmd_rpt_cnt = 8; // for now this is also used for the tap reset
				was_reset = true;
				// gpio_put(GPIO_RST, 0);
				// ets_delay_us(100);
				// gpio_put(GPIO_RST, 1);
				break;
			default:
				rep_cnt = 0;
				break;
			}
			if (cmd_exec < CMD_FLUSH)
			{
				jtag_transfer(pin_levels[cmd_exec], cmd_rpt_cnt);
			}
			else if (cmd_exec == CMD_FLUSH)
			{
				jtag_flush();
				jtag_ctx.tdo_bits_total = ROUND_UP_BITS(jtag_ctx.tdo_bits_total);
				if (jtag_ctx.tdo_bits_sent < jtag_ctx.tdo_bits_total)
				{
					int waiting_to_send_bits = jtag_ctx.tdo_bits_total - jtag_ctx.tdo_bits_sent;
					while (waiting_to_send_bits > 0)
					{
						int send_bits = waiting_to_send_bits > JTAG_PROTO_MAX_BITS ? JTAG_PROTO_MAX_BITS : waiting_to_send_bits;
						usb_send(s_tdo_bytes + (jtag_ctx.tdo_bits_sent / 8), send_bits / 8);
						jtag_ctx.tdo_bits_sent += send_bits;
						waiting_to_send_bits -= send_bits;
					}
					jtag_ctx.tdo_bits_total = jtag_ctx.tdo_bits_sent = 0;
				}
			}

			/* As soon as either 64 bytes (512 bits) have been collected or a CMD_FLUSH command is executed,
			        make the usb buffer available for the host to receive.
			 */
			int waiting_to_send_bits = jtag_ctx.tdo_bits_total - jtag_ctx.tdo_bits_sent;
			if (waiting_to_send_bits >= JTAG_PROTO_MAX_BITS)
			{
				if (dma_channel_is_busy(jtag_ctx.pio_rx_dma_channel))
				{
					uint32_t notify_value;
					xTaskNotifyWait(0xFFFFFFFF, 0, &notify_value, 0);
					if (dma_channel_is_busy(jtag_ctx.pio_rx_dma_channel))
						xTaskNotifyWait(0, JTAG_PIO_DMA_RX_COMPLETE_EVENT, &notify_value, portMAX_DELAY);
				}
				int send_bits = JTAG_PROTO_MAX_BITS;
				int n_byte = send_bits / 8;
				usb_send(s_tdo_bytes + (jtag_ctx.tdo_bits_sent / 8), n_byte);
				jtag_ctx.tdo_bits_sent += send_bits;
				waiting_to_send_bits -= send_bits;
				if (waiting_to_send_bits <= 0)
				{
					jtag_ctx.tdo_bits_total = jtag_ctx.tdo_bits_sent = 0;
				}
			}

			if (cmd < CMD_REP0 && cmd != CMD_FLUSH)
			{
				prev_cmd = cmd;
			}
		}
		ESP_LOGD(JTAG_TASK_TAG, "%d bytes", cnt);
		if (was_reset)
		{
			ws2812_set_rgb_state(RGB_LED_STATE_END);
		}
		else
		{
			ws2812_set_rgb_state(RGB_LED_STATE_JTAG);
		}
	}

	vTaskDelete(NULL);
}
