/* Copyright 2020 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include "rp2040_port.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/time.h"
#include "esp_log.h"
#include <unistd.h>
#include <string.h>

// #define SERIAL_DEBUG_ENABLE

#ifdef SERIAL_DEBUG_ENABLE

static void dec_to_hex_str(const uint8_t dec, uint8_t hex_str[3])
{
	static const uint8_t dec_to_hex[] = {
		'0', '1', '2', '3', '4', '5', '6', '7',
		'8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
	};

	hex_str[0] = dec_to_hex[dec >> 4];
	hex_str[1] = dec_to_hex[dec & 0xF];
	hex_str[2] = '\0';
}

static void serial_debug_print(const uint8_t *data, uint16_t size, bool write)
{
	static bool write_prev = false;
	uint8_t hex_str[3];

	if(write_prev != write)
	{
		write_prev = write;
		printf("\n--- %s ---\n", write ? "WRITE" : "READ");
	}

	for(uint32_t i = 0; i < size; i++)
	{
		dec_to_hex_str(data[i], hex_str);
		printf("%s ", hex_str);
	}
}

#else

static void serial_debug_print(const uint8_t *data, uint16_t size, bool write) { }

#endif

static absolute_time_t s_time_end;

static loader_rp2040_config_t loader_config;

esp_loader_error_t loader_port_rp2040_init(const loader_rp2040_config_t *config)
{
	memcpy(&loader_config, config, sizeof(loader_config));

	return ESP_LOADER_SUCCESS;
}

void loader_port_esp32_deinit(void)
{}


esp_loader_error_t loader_port_serial_write(const uint8_t *data, uint16_t size, uint32_t timeout)
{
	serial_debug_print(data, size, true);

	if (loader_config.write_uart(data, size, timeout) != size)
		return ESP_LOADER_ERROR_FAIL;
	return ESP_LOADER_SUCCESS;
}


esp_loader_error_t loader_port_serial_read(uint8_t *data, uint16_t size, uint32_t timeout)
{
	int read = loader_config.read_uart(data, size, timeout);

	serial_debug_print(data, read, false);

	if (read < 0)
	{
		return ESP_LOADER_ERROR_FAIL;
	}
	else if (read < size)
	{
		return ESP_LOADER_ERROR_TIMEOUT;
	}
	else
	{
		return ESP_LOADER_SUCCESS;
	}
}


// Set GPIO0 LOW, then
// assert reset pin for 50 milliseconds.
void loader_port_enter_bootloader(void)
{
	loader_config.set_boot_pin(false);
	loader_port_reset_target();
	loader_port_delay_ms(50);
	loader_config.set_boot_pin(true);
}


void loader_port_reset_target(void)
{
	loader_config.set_rst_pin(false);
	loader_port_delay_ms(50);
	loader_config.set_rst_pin(true);
}


void loader_port_delay_ms(uint32_t ms)
{
	sleep_ms(ms);
}


void loader_port_start_timer(uint32_t ms)
{
	s_time_end = make_timeout_time_ms(ms);
}


uint32_t loader_port_remaining_time(void)
{
	int64_t remaining = absolute_time_diff_us(get_absolute_time(), s_time_end) / 1000;
	return (remaining > 0) ? (uint32_t)remaining : 0;
}


void loader_port_debug_print(const char *str)
{
	printf("DEBUG: %s\n", str);
}

esp_loader_error_t loader_port_change_baudrate(uint32_t baudrate)
{
	uint32_t actual_baud_rate = loader_config.set_baud_rate(baudrate);
	return actual_baud_rate > 0 ? ESP_LOADER_SUCCESS : ESP_LOADER_ERROR_FAIL;
}