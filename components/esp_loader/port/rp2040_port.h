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

 #pragma once
#include <stdint.h>
#include "serial_io.h"
#include "FreeRTOS.h"
#include "queue.h"

typedef void (*loader_set_pin_level_t)(bool level);
typedef int32_t (*loader_uart_write_t)(const uint8_t* buf, uint32_t len, uint32_t timeout_ms);
typedef int32_t (*loader_uart_read_t)(uint8_t* buf, uint32_t len, uint32_t timeout_ms);
typedef uint32_t (*loader_uart_set_baudrate_t)(uint32_t baud_rate);

typedef struct
{
	loader_set_pin_level_t set_boot_pin;
	loader_set_pin_level_t set_rst_pin;
	loader_uart_read_t read_uart;
	loader_uart_write_t write_uart;
	loader_uart_set_baudrate_t set_baud_rate;
} loader_rp2040_config_t;

/**
 * @brief Initializes serial interface.
 *
 * @param baud_rate[in]       Communication speed.
 *
 * @return
 *     - ESP_LOADER_SUCCESS Success
 *     - ESP_LOADER_ERROR_FAIL Initialization failure
 */
esp_loader_error_t loader_port_rp2040_init(const loader_rp2040_config_t *config);

/**
 * @brief Deinitialize serial interface.
 */
void loader_port_rp2040_deinit(void);