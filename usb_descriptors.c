/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "pico.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "pico/unique_id.h"
#include "jtag.h"
#include "ubp_config.h"

#define EPNUM_CDC       2
#define EPNUM_VENDOR    3
#define EPNUM_MSC       4


#if MSC_ENABLED
#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_VENDOR_DESC_LEN + TUD_MSC_DESC_LEN)
#else
#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_VENDOR_DESC_LEN)
#endif

enum {
	ITF_NUM_CDC = 0,
	ITF_NUM_CDC_DATA,
	ITF_NUM_VENDOR,
#if MSC_ENABLED
	ITF_NUM_MSC,
#endif
	ITF_NUM_TOTAL
};

static const tusb_desc_device_t descriptor_config = {
	.bLength = sizeof(descriptor_config),
	.bDescriptorType = TUSB_DESC_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = TUSB_CLASS_MISC,
	.bDeviceSubClass = MISC_SUBCLASS_COMMON,
	.bDeviceProtocol = MISC_PROTOCOL_IAD,
#ifdef CFG_TUD_ENDPOINT0_SIZE
	.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
#else  // earlier versions have a typo in the name
	.bMaxPacketSize0 = CFG_TUD_ENDOINT0_SIZE,
#endif
	.idVendor = CONFIG_BRIDGE_USB_VID,
	.idProduct = CONFIG_BRIDGE_USB_PID,
	.bcdDevice = 0x100,
	.iManufacturer = 0x01,
	.iProduct = 0x02,
	.iSerialNumber = 0x03,
	.bNumConfigurations = 0x01
};

/*
    ESP usb builtin jtag subclass and protocol is 0xFF and 0x01 respectively.
    However, Tinyusb default values are 0x00.
    In order to use same protocol without tinyusb customization we are re-defining
    vendor descriptor here.
 */
// Interface number, string index, EP Out & IN address, EP size
#define TUD_VENDOR_EUB_DESCRIPTOR(_itfnum, _stridx, _epout, _epin, _epsize) \
	/* Interface */ \
	9, TUSB_DESC_INTERFACE, _itfnum, 0, 2, TUSB_CLASS_VENDOR_SPECIFIC, 0xFF, 0x01, _stridx, \
	/* Endpoint Out */ \
	7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0, \
	/* Endpoint In */ \
	7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 0

static uint8_t const desc_configuration[] = {
	// config number, interface count, string index, total length, attribute, power in mA
	TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

	// Interface number, string index, EP notification address and size, EP data address (out, in) and size.
	TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, 0x81, 8, EPNUM_CDC, 0x80 | EPNUM_CDC, TUD_OPT_HIGH_SPEED ? 512 : 64),

	// Interface number, string index, EP Out & IN address, EP size
	TUD_VENDOR_EUB_DESCRIPTOR(ITF_NUM_VENDOR, 5, EPNUM_VENDOR, 0x80 | EPNUM_VENDOR, 64),

#if MSC_ENABLED
	// Interface number, string index, EP Out & EP In address, EP size
	TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 6, EPNUM_MSC, 0x80 | EPNUM_MSC, 64),
#endif
};

static char serial_descriptor[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1] = { '\0' }; // 2 chars per hexnumber + '\0'

static char const *string_desc_arr[] = {
	(const char[]) { 0x09, 0x04 },                                             // 0: is supported language is English (0x0409)
	CONFIG_BRIDGE_MANUFACTURER,                                                // 1: Manufacturer
	CONFIG_BRIDGE_PRODUCT_NAME,                                                // 2: Product
	serial_descriptor,                                                         // 3: Serials
	"CDC",
	"JTAG",
	"MSC",

	/* JTAG_STR_DESC_INX 0x0A */
};

static uint16_t _desc_str[32];

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
	return desc_configuration;
}

uint8_t const *tud_descriptor_device_cb(void)
{
	return (uint8_t const *) &descriptor_config;
}

void init_serial_no(void)
{

	pico_get_unique_board_id_string(serial_descriptor, sizeof(serial_descriptor) - 1);
}

uint16_t const *tud_descriptor_string_cb(const uint8_t index, const uint16_t langid)
{
	uint8_t chr_count;

	if (index == 0)
	{
		memcpy(&_desc_str[1], string_desc_arr[0], 2);
		chr_count = 1;
	}
#if JTAG_ENABLED
	else if (index == JTAG_STR_DESC_INX)
	{
		chr_count = jtag_get_proto_caps(&_desc_str[1]) / 2;
	}
#endif
	else
	{
		// Convert ASCII string into UTF-16

		if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0])))
		{
			return NULL;
		}

		const char *str = string_desc_arr[index];

		// Cap at max char
		chr_count = strlen(str);
		if (chr_count > 31)
		{
			chr_count = 31;
		}

		for (uint8_t i = 0; i < chr_count; i++)
		{
			_desc_str[1 + i] = str[i];
		}
	}

	// first byte is length (including header), second byte is string type
	_desc_str[0] = (TUSB_DESC_STRING << 8 ) | (2 * chr_count + 2);

	return _desc_str;
}
