// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// ----------- //
// jtag_simple //
// ----------- //

#define jtag_simple_wrap_target 0
#define jtag_simple_wrap 17

#define jtag_simple_cycles_per_bit 10

static const uint16_t jtag_simple_program_instructions[] = {
	//     .wrap_target
	0x90a0,                  //  0: pull   block           side 0     @@@E:\GitHub\esp-usb-bridge-pico\jtag.pio:19
	0x6021,                  //  1: out    x, 1                       @@@E:\GitHub\esp-usb-bridge-pico\jtag.pio:21
	0x0027,                  //  2: jmp    !x, 7                      @@@E:\GitHub\esp-usb-bridge-pico\jtag.pio:22
	0x602f,                  //  3: out    x, 15                      @@@E:\GitHub\esp-usb-bridge-pico\jtag.pio:24
	0xc424,                  //  4: irq    wait 4                 [4] @@@E:\GitHub\esp-usb-bridge-pico\jtag.pio:27
	0x0044,                  //  5: jmp    x--, 4                     @@@E:\GitHub\esp-usb-bridge-pico\jtag.pio:28
	0x0000,                  //  6: jmp    0                          @@@E:\GitHub\esp-usb-bridge-pico\jtag.pio:29
	0x602b,                  //  7: out    x, 11                      @@@E:\GitHub\esp-usb-bridge-pico\jtag.pio:32
	0x6002,                  //  8: out    pins, 2                    @@@E:\GitHub\esp-usb-bridge-pico\jtag.pio:34
	0x6041,                  //  9: out    y, 1                       @@@E:\GitHub\esp-usb-bridge-pico\jtag.pio:36
	0x186d,                  // 10: jmp    !y, 13          side 1     @@@E:\GitHub\esp-usb-bridge-pico\jtag.pio:39
	0xc004,                  // 11: irq    nowait 4                   @@@E:\GitHub\esp-usb-bridge-pico\jtag.pio:40
	0x000e,                  // 12: jmp    14                         @@@E:\GitHub\esp-usb-bridge-pico\jtag.pio:41
	0xa142,                  // 13: nop                           [1] @@@E:\GitHub\esp-usb-bridge-pico\jtag.pio:43
	0x0120,                  // 14: jmp    !x, 0                  [1] @@@E:\GitHub\esp-usb-bridge-pico\jtag.pio:48
	0xb042,                  // 15: nop                    side 0     @@@E:\GitHub\esp-usb-bridge-pico\jtag.pio:52
	0xa242,                  // 16: nop                           [2] @@@E:\GitHub\esp-usb-bridge-pico\jtag.pio:53
	0x004a,                  // 17: jmp    x--, 10                    @@@E:\GitHub\esp-usb-bridge-pico\jtag.pio:56
	                         //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program jtag_simple_program = {
	.instructions = jtag_simple_program_instructions,
	.length = 18,
	.origin = -1,
};

static inline pio_sm_config jtag_simple_program_get_default_config(uint offset) {
	pio_sm_config c = pio_get_default_sm_config();
	sm_config_set_wrap(&c, offset + jtag_simple_wrap_target, offset + jtag_simple_wrap);
	sm_config_set_sideset(&c, 2, true, false);
	return c;
}
#endif

// -------------- //
// jtag_tdo_slave //
// -------------- //

#define jtag_tdo_slave_wrap_target 0
#define jtag_tdo_slave_wrap 1

static const uint16_t jtag_tdo_slave_program_instructions[] = {
	//     .wrap_target
	0x20c4,                  //  0: wait   1 irq, 4                   @@@E:\GitHub\esp-usb-bridge-pico\jtag.pio:61
	0x4001,                  //  1: in     pins, 1                    @@@E:\GitHub\esp-usb-bridge-pico\jtag.pio:62
	                         //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program jtag_tdo_slave_program = {
	.instructions = jtag_tdo_slave_program_instructions,
	.length = 2,
	.origin = -1,
};

static inline pio_sm_config jtag_tdo_slave_program_get_default_config(uint offset) {
	pio_sm_config c = pio_get_default_sm_config();
	sm_config_set_wrap(&c, offset + jtag_tdo_slave_wrap_target, offset + jtag_tdo_slave_wrap);
	return c;
}

#include "hardware/clocks.h"
#define JTAG_RX_PUSH_THRESHOLD (8)
//; 3-bit data:
//; BIT  0   = TDI out			(out pin 0)
//; BIT  1   = TMS out			(set pin 0)
//; BIT  2   = TDO requested	(in pin 0)
//;            TCK              (side pin 0)
static inline void jtag_simple_program_init(PIO pio, uint sm, uint offset,  uint slave_sm, uint slave_offset, uint tdi_tms, uint tdo, uint tck, float freq)
{
	const uint tdi = tdi_tms;
	const uint tms = tdi_tms + 1;
	// assign pins to the PIO module
	pio_gpio_init(pio, tdi); // TMS
	pio_gpio_init(pio, tms); // TDI
	//pio_gpio_init(pio, tdo); // TCK
	pio_gpio_init(pio, tck); // TDO
	pio_sm_config c = jtag_simple_program_get_default_config(offset);
	// set pin bases
	sm_config_set_out_pins(&c, tdi_tms, 2);
	//sm_config_set_in_pins(&c, tdo);
	sm_config_set_sideset_pins(&c, tck);
	// TX FIFOs Working with 3bits at a time
	sm_config_set_out_shift(&c, true, false, 32);
	// RX FIFOs Working with 8bits
	//sm_config_set_in_shift(&c, false, false, 32);
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
	float div = clock_get_hz(clk_sys) / (freq * jtag_simple_cycles_per_bit);
	sm_config_set_clkdiv(&c, div);
	// set initial pin levels: TDI=0, TMS=1, TCK=0
	pio_sm_set_pins_with_mask(pio, sm, (1u << (tms)), (1u << (tdi)) | (1u << (tms)) | (1u << (tck)));
	// set pin directions: OUTPUT=tdi,tms,tck   INPUT=tdo
	pio_sm_set_pindirs_with_mask(pio, sm, (1u << (tdi)) | (1u << (tms)) | (1u << (tck)), (1u << (tdi)) | (1u << (tms)) | (1u << (tck)) | (1u << (tdo)));
	//hw_set_bits(&pio->input_sync_bypass, 1u << tdo);
	pio_sm_init(pio, sm, offset, &c);
	// tdo rx slave sm
	c = jtag_tdo_slave_program_get_default_config(slave_offset);
	sm_config_set_in_pins(&c, tdo);
	sm_config_set_in_shift(&c, true, true, JTAG_RX_PUSH_THRESHOLD);
	sm_config_set_clkdiv(&c, div);
	sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
	pio_sm_init(pio, slave_sm, slave_offset, &c);
	pio_set_sm_mask_enabled(pio, (1u << sm) | (1u << slave_sm), true);
}

#endif

