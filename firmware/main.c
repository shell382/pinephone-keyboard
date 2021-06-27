/**
 * Pinephone Keyboard Firmware
 *
 * Copyright (C) 2021  Ondřej Jirman <megi@xff.cz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <string.h>
#include <em85f684a.h>

#ifndef CONFIG_FLASH_ENABLE
#define CONFIG_FLASH_ENABLE 1
#endif
#ifndef CONFIG_DEBUG_LOG
#define CONFIG_DEBUG_LOG 1
#endif
#ifndef CONFIG_USB_STACK
#define CONFIG_USB_STACK 1
#endif
#ifndef CONFIG_SELFTEST
#define CONFIG_SELFTEST 1
#endif
#ifndef CONFIG_STOCK_FW
#define CONFIG_STOCK_FW 1
#endif

#define USB_DEBUG 0

#define BIT(n) (1u << (n))

// {{{ Debug logging

#if CONFIG_DEBUG_LOG

// debug logging needs to have all variables volatile, since they
// can be accessed from interrupts
//
// any access to these variables has to happen with interrupts disabled

static volatile uint8_t __xdata log_buffer[1024];
// end = start => empty buffer
// end can never equal start on a filled buffer
// end points to the last char if end != start
static volatile uint16_t log_start = 0;
static volatile uint16_t log_end = 0;

// putc needs to disable interrupts (thus it's marked __critical)
// it's possible to call putc in any context
static void putc(char c) __critical
{
	log_end = (log_end + 1) % 1024;

	if (log_end == log_start) {
		// overflow, just push the start in front of us
		log_start = (log_start + 1) % 1024;
	}

	log_buffer[log_end] = c;
}

static void puts(const char* s)
{
	while (*s)
		putc(*s++);
}

static void put_uint(uint16_t value)
{
	char buf[6];
	char *p = &buf[6 - 1];

	*p = '\0';

	if (!value)
		*--p = '0';

	while (value) {
		*--p = '0' + value % 10;
		value /= 10;
	}

	puts(p);
}

static void put_hex_n(uint8_t nibble)
{
	char c;

	nibble &= 0xf;

	if (nibble < 10)
		c = '0' + nibble;
	else
		c = 'a' + (nibble - 10);

	putc(c);
}

static void put_hex_b(uint8_t hex)
{
	put_hex_n(hex >> 4);
	put_hex_n(hex);
}

static void put_hex_w(uint16_t hex)
{
	put_hex_b(hex >> 8);
	put_hex_b(hex);
}

#else

#define putc(a)
#define puts(a)
#define put_hex_b(a)
#define put_hex_w(a)
#define put_hex_n(a)
#define put_uint(a)

#endif

// }}}
// {{{ Global flags

static __bit jump_to_usb_bootloader = 0;

// }}}
// {{{ Interrupt forwarding

#if CONFIG_STOCK_FW

// Stock firmware translates all interrupts to inerrupt vectors at 0x4000
// if IRAM location 0xff contains 0. Otherwise, it call interrupt handlers
// in stock FW.

#pragma noiv

uint8_t __idata __at(0xff) stock_flag = 1;

//XXX: check if we want to jump to user FW

// cleanup after stock firmware and jump to user firmware
static void jmp_to_user_fw(void) __naked
{
	PAGESW = 0;

	// disable all interrupts
	IE = 0;
	P0_EIE1 = 0;
	P0_EIE2 = 0;
	P0_EIE3 = 0;

	// disable timers
	TCON = 0;
	TMOD = 0;
	PSW1 = 0;

	// disable I2C B
	P0_I2CBCR1 = 0x00;
	P0_I2CBCR2 = 0x20;
	P0_I2CBINT = 0;

	// disable watchdog
	P0_WDTCR = 0x07; // disable watchdog ~1s
	P0_WDTKEY = 0xb1; // disable watchdog

	// reset powerdown/reset registers
	P0_DEVPD1 = 0;
	P0_DEVPD2 = 0;
	P0_DEVPD3 = 0;
	P0_PRST = 0;

	// disable USB and clear irq flags
	PAGESW = 1;
	P1_PHYTEST0 &= ~BIT(6); // phy disable
	P1_UDCCTRL &= ~BIT(6); // udc disable
	P1_UDCINT0STA = 0;
	P1_UDCINT1STA = 0;
	P1_UDCINT2STA = 0;
	P1_UDCINT0EN = 0;
	P1_UDCINT1EN = 0;
	P1_UDCINT2EN = 0;

	// disable pullups, set all pins to input
	P1_PHCON2 = 0;
	P1_P9M0 = 0xffu;
	PAGESW = 0;
	P0_PHCON0 = 0;
	P0_PHCON1 = 0;
	P0_P6M0 = 0xffu;
	P0_P8M0 = 0xffu;
	P0_ICEN = 0;

	stock_flag = 0;
	__asm__ ("ljmp 0x4000");
}

#endif

// }}}
// {{{ Original USB bootloader integration

static void usb_bootloader_jump(void) __naked
{
	PAGESW = 0;

	// disable all interrupts
	IE = 0;
	P0_EIE1 = 0;
	P0_EIE2 = 0;
	P0_EIE3 = 0;

	// disable timers
	TCON = 0;
	TMOD = 0;
	PSW1 = 0;

	// disable I2C B
	P0_I2CBCR1 = 0x00;
	P0_I2CBCR2 = 0x20;
	P0_I2CBINT = 0;

	// disable watchdog
	P0_WDTCR = 0x07; // disable watchdog ~1s
	P0_WDTKEY = 0xb1; // disable watchdog

	// reset powerdown/reset registers
	P0_DEVPD1 = 0;
	P0_DEVPD2 = 0;
	P0_DEVPD3 = 0;
	P0_PRST = 0;

	// disable USB and clear irq flags
	PAGESW = 1;
	P1_PHYTEST0 &= ~BIT(6); // phy disable
	P1_UDCCTRL &= ~BIT(6); // udc disable
	P1_UDCINT0STA = 0;
	P1_UDCINT1STA = 0;
	P1_UDCINT2STA = 0;
	P1_UDCINT0EN = 0;
	P1_UDCINT1EN = 0;
	P1_UDCINT2EN = 0;

	// disable pullups, set all pins to input
	P1_PHCON2 = 0;
	P1_P9M0 = 0xffu;
	PAGESW = 0;
	P0_PHCON0 = 0;
	P0_PHCON1 = 0;
	P0_P6M0 = 0xffu;
	P0_P8M0 = 0xffu;
	P0_ICEN = 0;

	__asm__("mov r6,#0x5a");
	__asm__("mov r7,#0xe7");
	__asm__("ljmp 0x0118");
}

// }}}
// {{{ Timers/delays

// timers clock is 2 MHz so we need to wait for 2000 ticks to get delay of 1ms
#define T0_SET_TIMEOUT(n) { \
	TL0 = 0x00; \
	TH0 = (0x10000u - n) >> 8; \
	TL0 = (0x10000u - n) & 0xff; \
	}

#define T1_SET_TIMEOUT(n) { \
	TL1 = 0x00; \
	TH1 = (0x10000u - n) >> 8; \
	TL1 = (0x10000u - n) & 0xff; \
	}

#define delay_us(n) { \
	TL0 = 0x00; \
	TF0 = 0; \
	TH0 = (0x10000u - 2 * n) >> 8; \
	TL0 = (0x10000u - 2 * n) & 0xff; \
	while (!TF0); \
}

static volatile __bit run_timed_tasks = 0;

// we use this interrupt as a scheduling tick (wakeup from sleep)

void timer1_interrupt(void) __interrupt(IRQ_TIMER1) __using(1)
{
	run_timed_tasks = 1;

	// 20 ms
	T1_SET_TIMEOUT(40000);

	TF1 = 0;
}

// }}}
// {{{ GPIO change interrupt

// we use this interrupt for wakeup from sleep on input change on port 6

static volatile __bit p6_changed = 0;

void pinchange_interrupt(void) __interrupt(IRQ_PINCHANGE) __using(1)
{
	uint8_t saved_page = PAGESW;

	PAGESW = 0;

	// change flag
	if (P0_ICEN & BIT(1)) {
		p6_changed = 1;
	}

	// disable port 6 change detection
	P0_ICEN = 0;
	ICIE = 0;

	PAGESW = saved_page;
}

// }}}
// {{{ Key scanning

static __bit scan_active = 0;

// Keyboard has 12 columns and 6 rows directly connected to GPIOs.
//
// C1    P95
// C2    P96
// C3    P97
// C4    P50
// C5    P51
// C6    P52
// C7    P53
// C8    P54
// C9    P55
// C10   P56
// C11   P57
// C12   P80   (also USB IAP trigger when pulled low)
//
// R1    P60
// R2    P61
// R3    P62
// R4    P63
// R5    P64
// R6    P65
//
// INT   P90
// SCL   P92
// SDA   P93
//
// We will want to keep keyboard controller asleep unless some key is
// pressed. If a key is pressed, the controller will continuously scan
// for further pressed keys. When all keys are released, the controller
// can go back to sleep.
//
// For this to work, we'll use port 6 ability to wake up the controller
// on change.
//
// During sleep:
// - all columns will be set to low state
// - all rows will have pull-up enabled
// - when user presses any key, row state will change to low and
//   the controller will wake up
//
// During active state:
// - all columns will be put to hi-Z state, except for the currently
//   scanned one, which will be in low state
// - state of rows will be read, and will indicate state of keys
//   in the selected column (0 = pressed, 1 = not pressed)
//
// De-bouncing:
// - scanning will happen in 5ms intervals and only if the two
//   consecutive scans match, will the result be considered valid
//
// Configure GPIO for keyboard key scanning

//
// Switch to idle state
//
// In this state we can use keyscan_idle_is_pressed() to detect whether
// any key is pressed, and switch to active mode via keyscan_active().
//
static void keyscan_idle(void)
{
	// enable output low on all columns (P9[7:5] P5[7:0] P8[0])

	PAGESW = 0;

	P5 = 0;
	P8 &= 0xfe;
	P9 &= 0x1f;

	P0_P5M0 = 0x00;
	P0_P8M0 &= 0xfeu;
	PAGESW = 1;
	P1_P9M0 &= 0x1fu;

	// delay a bit for things to stabilize
        delay_us(10);

	p6_changed = 0;
	scan_active = 0;
}

static uint8_t keyscan_idle_is_pressed(void)
{
	return ~P6 & 0x3f;
}

//
// Switch to active mode.
//
// In this state, we can call keyscan_scan() to perform a scan.
//
static void keyscan_active(void)
{
	// put all columns to hi-Z (P9[7:5] P5[7:0] P8[0])

	PAGESW = 0;

	P5 = 0;
	P8 &= 0xfe;
	P9 &= 0x1f;

	// make all columns an input (hi-Z) in preparation for individual
	// column scanning
	P0_P5M0 = ~0x00u;
	P0_P8M0 |= ~0xfeu;
	PAGESW = 1;
	P1_P9M0 |= ~0x1fu;

	scan_active = 1;
}

// 12 byte storage required
static uint8_t keyscan_scan(uint8_t* res)
{
	uint8_t pin, mask = 0, row;

	// for each column:
	// - output low on column
	// - wait (for voltage to stabilize)
	// - read rows
	// - turn column back to hi-Z

	PAGESW = 1;
	for (pin = 5; pin <= 7; pin++) {
		P1_P9M0 &= ~BIT(pin);
                delay_us(3);
                row = ~P6 & 0x3f;
                mask |= row;
		*res++ = row;
		P1_P9M0 |= BIT(pin);
	}

	PAGESW = 0;
	for (pin = 0; pin <= 7; pin++) {
		P0_P5M0 &= ~BIT(pin);
                delay_us(3);
                row = ~P6 & 0x3f;
                mask |= row;
		*res++ = row;
		P0_P5M0 |= BIT(pin);
	}

	P0_P8M0 &= ~BIT(0);
	delay_us(3);
        row = ~P6 & 0x3f;
        mask |= row;
	*res++ = row;
	P0_P8M0 |= BIT(0);

	return mask;
}

// }}}
// {{{ Enternal interrupt control

static void ext_int_assert(void)
{
	P90 = 0;
	PAGESW = 1;
	P1_P9M0 &= ~BIT(0);
}

static void ext_int_deassert(void)
{
	P90 = 0;
	PAGESW = 1;
	P1_P9M0 |= BIT(0);
}

// }}}
// {{{ CRC-8

static const uint8_t crc8_0x7_table[] = {
	0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15,
	0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
	0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
	0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
	0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5,
	0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
	0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85,
	0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
	0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
	0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
	0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2,
	0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
	0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32,
	0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
	0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
	0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
	0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c,
	0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
	0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec,
	0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
	0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
	0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
	0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c,
	0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
	0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b,
	0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
	0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
	0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
	0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb,
	0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
	0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb,
	0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
};

static uint8_t crc8(const uint8_t *pdata, size_t nbytes)
{
	unsigned int idx;
	uint8_t crc = 0;

	while (nbytes--) {
		idx = (crc ^ *pdata);
		crc = (crc8_0x7_table[idx]) & 0xff;
		pdata++;
	}

	return crc;
}

// }}}
// {{{ Public I2C register interface

#include "registers.h"

// all the variables are volatile because they can be accessed
// from interrupt context of I2C interrupt handler

static volatile uint8_t __idata ro_regs[REG_KEYMATRIX_STATE_END + 1] = {
	[REG_DEVID_K] = 'K',
	[REG_DEVID_B] = 'B',
	[REG_FW_REVISION] = FW_REVISION,
	[REG_FW_FEATURES] =
#if CONFIG_USB_STACK && CONFIG_DEBUG_LOG
		REG_FW_FEATURES_USB_DEBUGGER |
#endif
#if CONFIG_FLASH_ENABLE
		REG_FW_FEATURES_FLASHING_MODE |
#endif
#if CONFIG_SELFTEST
		REG_FW_FEATURES_SELF_TEST |
#endif
#if CONFIG_STOCK_FW
		REG_FW_FEATURES_STOCK_FW |
#endif
		0,
	[REG_KEYMATRIX_SIZE] = 0xc6, // 12 x 6
};

static volatile uint8_t ctl_regs[3] = {0, 0, 0};
static volatile __bit sys_cmd_run = 0;

static volatile uint8_t reg_addr = 0;

// }}}
// {{{ Flashing

static volatile uint8_t __code __at(0x3fff) app_flag;

#if CONFIG_FLASH_ENABLE

// all the variables are volatile because they can be accessed
// from interrupt context of I2C interrupt handler

static volatile uint8_t flash_regs[5] = {0, 0, 0, 0, 0};

// this is where the HW expects the data for a code ROM page
static volatile uint8_t __xdata __at(0x780) flash_content[128];
static volatile uint8_t __xdata __at(0x700) flash_content2[128];

// used to signal that flashing command should be executed and
// to block further writes to flashing registers via I2C
static volatile __bit flash_cmd_run = 0;

#define REG_FLASH(n) flash_regs[REG_FLASH_##n - REG_FLASH_ADDR_L]

static void user_app_flag_set(uint8_t flag) __critical
{
	if (app_flag == flag)
		return;

	for (uint8_t i = 0; i < 128; i++)
		flash_content2[i] = 0xff;
	flash_content2[0x7fu] = flag;

	PAGESW = 0;

	uint8_t ckcon_saved = CKCON1;
	CKCON1 = (CKCON1 & ~0x06u) | (1 << 1); // set HS pre-divider to /4

	// unlock
	P0_FLCR &= ~BIT(2);
	P0_FLKEY = 0xA9;
	P0_FLKEY = 0x7F;

	__asm__ (
		// dptr0 = source ptr, dptr1 = dest ptr
		"push psw\n"
		"push _DPL\n"
		"push _DPH\n"
		"push ar7\n"
		"push A\n"

		"mov _DPL1,#0x80\n"
		"mov _DPH1,#0x3f\n"
		"mov A,#_flash_content2\n"
		"mov _DPL,A\n"
		"mov A,#(_flash_content2 >> 8)\n"
		"mov _DPH,A\n"
		"mov r7,#0\n" // counter

		"00002$: movx a,@dptr\n"
		"inc dptr\n"
		"orl _PCON,#0x08\n" // select dptr1
		"movx @dptr,a\n"
		"inc dptr\n"
		"anl _PCON,#0xf7\n" // select dptr0
		"inc r7\n"
		"cjne r7,#0x80,00002$\n"

		"pop A\n"
		"pop ar7\n"
		"pop _DPH\n"
		"pop _DPL\n"
		"pop psw\n"
	);

	P0_FLCR |= BIT(0) | BIT(1);
	__asm__("nop");
	while (P0_FLCR & (BIT(0) | BIT(1))); // wait until programming is done

	CKCON1 = ckcon_saved;
}

static void exec_flashing_command(void)
{
	if (!flash_cmd_run)
		return;

	// skip normal result codes
	if (REG_FLASH(CMD) == 0 || REG_FLASH(CMD) == 0xff)
		goto out;

	// return error if the unlock magic is incorrect
	if (REG_FLASH(UNLOCK) != REG_FLASH_UNLOCK_MAGIC)
		goto err;

	if (REG_FLASH(CMD) == REG_FLASH_CMD_READ_ROM) {
		// does not need to be in critical section, because I2C access
		// is prevented via flash_cmd_run
		__asm__ (
			// dptr0 = CODE ptr, dptr1 = XRAM ptr
			"push psw\n"
			"push _DPL\n"
			"push _DPH\n"
			"push ar7\n"
			"push A\n"

			"mov _DPL,_flash_regs\n"
			"mov _DPH,(_flash_regs + 1)\n"
			"mov A,#_flash_content\n"
			"mov _DPL1,A\n"
			"mov A,#(_flash_content >> 8)\n"
			"mov _DPH1,A\n"

			"mov r7,#0\n" // counter
			"00001$: mov A,r7\n"
			"movc a,@a+dptr\n"
			"orl _PCON,#0x08\n" // select dptr1
			"movx @dptr,a\n"
			"inc dptr\n"
			"anl _PCON,#0xf7\n" // select dptr0
			"inc r7\n"
			"cjne r7,#0x80,00001$\n"

			"pop A\n"
			"pop ar7\n"
			"pop _DPH\n"
			"pop _DPL\n"
			"pop psw\n"
		);

		REG_FLASH(CRC8) = crc8(flash_content, 128);
	} else if (REG_FLASH(CMD) == REG_FLASH_CMD_WRITE_ROM) {
		// does not need to be in critical section, because I2C access
		// is prevented via flash_cmd_run
		if (REG_FLASH(CRC8) != crc8(flash_content, 128))
			goto err;
		if (REG_FLASH(ADDR_H) < 0x40 || REG_FLASH(ADDR_H) >= 0x80)
			goto err;
		if ((REG_FLASH(ADDR_L) % 128) != 0)
			goto err;

		user_app_flag_set(0xff);

		// Burn the code, we need to disable interrupts during write
		//
		// 1) unlock by writing 0xa9 0x7f to FLKEY
		// 2) set HS pre-divider to /4
		// 3) FLCR |= BIT(2) (if writing options)
		// 4) MOVX data to area starting from the ROM address we want to
		//    write
		// 5) FLCR |= BIT(0) | BIT(1);
		// 6) nop and wait for FLCR bits to clear
		// 7) FLCR &= ~BIT(2), restore pre-divider (cleanup)
		//
		// FLCR:
		// bit 0 = WE
		// bit 1 = EE
		// bit 2 = MEMSP
		// bit 7 = EPEN (protection)
		//
		__critical {
			PAGESW = 0;

			uint8_t ckcon_saved = CKCON1;
			CKCON1 = (CKCON1 & ~0x06u) | (1 << 1); // set HS pre-divider to /4

			// unlock
			P0_FLCR &= ~BIT(2);
			P0_FLKEY = 0xA9;
			P0_FLKEY = 0x7F;

			__asm__ (
				// dptr0 = source ptr, dptr1 = dest ptr
				"push psw\n"
				"push _DPL\n"
				"push _DPH\n"
				"push ar7\n"
				"push A\n"

				"mov _DPL1,_flash_regs\n"
				"mov _DPH1,(_flash_regs + 1)\n"
				"mov A,#_flash_content\n"
				"mov _DPL,A\n"
				"mov A,#(_flash_content >> 8)\n"
				"mov _DPH,A\n"
				"mov r7,#0\n" // counter

				"00002$: movx a,@dptr\n"
				"inc dptr\n"
				"orl _PCON,#0x08\n" // select dptr1
				"movx @dptr,a\n"
				"inc dptr\n"
				"anl _PCON,#0xf7\n" // select dptr0
				"inc r7\n"
				"cjne r7,#0x80,00002$\n"

				"pop A\n"
				"pop ar7\n"
				"pop _DPH\n"
				"pop _DPL\n"
				"pop psw\n"
			);

			P0_FLCR |= BIT(0) | BIT(1);
			__asm__("nop");
			while (P0_FLCR & (BIT(0) | BIT(1))); // wait until programming is done

			CKCON1 = ckcon_saved;
		}
	} else if (REG_FLASH(CMD) == REG_FLASH_CMD_COMMIT) {
		user_app_flag_set(1);
	} else if (REG_FLASH(CMD) == REG_FLASH_CMD_ERASE_ROM) {
		user_app_flag_set(0xff);
	} else {
		goto err;
	}

	REG_FLASH(CMD) = 0;
out:
	REG_FLASH(UNLOCK) = 0;
	flash_cmd_run = 0;
	return;

err:
	REG_FLASH(CMD) = 0xff;
	goto out;
}

#endif

// }}}
// {{{ Self-tests

#if CONFIG_SELFTEST

static void set_column_input(uint8_t col)
{
	if (col <= 2)
		PAGESW = 1;
	else
		PAGESW = 0;

	switch (col) {
	case 0:  P1_P9M0 |= BIT(5); break;
	case 1:  P1_P9M0 |= BIT(6); break;
	case 2:  P1_P9M0 |= BIT(7); break;
	case 3:  P0_P5M0 |= BIT(0); break;
	case 4:  P0_P5M0 |= BIT(1); break;
	case 5:  P0_P5M0 |= BIT(2); break;
	case 6:  P0_P5M0 |= BIT(3); break;
	case 7:  P0_P5M0 |= BIT(4); break;
	case 8:  P0_P5M0 |= BIT(5); break;
	case 9:  P0_P5M0 |= BIT(6); break;
	case 10: P0_P5M0 |= BIT(7); break;
	case 11: P0_P8M0 |= BIT(0); break;
	}
}

static void set_column_output(uint8_t col)
{
	if (col <= 2)
		PAGESW = 1;
	else
		PAGESW = 0;

	switch (col) {
	case 0:  P1_P9M0 &= ~BIT(5); break;
	case 1:  P1_P9M0 &= ~BIT(6); break;
	case 2:  P1_P9M0 &= ~BIT(7); break;
	case 3:  P0_P5M0 &= ~BIT(0); break;
	case 4:  P0_P5M0 &= ~BIT(1); break;
	case 5:  P0_P5M0 &= ~BIT(2); break;
	case 6:  P0_P5M0 &= ~BIT(3); break;
	case 7:  P0_P5M0 &= ~BIT(4); break;
	case 8:  P0_P5M0 &= ~BIT(5); break;
	case 9:  P0_P5M0 &= ~BIT(6); break;
	case 10: P0_P5M0 &= ~BIT(7); break;
	case 11: P0_P8M0 &= ~BIT(0); break;
	}
}

static __bit get_column_value(uint8_t col)
{
	switch (col) {
	case 0:  return P95;
	case 1:  return P96;
	case 2:  return P97;
	case 3:  return P50;
	case 4:  return P51;
	case 5:  return P52;
	case 6:  return P53;
	case 7:  return P54;
	case 8:  return P55;
	case 9:  return P56;
	case 10: return P57;
	case 11: return P80;
	default: return 0;
	}
}

static void set_column_value(uint8_t col, __bit val)
{
	switch (col) {
	case 0:  P95 = val; break;
	case 1:  P96 = val; break;
	case 2:  P97 = val; break;
	case 3:  P50 = val; break;
	case 4:  P51 = val; break;
	case 5:  P52 = val; break;
	case 6:  P53 = val; break;
	case 7:  P54 = val; break;
	case 8:  P55 = val; break;
	case 9:  P56 = val; break;
	case 10: P57 = val; break;
	case 11: P80 = val; break;
	}
}

static void self_test_run(void)
{
	PAGESW = 0;

	// all rows pull-up already as a defauklt config, so set all columns
	// to hi-Z first
	P0_P5M0 = ~0x00u;
	P0_P8M0 |= ~0xfeu;

	PAGESW = 1;
	P1_P9M0 |= ~0x1fu;

	// for each column:
	// - output low
	// - read other columns
	// - turn column back to hi-Z
	// data output:
	// - list of columns shorted together in 2 byte sequences as a bitmask
	//   - first byte: cols 12-9 aligned to LSB
	//   - second byte: cols 8-1
	// - terminated by two-byte 00 00 sequence

	puts("column self-test:\n");

	for (uint8_t c1 = 0; c1 < 12; c1++) {
                set_column_output(c1);

		for (uint8_t c2 = c1 + 1; c2 < 12; c2++) {
			set_column_value(c1, 0);
			__bit a = get_column_value(c2);
			set_column_value(c1, 1);
			__bit b = get_column_value(c2);

			if (!a && b) {
				// column-column short found
				puts("c-c short: ");
				put_uint(c1);
				puts("  ");
				put_uint(c2);
				puts("\n");
			}
		}

		set_column_input(c1);
	}

	puts("done\n");
}

#endif

// }}}
// {{{ System commands

#define REG_SYS(n) ctl_regs[REG_SYS_##n - REG_SYS_CONFIG]

static void exec_system_command(void)
{
	if (!sys_cmd_run)
		return;

	if (REG_SYS(COMMAND) == 0 || REG_SYS(COMMAND) == 0xff)
		goto out_done;

	if (REG_SYS(COMMAND) == REG_SYS_COMMAND_MCU_RESET) {
		RSTSC &= ~BIT(7);
		RSTSC |= BIT(7);
#if CONFIG_SELFTEST
	} else if (REG_SYS(COMMAND) == REG_SYS_COMMAND_SELFTEST) {
		self_test_run();
#endif
	} else if (REG_SYS(COMMAND) == REG_SYS_COMMAND_USB_IAP) {
		jump_to_usb_bootloader = 1;
	} else {
		REG_SYS(COMMAND) = 0xff;
		goto out_done;
	}

	REG_SYS(COMMAND) = 0x00;
out_done:
	sys_cmd_run = 0;
}

// }}}
// {{{ I2C register access

// only call this in interrupt context from regbank 1!
static uint8_t reg_get_value(void) __using(1)
{
#if CONFIG_DEBUG_LOG
	// read from this register reads the next byte of log buffer
	// (register address is not advanced!)
	if (reg_addr == 0xff) {
		if (log_start != log_end) {
			log_start = (log_start + 1) % 1024;
			return log_buffer[log_start]; // push data to fifo
		}

		goto none;
	}
#endif

	if (reg_addr <= REG_KEYMATRIX_STATE_END) {
		return ro_regs[reg_addr];
	} else if (reg_addr < REG_SYS_CONFIG) {
		goto none;
	} else if (reg_addr <= REG_SYS_USER_APP_BLOCK) {
		return ctl_regs[reg_addr - REG_SYS_CONFIG];
#if CONFIG_FLASH_ENABLE
	} else if (reg_addr < REG_FLASH_DATA_START) {
		goto none;
	} else if (reg_addr <= REG_FLASH_DATA_END) {
		return flash_content[reg_addr - REG_FLASH_DATA_START];
	} else if (reg_addr <= REG_FLASH_CMD) {
		return flash_regs[reg_addr - REG_FLASH_ADDR_L];
#endif
	}

none:
	return 0;
}

// only call this in interrupt context from regbank 1!
static void reg_set_value(uint8_t val) __using(1)
{
	if (reg_addr < REG_SYS_CONFIG) {
		return;
	} else if (reg_addr <= REG_SYS_USER_APP_BLOCK) {
		if (reg_addr == REG_SYS_COMMAND) {
			if (sys_cmd_run)
				return;
			sys_cmd_run = 1;
		}

		ctl_regs[reg_addr - REG_SYS_CONFIG] = val;
#if CONFIG_FLASH_ENABLE
	} else if (reg_addr < REG_FLASH_DATA_START) {
		return;
	} else if (reg_addr <= REG_FLASH_DATA_END) {
		if (flash_cmd_run)
			return;
		flash_content[reg_addr - REG_FLASH_DATA_START] = val;
	} else if (reg_addr <= REG_FLASH_CMD) {
		if (flash_cmd_run)
			return;

		flash_regs[reg_addr - REG_FLASH_ADDR_L] = val;

		if (reg_addr == REG_FLASH_CMD)
			flash_cmd_run = 1;
#endif
	}
}

/*
 * Host write transaction: sending 01 02 03 04 to device at 0x15 (0x2a == 0x15 << 1)
 *
 * int=60 CR1=8c CR2=af DATA_PRE=2a rx
 * int=60 CR1=8e CR2=af DATA_PRE=01 rx
 * int=60 CR1=8e CR2=af DATA_PRE=02 rx
 * int=60 CR1=8e CR2=af DATA_PRE=03 rx
 * int=60 CR1=8e CR2=af DATA_PRE=04 rx
 * int=70 CR1=0c CR2=2f DATA_PRE=04 stop
 *
 * Host read transaction: receiving 4 bytes
 *
 * int=a0 CR1=8d CR2=af tx
 * int=a0 CR1=8d CR2=af tx
 * int=a0 CR1=8d CR2=af tx
 * int=a0 CR1=89 CR2=af tx   NACK from host (last read byte)
 * int=b0 CR1=08 CR2=2f stop  STOP condition reported
 *
 * CR1:
 * 7: STROBE/PEND   (RX/TX: not set on stop IRQ, even though RXSF/TXSF is also set)
 * 3: SAR_EMPTY     (RX/TX: always set)
 * 2: ACK           (RX: always set)
 *                  (TX: set on all except on the last TX byte)
 * 1: FULL          (RX: not set on first RX byte, which is a device address)
 *                  (TX: always not set)
 * 0: EMPTY         (RX: always not set)
 *                  (TX: alwyas set except after stop IRQ)
 *
 * CR2:
 * 7: I2C busy flag (RX/TX: not set after stop IRQ)
 * 6: ?
 * 5: SW_RESET
 * 4: BBF
 *
 * I2CBINT:
 * 7: TXSF
 * 6: RXSF
 * 5: STP_IEN
 * 4: STOPF
 *
 * Powerdown is only possible after the stop bit. Wakeup only happens
 * on address match.
 */

#define I2C_ADDR 0x15

static volatile uint8_t i2c_n = 0;

// interrupt needs to be enabled for wakeup from powerdown to work
void i2c_b_interrupt(void) __interrupt(IRQ_I2CB) __using(1)
{
	uint8_t saved_page = PAGESW;
	PAGESW = 0;

	uint8_t intf = P0_I2CBINT;
	uint8_t cr1 = P0_I2CBCR1;
	uint8_t cr2 = P0_I2CBCR2;

	// handle stop condition
	if (intf & BIT(4)) {
		i2c_n = 0;
		P0_I2CBINT &= ~(BIT(4) | BIT(6) | BIT(7));
		goto out_restore_page;
	}

	// handle TX (byte to be sent to master - this is timing sensitive!)
	if (intf & BIT(7)) {
		// previous TX was the last byte
		if (!(cr1 & BIT(2)))
			goto tx_ack;

		P0_I2CBDB = reg_get_value();
		if (reg_addr != 0xff)
			reg_addr++;

		i2c_n++;
tx_ack:
		P0_I2CBINT &= ~BIT(7);
		goto out_restore_page;
	}

	// handle RX (byte received from master)
	if (intf & BIT(6)) {
		uint8_t tmp = P0_I2CBDB;

		// first RX byte is device address, determined by !FULL flag
		if (!(cr1 & BIT(1)))
			goto rx_ack;

		// set address
		if (i2c_n++ == 0) {
			reg_addr = tmp;
			goto rx_ack;
		}

		// set reg data
		reg_set_value(tmp);
		reg_addr++;

rx_ack:
		P0_I2CBINT &= ~BIT(6);
		goto out_restore_page;
	}

out_restore_page:
	P0_I2CBCR1 &= ~BIT(7); // clear data pending
	PAGESW = saved_page;
}

//
// Slave mode I2C for communication with the SoC
//
// - address is 0x15
// - 400kHz
// - interrupts are used to handle tx/rx/end of transaction (stop bit)
//
void i2c_slave_init(void)
{
	PAGESW = 0;

	// setup I2C B for slave mode
	//P0_I2CBCR1 = 0x20;
	//P0_I2CBCR2 = 0x07 << 1 | 0x01;  // 400kHz mode, enable I2C B controller, enable

	P0_I2CBCR1 = 0x00;
	P0_I2CBCR2 = 0x07 << 1 | BIT(0);  // 100kHz mode, enable I2C B controller, enable

	// setup I2C address
	P0_I2CBDAH = 0;
	P0_I2CBDAL = I2C_ADDR;

	P0_I2CBINT = BIT(5); // enable I2C B stop interrupt
	P0_EIE3 |= BIT(5); // enable I2C B interrupt
}

// }}}
// {{{ USB debugging interface

#if CONFIG_USB_STACK

#if USB_DEBUG

#define usb_putc(a)      putc(a)
#define usb_puts(a)      puts(a)
#define usb_put_hex_b(a) put_hex_b(a) 
#define usb_put_hex_w(a) put_hex_w(a)
#define usb_put_hex_n(a) put_hex_n(a)
#define usb_put_uint(a)  put_uint(a)

#else

#define usb_putc(a)
#define usb_puts(a)
#define usb_put_hex_b(a)
#define usb_put_hex_w(a)
#define usb_put_hex_n(a)
#define usb_put_uint(a)

#endif

#define USB_ID(w) (uint16_t)w & 0xff, ((uint16_t)w >> 8)
#define USB_BCD(a, b) b, a

static const uint8_t usb_desc_device[] ={
	18,			// bLength
	1,			// bDescriptorType
	USB_BCD(0x2, 0x0),	// bcdUSB
	0xff,			// bDeviceClass
	0,			// bDeviceSubClass
	0xff,			// bDeviceProtocol
	64,			// bMaxPacketSize0
	USB_ID(0x04f3),		// idVendor
	USB_ID(0xb001),		// idProduct
	USB_BCD(0x1, 0x0),	// bcdDevice
	1,			// iManfacturer
	2,			// iProduct
	0,			// iSerialNumber
	1,			// bNumConfgurations
};

#define USB_EP_OUT(addr, attr, maxsize, interval) \
	7, 5, addr, attr, USB_ID(maxsize), interval
#define USB_EP_IN(addr, attr, maxsize, interval) \
	USB_EP_OUT((addr) | 0x80, attr, maxsize, interval)

static const uint8_t usb_desc_config[] = {
	9,				// bLength
	2,				// bDescriptorType
	USB_ID(sizeof(usb_desc_config)),// bTotolLength
	1,				// bNumInterfaces
	1,				// bConfigurationValue
	0,				// iConfiguration string index
	BIT(7) // must be set           // bmAttributes
	| BIT(6) // self power
	| BIT(5) // remote wakeup
	,
	100,				// bMaxPower

	// Interface 0
	9,				// bLength
	4,				// bDescriptorType
	0,				// bInterfaceNumber
	0,				// bAlternateSetting
	4,				// bNumEndpoints
	0xff,				// bInterfaceClass
	0,				// bInterfaceSubClass
	0xff,				// bInterfaceProtocol
	0,				// iInterface

	USB_EP_OUT(1, 3, 64, 1), // request
	USB_EP_IN(2, 3, 64, 1),  // response
	USB_EP_IN(3, 3, 64, 1), // debug logging output
	USB_EP_IN(4, 3, 64, 1), // key status changes
};

static const uint8_t usb_string_lang[] = {
	4, 3,
	USB_ID(0x0409),
};

static const uint8_t usb_string_manufacturer[] = {
	4 * 2 + 2,
	3,

	'm', 0,
	'e', 0,
	'g', 0,
	'i', 0,
};

static const uint8_t usb_string_product[] = {
	5 * 2 + 2,
	3,

	'd', 0,
	'e', 0,
	'b', 0,
	'u', 0,
	'g', 0,
};

static const uint8_t * const usb_strings[] = {
	usb_string_lang,
	usb_string_manufacturer,
	usb_string_product,
};

static uint16_t usb_ep0_in_remaining = 0;
static uint8_t const* usb_ep0_in_ptr;
static uint8_t usb_command_status = 0;
static uint8_t usb_command[8];
static uint8_t usb_response[8];
static volatile __bit usb_key_change = 0;

static void usb_tasks(void) __using(1)
{
	uint8_t buf[8];
	uint8_t saved_page = PAGESW;

	PAGESW = 1;

	// handle reset request
	if (P1_UDCINT0STA & BIT(5)) {
		P1_USBCTRL |= BIT(5);
		P1_USBCTRL &= ~BIT(5);

		// clear EP0-3 buffers
		P1_UDCEPBUF0CTRL |= 0x55u;
		P1_UDCEPBUF0CTRL &= ~0x55u;
		// clear EP4
		P1_UDCEPBUF1CTRL |= BIT(0);
		P1_UDCEPBUF1CTRL &= ~BIT(0);

		// clear EP0 / EP1 in buffers
		P1_UDCBUFSTA &= ~(BIT(0) | BIT(1));

		//XXX: what about others?
                //XXX: reset software variables...
		usb_puts("usb rst\n");

		// ack reset request
		P1_UDCINT0STA &= ~BIT(5);
	}

	// ep0 setup request received
	if (P1_UDCINT0STA & BIT(1)) {
		usb_puts("ep0 su: ");

		// buf: bReqType bReq wVal(l/h) wIndex wLength
		for (uint8_t i = 0; i < 8; i++) {
			buf[i] = P1_UDCEP0BUFDATA;
			usb_put_hex_b(buf[i]);
		}

		usb_puts("\n");

		//P1_UDCEPBUF0CTRL |= BIT(0);
		//P1_UDCEPBUF0CTRL &= ~BIT(0);

		// how much data to send to ep0 in
		usb_ep0_in_remaining = (((uint16_t)buf[7] << 8) | buf[6]);
		uint16_t in0_len = 0;

		// standard commands
		if (buf[0] == 0x80) {
			// GET_DESCRIPTOR
                        if (buf[1] == 0x06) {
                                if (buf[3] == 1) {
					// device desc: 80 06 00 01 00 00
					if (buf[2] == 0) {
						usb_ep0_in_ptr = usb_desc_device;
						in0_len = sizeof(usb_desc_device);
						goto ack_ep0_setup;
					}
				} else if (buf[3] == 2) {
					// cfg desc: 80 06 00 02 00 00
					if (buf[2] == 0) {
						usb_ep0_in_ptr = usb_desc_config;
						in0_len = sizeof(usb_desc_config);
						goto ack_ep0_setup;
					}
				} else if (buf[3] == 3) {
					// string desc: 80 06 str_index 03 00 00
					if (buf[2] < sizeof(usb_strings) / sizeof(usb_strings[0])) {
						usb_ep0_in_ptr = usb_strings[buf[2]];
						in0_len = usb_ep0_in_ptr[0];
						goto ack_ep0_setup;
					}
				}
			}
		}

		usb_ep0_in_remaining = 0;
                P1_UDCCTRL |= BIT(4); // stall control endpoint req

ack_ep0_setup:
		if (in0_len < usb_ep0_in_remaining)
			usb_ep0_in_remaining = in0_len;

		// ack
		P1_UDCINT0STA &= ~BIT(1);
	}

	// USB host initiated EP0 IN transfer
	if (P1_UDCINT1STA & BIT(0)) {
		// ack interrupt
		P1_UDCINT1STA &= ~BIT(0);

		// check if we're ready to send to ep0
		if (!(P1_UDCEPBUF0CTRL & BIT(1)) && (P1_UDCBUFSTA & BIT(0))) {
			// if ep0 in buffer not empty, clear it first
			//if (!(P1_UDCBUFSTA & BIT(0))) {
				// clear ep0 buffer
				//P1_UDCEPBUF0CTRL |= BIT(0);
				//P1_UDCEPBUF0CTRL &= ~BIT(0);
			//}

			usb_puts("ep0 in: ptr=");
			usb_put_hex_w((uint16_t)usb_ep0_in_ptr);
			usb_puts(" rem=");
			usb_put_hex_w(usb_ep0_in_remaining);
			usb_puts(" ");

			for (uint8_t n = 0; n < 64; n++) {
				// push data to EP0 in (max 64 bytes)
				if (usb_ep0_in_remaining > 0) {
					usb_ep0_in_remaining--;
					usb_put_hex_b(*usb_ep0_in_ptr);
					P1_UDCEP0BUFDATA = *usb_ep0_in_ptr++;
				} else {
					break;
				}
			}

			usb_puts("\n");

			// confirm sending data
			P1_UDCEPBUF0CTRL |= BIT(1);
		}
	}

	// data received on ep0 out
	if (P1_UDCINT1STA & BIT(1)) {
		usb_puts("ep0 out\n");
		// we don't handle any control transfers that send us data

		// reset ep0 buf
		P1_UDCEPBUF0CTRL |= BIT(0);
		P1_UDCEPBUF0CTRL &= ~BIT(0);

		// ack interrupt
		P1_UDCINT1STA &= ~BIT(1);
	}

	// does not happen, EP1 IN is not configured on host
	if (P1_UDCINT1STA & BIT(2)) {
		P1_UDCINT1STA &= ~BIT(2);
	}

	// data received on ep1 out (command endpoint)
	if (P1_UDCINT1STA & BIT(3)) {
		// read data from ep1 fifo
		uint8_t bytes = P1_UDCEP1DATAOUTCNT + 1;

		for (uint8_t i = 0; i < 8; i++)
			usb_command[i] = P1_UDCEP1BUFDATA;
		usb_command_status = 1;
		usb_puts("ep1 out\n");

		P1_UDCINT1STA &= ~BIT(3);

		// clear the rest
		P1_UDCEPBUF0CTRL |= BIT(2);
		P1_UDCEPBUF0CTRL &= ~BIT(2);

		//do {
			//P1_USBCTRL |= BIT(6);
		//} while(!(P1_USBCTRL & BIT(6)));
	}

	// process USB commands
        if (usb_command_status == 1) {
		// what command the response is for
		usb_response[0] = usb_command[0];
		// success = 0, error code otherwise
		usb_response[1] = 0x00;

                if (usb_command[0] == 0x01) {
			// bootloader mode
			jump_to_usb_bootloader = 1;
		} else {
			// command unknown
			usb_response[1] = 1;
		}

		usb_command_status = 2;
	}

	// USB host initiated EP2 IN transfer
	if (P1_UDCINT1STA & BIT(4)) {
		// send out response to last command on ep2 in
		if (usb_command_status == 2 && !(P1_UDCEPBUF0CTRL & BIT(5))) {
			P1_UDCEP2DATAINCNT = 8 - 1; // how much bytes to send

			for (uint8_t i = 0; i < 8; i++)
				P1_UDCEP2BUFDATA = usb_response[i];

			P1_UDCEPBUF0CTRL |= BIT(5); // EP2 data ready
			usb_command_status = 0;
		}

		// ack
		P1_UDCINT1STA &= ~BIT(4);
	}

	// USB host initiated EP3 IN transfer
	if (P1_UDCINT1STA & BIT(6)) {
#if CONFIG_DEBUG_LOG
		// all log_* variables need to be accessed with interrupts
		// disabled
		__critical {
			// push printf debug buffer to ep3 in
			if (!(P1_UDCEPBUF0CTRL & BIT(7)) && log_start != log_end) {
				uint8_t cnt = 0;

				while (cnt < 64 && log_start != log_end) {
					log_start = (log_start + 1) % 1024;
					P1_UDCEP3BUFDATA = log_buffer[log_start]; // push data to fifo
					cnt++;
				}

				P1_UDCEP3DATAINCNT = cnt - 1;
				P1_UDCEPBUF0CTRL |= BIT(7); // EP3 data ready
			}
		}
#endif
		// ack
		P1_UDCINT1STA &= ~BIT(6);
	}

	// USB host initiated EP4 IN transfer
	if (P1_UDCINT2STA & BIT(2)) {
		// push key change events to ep4 in
		if (!(P1_UDCEPBUF1CTRL & BIT(1)) && usb_key_change) {
			for (uint8_t i = 0; i < 12; i++)
				P1_UDCEP4BUFDATA = ro_regs[i + REG_KEYMATRIX_STATE];

			P1_UDCEP4DATAINCNT = 12 - 1;
			P1_UDCEPBUF1CTRL |= BIT(1); // EP4 data ready
			usb_key_change = 0;
		}

		// ack
		P1_UDCINT2STA &= ~BIT(2);
	}

	// suspend request
	if (P1_UDCINT0STA & BIT(6)) {
		usb_puts("usb suspend\n");

                // host requests suspend, we satisfy it

		// clear device resume request bit, we can set it later to wake
		// the host / resume USB activity
		P1_UDCCTRL &= ~BIT(5);

		// ack
		P1_UDCINT0STA &= ~BIT(6);
	}

	// resume request
	if (P1_UDCINT0STA & BIT(3)) {
		usb_puts("usb resume\n");

		// ack
		P1_UDCINT0STA &= ~BIT(3);
	}

	PAGESW = saved_page;
}

void usb_interrupt(void) __interrupt(IRQ_USB) __using(1)
{
	usb_tasks();
}

enum {
	UDC_EP_CONTROL = 0,
	UDC_EP_ISO,
	UDC_EP_BULK,
	UDC_EP_INTERRUPT,
};

#define UDC_EP_CONF(conf, intf, alt, type) \
        (conf << 6) | (intf << 4) | (alt << 2) | type
#define UDC_EP_OUT_CONF(ep1, ep2, ep3, ep4) \
	ep4 | (ep3 << 2) | (ep2 << 4) | (ep1 << 6)

static const uint8_t udc_config[5] = {
	UDC_EP_CONF(1, 0, 0, UDC_EP_INTERRUPT),
	UDC_EP_CONF(1, 0, 0, UDC_EP_INTERRUPT),
	UDC_EP_CONF(1, 0, 0, UDC_EP_INTERRUPT),
	UDC_EP_CONF(1, 0, 0, UDC_EP_INTERRUPT),
	UDC_EP_OUT_CONF(UDC_EP_INTERRUPT, UDC_EP_INTERRUPT, UDC_EP_INTERRUPT, UDC_EP_INTERRUPT),
};

static void usb_init(void)
{
	PAGESW = 1;

	P1_UDCCTRL |= BIT(6); // udc enable
	// wait for UDC to complete initialization
	while (!(P1_UDCCTRL & BIT(1)));
	__asm__("nop");

	// setup USB EP depths
	P1_UDCEP1BUFDEPTH = 64 - 1;
	P1_UDCEP2BUFDEPTH = 64 - 1;
	P1_UDCEP3BUFDEPTH = 64 - 1;
	P1_UDCEP4BUFDEPTH = 64 - 1;
	__asm__("nop");
	__asm__("nop");

	// configure UDC
	for (uint8_t i = 0; i < 4; i++) {
		P1_UDCCFDATA = udc_config[i];

		while (!(P1_UDCCFSTA & BIT(7)));
		while (P1_UDCCFSTA & BIT(7));
	}

	P1_UDCCFDATA = udc_config[4];
	while (!(P1_UDCCFSTA & BIT(6)));

	// enable USB EPRDY
	P1_USBCTRL |= BIT(6);

	P1_UDCEPCTRL = 0xf;
	P1_UDCINT0STA = 0;
	P1_UDCINT1STA = 0;
	P1_UDCINT2STA = 0;

	P1_UDCINT0EN = BIT(5) | BIT(1) | BIT(6) | BIT(3);
	P1_UDCINT1EN = BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(6);
	P1_UDCINT2EN = BIT(2);
	//P1_UDCINT0EN = 0;
	//P1_UDCINT1EN = 0;
	//P1_UDCINT2EN = 0;

	// enable phy, wakeup enable
	P1_PHYTEST0 |= BIT(5) | BIT(6);
	__asm__("nop");
	__asm__("nop");

	PAGESW = 0;

	// enable USB interrupts
	P0_EIE2 |= BIT(2);
}

#endif

static void usb_disable(void)
{
	// reset phy/usb
	PAGESW = 1;

	P1_PHYTEST0 &= ~(BIT(6) | BIT(5)); // phy disable
	P1_UDCCTRL &= ~BIT(6); // udc disable
}

// }}}

void main(void)
{
	PAGESW = 0;

	// setup interrupts
	EA = 0;
	IE = 0;
	P0_EIE1 = 0;
	P0_EIE2 = 0;
	P0_EIE3 = 0;

	// set CPU clock to normal (high frequency) mode
	// [7] = power down HS clock in low speed mode - 1: yes 0: no
	// [2:1] = high speed clock pre-divider -  1: /4 2: /2 3: /1
	// [0] = cpu clock mode 1: high speed mode  0: low speed mode
	CKCON1 = (CKCON1 & ~0x87u) | 0x07; // 0x87

	// set timer 1 and timer 0 clock source to sysclk/12 (2 MHz)
	CKCON0 = 0x00;

	// wait until high speed clock is stable
	while (!(CKCON0 & BIT(1)));

	// set both timers to 16-bit counter modes
	TMOD = 0x11;

	// enable both timers
	TCON = 0x50;

	// protect FLASH from 0x0000 to 0x4000 from being accessed by code at 0x4000+
	P0_EPPOINTL = 0x80;

	// setup watchdog (timer base is 8ms, prescaler sets up timeout /128 = ~1s)
//	P0_WDTCR = 0x87; // enable watchdog ~1s
//	P0_WDTKEY = 0x4e; // reset watchdog

	P0_WDTCR = 0x07; // disable watchdog ~1s
	P0_WDTKEY = 0xb1; // disable watchdog

	// power down unused peripherlas
	P0_DEVPD1 |= BIT(6) | BIT(5) | BIT(3) | BIT(1); // PWM A, timer 3, SPI, LVD
	P0_DEVPD2 |= BIT(6) | BIT(3) | BIT(0); // PWM C, PWM B, I2C A
	P0_DEVPD3 |= BIT(2) | BIT(1) | BIT(0); // PWM E, PWM D, PWM F

	// keep UART, SPI, and I2C A in reset
	//P0_PRST |= BIT(0) | BIT(2) | BIT(3);

	// enable pullups only all port 6 pins and make those pins into input
	PAGESW = 0;
	P0_PHCON0 = 0x00;
	P0_PHCON1 = 0xffu; // port 6 pull-up enable
	P0_P6M0 = 0xff; // port 6 input
	PAGESW = 1;
	P1_PHCON2 = 0x00;

	// enable auto-tuning internal RC oscillator based on USB SOF packets
	P1_IRCCTRL &= ~BIT(1); // disable manual trim

#if CONFIG_STOCK_FW
	puts("ppkb firmware " FW_REVISION_STR " (stock)\n");
#else
	puts("ppkb firmware " FW_REVISION_STR " (user)\n");
#endif

	i2c_slave_init();

	T1_SET_TIMEOUT(40000);

	usb_disable();

#if !CONFIG_USB_STACK
	PAGESW = 1;

	// GPIO on USB pins
	P1_USBCTRL &= ~BIT(7);

	// turn off PLL48
	P1_UDCCTRL |= BIT(0);

	// turn off unused USB resources (phy power down, PLL48 powerdown
	P1_USBCTRL |= BIT(0) | BIT(1);

	// enable auto-tuning internal RC oscillator based on USB SOF packets
	P1_IRCCTRL |= BIT(1); // enable manual trim
#endif

#if CONFIG_FLASH_ENABLE
	for (uint8_t i = 0; i < 128; i++)
		flash_content[i] = 0;
#endif

	// enable interrupts
	ET1 = 1;
	EA = 1;
	ext_int_deassert();

	keyscan_idle();

	__bit usb_initialized = 0;
	__bit user_app_checked = 0;
	uint16_t ticks = 0;
	while (1) {
		// execute I2C system/flashing commands, once the I2C
		// transaction ends, as soon as possible
		if (i2c_n == 0) {
			exec_system_command();

#if CONFIG_FLASH_ENABLE
			exec_flashing_command();
#endif
		}

		// if we were asked to jump to USB IAP, do it
		if (jump_to_usb_bootloader)
			__asm__ ("ljmp _usb_bootloader_jump");

		// if the 20ms timer did not expire yet, check if we can
		// powerdown, otherwise busyloop
		if (!run_timed_tasks) {
#if CONFIG_USB_STACK
			PAGESW = 1;
			__bit usb_suspended = !!(P1_UDCCTRL & BIT(2));
#endif
			PAGESW = 0;
			__bit i2c_idle = !(P0_I2CBCR2 & BIT(7)) && i2c_n == 0;

			// if USB is suspended by host and I2C has no activity,
			// and we're not in active scanning mode, power down the MCU
			if (i2c_idle && !scan_active
			    && !p6_changed
#if CONFIG_USB_STACK
			    && usb_initialized  && usb_suspended
#endif
#if CONFIG_STOCK_FW
			    && user_app_checked
#endif
			    ) {
				// go to idle CPU mode when there's nothing to
				// do, any interrupt will wake us
				//PCON |= BIT(0);

				// enable interrupt whenever P6 is different
				// from the current value (which would be
				// whenever some key is pressed, because by
				// default all pins on P6 are pulled high)
				//
				// input change detection works by comparing the
				// pin state against the P6 latch for output
				p6_changed = 0;
				P6 = P6;
				P0_ICEN = BIT(5);
				ICIE = 1;

				// power down (timers don't work in power-down)
				PCON |= BIT(1) | BIT(0);
				__asm__("nop");

				// we may not be woken up only by IC interrupt, so
				// disable IC interrupts after each wakeup
				ICIE = 0;

#if CONFIG_USB_STACK
				// if we were woken up by USB host, USBCTRL.5
				// will be set, clear it
				PAGESW = 1;
				if (!(P1_UDCCTRL & BIT(2)))
					P1_USBCTRL &= ~BIT(5);
#endif
			}

			continue;
		}

		// every 20ms we will get here to perform some timed tasks
		ticks++;
		run_timed_tasks = 0;

#if CONFIG_STOCK_FW
		// after 1s check if we should jump to user firmware
		if (!user_app_checked && ticks > 1000 / 20) {
			if (app_flag == 1 && ctl_regs[REG_SYS_USER_APP_BLOCK - REG_SYS_CONFIG] != REG_SYS_USER_APP_BLOCK_MAGIC)
				jmp_to_user_fw();

			user_app_checked = 1;
		}
#endif

#if CONFIG_USB_STACK
		// after 500ms, init usb
		if (!usb_initialized && ticks > 500 / 20) {
			usb_init();
			usb_initialized = 1;
		}
#endif

		// if active scanning is not active and port 6 change was
		// detected, and some key is still pressed, enter active
		// scanning mode
		if (!scan_active && keyscan_idle_is_pressed())
			keyscan_active();

		// if we're in active scanning, scan the keys, and report
		// new state
		if (scan_active) {
			uint8_t keys[12];
			uint8_t active_rows = keyscan_scan(keys);

			// check for changes
			if (memcmp(ro_regs + REG_KEYMATRIX_STATE, keys, 12)) {
				// update regs
				__critical {
					memcpy(ro_regs + REG_KEYMATRIX_STATE, keys, 12);
					ro_regs[REG_KEYMATRIX_STATE_CRC8] = crc8(ro_regs + REG_KEYMATRIX_STATE, 12);
				}

				// signal interrupt
				ext_int_assert();
				delay_us(100);
				ext_int_deassert();
#if CONFIG_USB_STACK
				usb_key_change = 1;

				// USB wakeup
				PAGESW = 1;
				if (P1_UDCCTRL & BIT(2)) {
					P1_UDCCTRL |= BIT(5);
					P1_UDCCTRL &= ~BIT(5);
				}
#endif

				// pressing FN+PINE+F switches to flashing mode (keys 1:2 3:5 5:2, electrically)
				if (keys[0] & BIT(2) && keys[2] & BIT(5) && keys[4] & BIT(2))
					jump_to_usb_bootloader = 1;
			}

			if (!active_rows)
				keyscan_idle();
		}
	}
}
