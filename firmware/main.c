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

// configuration (we can make this runtime configurable via i2c)
// polled input mode is necessary if some rows are always on
#define POLL_INPUT 1

#define BIT(n) (1u << (n))

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

static __sbit p6_changed = 0;
static __sbit run_tasks = 0;

// we use this interrupt for wakeup from sleep on input change

void pinchange_interupt(void) __interrupt(IRQ_PINCHANGE)
{
	uint8_t saved_page = PAGESW;

	PAGESW = 0;

	if (P0_ICEN & BIT(1))
		p6_changed = 1;
	
	// clear input change flags
	P0_ICEN = BIT(5);

	PAGESW = saved_page;
}

// we use this interrupt as a scheduling tick (wakeup from sleep)

void timer1_interupt(void) __interrupt(IRQ_TIMER1)
{
	run_tasks = 1;

	// 20 ms
	T1_SET_TIMEOUT(40000);

	TF1 = 0;
}

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
void keyscan_idle(void)
{
	// enable output low on all columns (P9[7:5] P5[7:0] P8[0])

	PAGESW = 0;

	P5 = 0;
	P8 &= 0xfe;
	P9 &= 0x1f;

#if POLL_INPUT
	// make all columns an input, hi-Z (saves power)
	P0_P5M0 = ~0x00u;
	P0_P8M0 |= ~0xfeu;
	PAGESW = 1;
	P1_P9M0 |= ~0x1fu;
	ICIE = 0;
	p6_changed = 0;
#else
	P0_P5M0 = 0x00;
	P0_P8M0 &= 0xfe;
	PAGESW = 1;
	P1_P9M0 &= 0x1f;

	// enable input change interrupt on port6 and clear the interrupt flag after
	// things stabilize
        delay_us(10);

	PAGESW = 0;
	p6_changed = 0;
	P0_ICEN = BIT(5);
	ICIE = 1;
#endif
}

uint8_t keyscan_idle_is_pressed(void)
{
	return ~P6 & 0x3f;
}

//
// Switch to active mode.
//
// In this state, we can call keyscan_scan() to perform a scan.
//
void keyscan_active(void)
{
	// put all columns to hi-Z (P9[7:5] P5[7:0] P8[0])

	// disable input change interrupt
	ICIE = 0;

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
}

// XXX: do we need to debounce in the scan function?
// XXX: it looks like that there should be no bouncing going on mechanically

// 12 byte storage required
uint8_t keyscan_scan(uint8_t* res)
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

void ext_int_assert(void)
{
	P90 = 0;
	PAGESW = 1;
	P1_P9M0 &= ~BIT(0);
}

void ext_int_deassert(void)
{
	P90 = 0;
	PAGESW = 1;
	P1_P9M0 |= BIT(0);
}

#define I2C_N_REGS 16

static uint8_t i2c_transfer = 0x00;
static uint8_t i2c_addr = 0;
static uint8_t i2c_regs[I2C_N_REGS] = {0xaa, 0x55};
static uint8_t i2c_cmd[I2C_N_REGS];
static uint8_t i2c_cmd_len = 0;

void i2c_b_interupt(void) __interrupt(IRQ_I2CB)
{						    
	uint8_t saved_page = PAGESW;
	uint8_t tmp;
	PAGESW = 0;

	// handle TX
	if (P0_I2CBINT & BIT(7)) {
		if (i2c_addr < 16)
			P0_I2CBDB = i2c_regs[i2c_addr++];
		else
			P0_I2CBDB = 0xff;

		P0_I2CBCR1 &= ~BIT(7); // clear data pending
		P0_I2CBINT &= ~BIT(7);
	}
	
	// handle RX
	if (P0_I2CBINT & BIT(6)) {
		tmp = P0_I2CBDB;
		if (i2c_cmd_len < 16)
			i2c_cmd[i2c_cmd_len++] = tmp;

		PAGESW = 0;

		P0_I2CBCR1 &= ~BIT(7); // clear data pending
		P0_I2CBINT &= ~BIT(6);
	}
	
	// handle stop condition
	if (P0_I2CBINT & BIT(4)) {
		i2c_addr = 0;
		i2c_cmd_len = 0;
		P0_I2CBINT &= ~BIT(4);
	}

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
	P0_I2CBDAH = 0x00;
	P0_I2CBDAL = 0x15;

	P0_I2CBINT = BIT(5); // enable I2C B stop interrupt
	P0_EIE3 |= BIT(5); // enable I2C B interrupt
}

void main(void)
{
	uint8_t scan_active = 0;

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
	P0_PHCON1 = 0xff; // port 6 pull-up enable
	P0_P6M0 = 0xff; // port 6 input
	PAGESW = 1;
	P1_PHCON2 = 0x00;

	// enable auto-tuning internal RC oscillator based on USB SOF packets
	//P1_IRCCTRL &= ~BIT(1); // disable manual trim

	i2c_slave_init();

	T1_SET_TIMEOUT(40000);

	// enable interrupts
	ET1 = 1;
	EA = 1;
	ext_int_deassert();

#if POLL_INPUT
	keyscan_active();
#else
	keyscan_idle();
#endif
	uint8_t asserted = 0;
	//uint16_t ticks = 0;
	while (1) {
		if (!run_tasks) {
			// power down
			//PCON |= BIT(1);

			// go to idle CPU mode when there's nothing to do
			//PCON |= BIT(0);
			__asm__("nop");
			continue;
		}

		//ticks++;
		run_tasks = 0;

#if POLL_INPUT
		// every 10ms we will scan the keyboard keys state and check for changes
		uint8_t keys[12];
		uint8_t active_rows = keyscan_scan(keys);
		if (active_rows) {
			// pressing FN+PINE+F switches to flashing mode (keys 1:2 3:5 5:2, electrically)
			if (keys[0] & BIT(2) && keys[2] & BIT(5) && keys[4] & BIT(2)) {
				EA = 0;
				__asm__("mov r6,#0x5a");
				__asm__("mov r7,#0xe7");
				__asm__("ljmp 0x0118");
			}

			// check for changes
			if (!memcmp(i2c_regs + 4, keys, 12))
				continue;

			// signal interrupt
			memcpy(i2c_regs + 4, keys, 12);
			ext_int_assert();
			delay_us(100);
			ext_int_deassert();
		}

#else
		if (scan_active) {
			uint8_t active_rows = keyscan_scan(i2c_regs + 4);
			if (!active_rows) {
				scan_active = 0;
				keyscan_idle();

				// power down
				//PCON |= BIT(1);
				//__asm__("nop");
			}
			
			// pressing FN+PINE+F switches to flashing mode (keys 1:2 3:5 5:2, electrically)
			if (i2c_regs[4 + 0] & BIT(2) && i2c_regs[4 + 2] & BIT(5) && i2c_regs[4 + 4] & BIT(2)) {
				EA = 0;
				__asm__("mov r6,#0x5a");
				__asm__("mov r7,#0xe7");
				__asm__("ljmp 0x0118");
			}

			continue;
		}
		
		if (keyscan_idle_is_pressed()) {
			scan_active = 1;
			keyscan_active();
		}
#endif
	}
}
