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

// {{{ Key scanning

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

// }}}
// {{{ I2C

#define I2C_N_REGS 16

static uint8_t i2c_transfer = 0x00;
static uint8_t i2c_addr = 0;
static uint8_t i2c_regs[I2C_N_REGS] = {0xaa, 0x55};
static uint8_t i2c_cmd[I2C_N_REGS];
static uint8_t i2c_cmd_len = 0;

//XXX: how to determine end of I2C transaction from the interrupt?
//XXX: we need this to be able to determine when it's safe to go back to sleep/power down

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

// }}}
// {{{ Debug logging

static uint8_t __xdata log_buffer[1024];
// end = start => empty buffer
// end can never equal start on a filled buffer
// end points to the last char if end != start
static uint16_t log_start = 0;
static uint16_t log_end = 0;

static void putc(char c)
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

// }}}
// {{{ USB

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

static void usb_disable(void)
{
	// reset phy/usb
	PAGESW = 1;
	P1_PHYTEST0 &= ~BIT(6); // phy disable
	P1_UDCCTRL &= ~BIT(6); // udc disable
}

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

	// enable USB
	P1_USBCTRL |= BIT(6);

	P1_UDCINT0EN = 0;
	P1_UDCINT1EN = 0;
	P1_UDCINT2EN = 0;
	P1_UDCEPCTRL = 0xf;
	P1_UDCINT0STA = 0;
	P1_UDCINT1STA = 0;
	P1_UDCINT2STA = 0;

	// enable phy
	P1_PHYTEST0 |= BIT(5) | BIT(6);
	__asm__("nop");
	__asm__("nop");

	PAGESW = 0;
}

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

static uint16_t usb_ep0_in_remaining;
static uint8_t const*  usb_ep0_in_ptr;
static uint8_t usb_command_status = 0;
static uint8_t usb_key_change = 0;
static uint8_t usb_command[8];
static uint8_t usb_response[8];

static void usb_tasks(void)
{
	uint8_t buf[8];

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

		puts("usb reset int\n");

		// ack reset request
		P1_UDCINT0STA &= ~BIT(5);
	}

	// ep0 setup request received
	if (P1_UDCINT0STA & BIT(1)) {
		// buf: bReqType bReq wVal(l/h) wIndex wLength
		for (uint8_t i = 0; i < 8; i++)
			buf[i] = P1_UDCEP0BUFDATA;

		// how much data to send to ep0 in
		usb_ep0_in_remaining = (uint16_t)((buf[7] << 8) | buf[6]);
		uint16_t in0_len = 0;

		puts("ep0 setup: ");
		put_hex_b(buf[0]);
		put_hex_b(buf[1]);
		put_hex_b(buf[2]);
		put_hex_b(buf[3]);
		putc('\n');

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
		// check if we're ready to send to ep0
		if (!(P1_UDCEPBUF0CTRL & BIT(1))) {
			puts("ep0 in int ack\n");

			// if ep0 in buffer not empty, clear it first
			if (!(P1_UDCBUFSTA & BIT(0))) {
				// clear ep0 buffer
				P1_UDCEPBUF0CTRL |= BIT(0);
				P1_UDCEPBUF0CTRL &= ~BIT(0);
			}

			for (uint8_t n = 0; n < 64; n++) {
				// push data to EP0 in (max 8 bytes)
				if (usb_ep0_in_remaining > 0) {
					usb_ep0_in_remaining--;
					P1_UDCEP0BUFDATA = *usb_ep0_in_ptr++;
				} else {
					break;
				}
			}

			// confirm sending data
			P1_UDCEPBUF0CTRL |= BIT(1);
			// ack interrupt
			P1_UDCINT1STA &= ~BIT(0);
		}
	}

	// data received on ep0 out
	if (P1_UDCINT1STA & BIT(1)) {
		// we don't handle any control transfers that send us data

		// reset ep0 buf
		P1_UDCEPBUF0CTRL |= BIT(0);
		P1_UDCEPBUF0CTRL &= ~BIT(0);

		// ack interrupt
		P1_UDCINT1STA &= ~BIT(1);
	}

	// does not happen, EP1 IN is not configured on host
	if (P1_UDCINT1STA & BIT(2)) {
		puts("ep1 in int ack\n");
		P1_UDCINT1STA &= ~BIT(2);
	}

	// data received on ep1 out (command endpoint)
	if (P1_UDCINT1STA & BIT(3)) {
		// read data from ep1 fifo
		uint8_t bytes = P1_UDCEP1DATAOUTCNT + 1;

		puts("usb cmd len=");
		put_uint(bytes);
		putc(' ');
		for (uint8_t i = 0; i < 8; i++) {
			usb_command[i] = P1_UDCEP1BUFDATA;

			putc(' ');
			put_hex_b(usb_command[i]);
		}
		usb_command_status = 1;
		putc('\n');

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
			EA = 0;
			__asm__("mov r6,#0x5a");
			__asm__("mov r7,#0xe7");
			__asm__("ljmp 0x0118");
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

			puts("ep2 in response\n");

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

		// ack
		P1_UDCINT1STA &= ~BIT(6);
	}

	// USB host initiated EP4 IN transfer
	if (P1_UDCINT2STA & BIT(2)) {
		// push key change events to ep4 in
		if (!(P1_UDCEPBUF1CTRL & BIT(1)) && usb_key_change) {
			puts("key change sent\n");

			for (uint8_t i = 0; i < 12; i++)
				P1_UDCEP4BUFDATA = i2c_regs[i + 4];

			P1_UDCEP4DATAINCNT = 12 - 1;
			P1_UDCEPBUF1CTRL |= BIT(1); // EP4 data ready
			usb_key_change = 0;
		}

		// ack
		P1_UDCINT2STA &= ~BIT(2);
	}

	// suspend request
	if (P1_UDCINT0STA & BIT(6)) {
		puts("suspend int ack\n");
		// ack
		P1_UDCINT0STA &= ~BIT(6);

                //XXX: handle suspend properly

		// suspend UDC
		P1_UDCCTRL &= ~BIT(5);
	}
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
	P1_IRCCTRL &= ~BIT(1); // disable manual trim

	i2c_slave_init();

	T1_SET_TIMEOUT(40000);

	usb_disable();

	// enable interrupts
	ET1 = 1;
	EA = 1;
	ext_int_deassert();

	puts("Booted kb 0.1\n");

#if POLL_INPUT
	keyscan_active();
#else
	keyscan_idle();
#endif
	uint8_t asserted = 0;
	uint8_t usb_initialized = 0;
	uint16_t ticks = 0;
	while (1) {
		if (usb_initialized)
			usb_tasks();

		if (!run_tasks) {
			// power down (timers don't work in power-down)
			//PCON |= BIT(1);
			// go to idle CPU mode when there's nothing to do (doesn't help much)
			// switching to LOSC may work better
			//PCON |= BIT(0);

			__asm__("nop");
			continue;
		}

		ticks++;
		run_tasks = 0;

		// usb init needs to run after 500ms
		if (ticks > 500 / 20 && !usb_initialized) {
			usb_init();
			usb_initialized = 1;
		}

#if POLL_INPUT
		// every 20ms we will scan the keyboard keys state and check for changes
		uint8_t keys[12];
		uint8_t active_rows = keyscan_scan(keys);

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
		usb_key_change = 1;
#else
		//XXX: not figured out yet, not tested, not working
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
