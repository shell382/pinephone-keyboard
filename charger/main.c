/*
 * Pinephone keyboard power management daemon/tool.
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

// {{{ includes

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/types.h>
#include <dirent.h>
#include <poll.h>

#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/gpio.h>
//#include <i2c/smbus.h>

#define DEBUG 1

#if DEBUG
#define debug(args...) printf(args)
#else
#define debug(args...)
#endif

// }}}
// {{{ utils

static void syscall_error(int is_err, const char* fmt, ...)
{
	va_list ap;

	if (!is_err)
		return;

	fprintf(stderr, "ERROR: ");
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fprintf(stderr, ": %s\n", strerror(errno));

	exit(1);
}

static void error(const char* fmt, ...)
{
	va_list ap;

	fprintf(stderr, "ERROR: ");
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fprintf(stderr, "\n");

	exit(1);
}

bool read_file(const char* path, char* buf, size_t size)
{
	int fd;
	ssize_t ret;

	fd = open(path, O_RDONLY);
	if (fd < 0)
		return false;

	ret = read(fd, buf, size);
	close(fd);
	if (ret < 0)
		return false;

	if (ret < size) {
		buf[ret] = 0;
		return true;
	} else {
		buf[size - 1] = 0;
		return false;
	}
}

#define BIT(n) (1u << (n))

#define KB_ADDR 0x15
#define POWER_ADDR 0x75

static int pogo_i2c_open(void)
{
	int ret;
	char path[256], buf[1024];
	int fd = -1;

	for (int i = 0; i < 8; i++) {
		snprintf(path, sizeof path, "/sys/class/i2c-adapter/i2c-%d/uevent", i);
		if (!read_file(path, buf, sizeof buf))
			continue;
		
		if (!strstr(buf, "OF_FULLNAME=/soc/i2c@1c2b400"))
			continue;
		
		snprintf(path, sizeof path, "/dev/i2c-%d", i);
		
		int fd = open(path, O_RDWR);
		syscall_error(fd < 0, "open(%s) failed");

		//ret = ioctl(fd, I2C_SLAVE, addr);
		//syscall_error(ret < 0, "I2C_SLAVE failed");

		return fd;		
	}
	
	error("Can't find POGO I2C adapter");
	return -1;
}

uint8_t read_power(int fd, uint8_t reg)
{
	int ret;
	uint8_t val;
	struct i2c_msg msgs[] = {
		{ POWER_ADDR, 0, 1, &reg }, // address
		{ POWER_ADDR, I2C_M_RD, 1, &val },
	};

	struct i2c_rdwr_ioctl_data msg = {
		.msgs = msgs,
		.nmsgs = sizeof(msgs) / sizeof(msgs[0])
	};

	ret = ioctl(fd, I2C_RDWR, &msg);
	syscall_error(ret < 0, "I2C_RDWR failed");
	
	return val;
}

void write_power(int fd, uint8_t reg, uint8_t val)
{
	int ret;
	uint8_t buf[] = {reg, val};

	struct i2c_msg msgs[] = {
		{ POWER_ADDR, 0, 2, buf },
	};

//	printf("wr 0x%02hhx: %02hhx\n", reg, val);

	struct i2c_rdwr_ioctl_data msg = {
		.msgs = msgs,
		.nmsgs = sizeof(msgs) / sizeof(msgs[0])
	};

	ret = ioctl(fd, I2C_RDWR, &msg);
	syscall_error(ret < 0, "I2C_RDWR failed");
}

void update_power(int fd, uint8_t reg, uint8_t mask, uint8_t val)
{
	uint8_t tmp;
	
	tmp = read_power(fd, reg);
	tmp &= ~mask;
	tmp |= val & mask;
	write_power(fd, reg, tmp);
}

// bits 4-0 are mapped to gpio4 - gpio0
//#define MFP_CTL0 0x51
//#define MFP_CTL1 0x52
//#define GPIO_INEN 0x53
//#define GPIO_OUTEN 0x54
//#define GPIO_DATA 0x55

#define BATVADC_DAT_L 0xa2
#define BATVADC_DAT_H 0xa3
#define BATIADC_DAT_L 0xa4
#define BATIADC_DAT_H 0xa5
#define BATOCV_DAT_L 0xa8
#define BATOCV_DAT_H 0xa9

// in mV
unsigned get_bat_voltage(int fd)
{
	unsigned l = read_power(fd, BATVADC_DAT_L);
	unsigned h = read_power(fd, BATVADC_DAT_H);
	
	if (h & 0x20)
		return 2600 - ((~l & 0xff) + ((~h & 0x1f) << 8) + 1) * 1000 / 3724;

	return 2600 + (l + (h << 8)) * 1000 / 3724;
}

int get_bat_current(int fd)
{
	unsigned l = read_power(fd, BATIADC_DAT_L);
	unsigned h = read_power(fd, BATIADC_DAT_H);
	
	if (h & 0x20)
		return - (int)((~l & 0xff) + ((~h & 0x1f) << 8) + 1) * 1000 / 1341;

	return (l + (h << 8)) * 1000 / 1341;
}

unsigned get_bat_oc_voltage(int fd)
{
	unsigned l = read_power(fd, BATOCV_DAT_L);
	unsigned h = read_power(fd, BATOCV_DAT_H);
	
	if (h & 0x20)
		return 2600 - ((~l & 0xff) + ((~h & 0x1f) << 8) + 1) * 1000 / 3724;

	return 2600 + (l + (h << 8)) * 1000 / 3724;
}

enum {
	POWER_CHARGER_ENABLED,
	POWER_VOUT_ENABLED,
	POWER_VOUT_AUTO,
};

#define SYS_CTL0 0x01
#define SYS_CTL1 0x02
#define SYS_CTL2 0x0c
#define SYS_CTL3 0x03
#define SYS_CTL4 0x04
#define SYS_CTL5 0x07

#define Charger_CTL1 0x22
#define Charger_CTL2 0x24
#define CHG_DIG_CTL4 0x25

void power_setup(int fd, unsigned flags)
{
	update_power(fd, SYS_CTL1, 0x03, 0x00); // disable automatic control based on load detection
	update_power(fd, SYS_CTL0, 0x1e, BIT(1) | BIT(2)); // 2=boost 1=charger enable
	update_power(fd, SYS_CTL3, BIT(5), 0); // disable "2x key press = shutdown" function
	update_power(fd, SYS_CTL4, BIT(5), 0); // disable "VIN pull out -> VOUT auto-enable" function
	
	update_power(fd, CHG_DIG_CTL4, 0x1f, 15); // set charging current (in 100mA steps)
}

#define READ0 0x71
#define READ1 0x72
#define READ2 0x77

const char* get_chg_status_text(uint8_t s)
{
	switch (s) {
	case 0: return "Idle";
	case 1: return "Trickle charge";
	case 2: return "Constant current phase";
	case 3: return "Constant voltage phase";
	case 4: return "Constant voltage stop";
	case 5: return "Full";
	case 6: return "Timeout";
	default: return "Unknown";
	}
}

void power_status(int fd)
{
	uint8_t r0 = read_power(fd, READ0);
	uint8_t r1 = read_power(fd, READ1);
	uint8_t r2 = read_power(fd, READ2);

	printf("Charger: %s (%s%s%s%s%s%s%s)\n", get_chg_status_text((r0 >> 5) & 0x7),
		r0 & BIT(4) ? " chg_op" : "",
		r0 & BIT(3) ? " chg_end" : "",
		r0 & BIT(2) ? " cv_timeout" : "",
		r0 & BIT(1) ? " chg_timeout" : "",
		r0 & BIT(0) ? " trickle_timeout" : "",
		r1 & BIT(6) ? " VIN overvoltage" : "",
		r1 & BIT(5) ? " <= 75mA load" : ""
	);

	printf("Button: %02hhx (%s%s%s%s)\n", r2,
		r2 & BIT(3) ? " btn_press" : " btn_not_press",
		r2 & BIT(2) ? " double_press" : "",
		r2 & BIT(1) ? " long_press" : "",
		r2 & BIT(0) ? " short_press" : ""
	);

	printf("0x70: %02hhx\n", read_power(fd, 0x70));

	update_power(fd, READ2, 0x7, 0x7);
}

/*
- Independent control of
  - Boost (5V VOUT to power Pinephone)
  - Battery Charger

- Optional automatic power on when load is inserted
- Optional auto enable of VOUT when disconnecting VIN (reg 0x04)

- Optiobal automatic shutdown when VOUT has light load (customizable via reg
  0x0c, min. is 100mA, shutdown lastsa 8-64s (see reg 0x04))
  
- Charger_CTL1 0x22
  - Control of charging current based on VOUT undervoltage (it tries
    to keep VOUT in a certain range by reducing load on VIN by
    decreasing charging current?)

- Battery type selection 4.2/4.3/4.35V
  - + extra margin 0-42mV during constant voltage phase?
  - External (via VSET pin) or internal setting (via reg 0x24)

- Charging current selection (100mA - 2.3A ?)

- Charging status register
  - charging state - idle, trickle, constant voltage/current phase, full,
    timeout
  - LED heavey load indication
  - VIN overvoltage indication (> 5.6V)

- Button press status
  - current state: UP/DOWN
  - long press
  - short press

GPIO:

- KEY input
  - Long press button time selection 1-4s
  - Enable/disable 2x short press shutdown function
- L3/L4 function selection:
  - GPIO0/1
  - normal function
- LIGHT pin function selection:
  - GPIO2
  - VREF
  - WLED
- VSET
  - VSET (normal function to select battery voltage via PIN setting)
  - GPIO4
- RSET
  - GPIO3
  - battery internal resistance selection via resistor on the RSET pin

- separate input/output enable register for all 5 GPIOs
- GPIO data register to read/write values to pins

ADC:

- 14 bit two register VBAT, IBAT, VBAT_OCV readings
*/

int main(int ac, char* av[])
{
	int fd, ret;

	fd = pogo_i2c_open();

	printf("V=%u mV (OCV %u mV)   I=%d mA\n", get_bat_voltage(fd), get_bat_oc_voltage(fd), get_bat_current(fd));

//	uint8_t v = read_power(fd, 2);
//	printf("%02hhx\n", v);
	
	return 0;
}
