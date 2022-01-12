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

#include "common.c"
#include "firmware/registers.h"

/*
 * - Independent control of
 *   - Boost (5V VOUT to power Pinephone)
 *   - Battery Charger
 *
 * - Optional automatic power on when load is inserted
 * - Optional auto enable of VOUT when disconnecting VIN (reg 0x04)
 *
 * - Optiobal automatic shutdown when VOUT has light load (customizable via reg
 *   0x0c, min. is 100mA, shutdown lastsa 8-64s (see reg 0x04))
 *
 * - Charger_CTL1 0x22
 *   - Control of charging current based on VOUT undervoltage (it tries
 *     to keep VOUT in a certain range by reducing load on VIN by
 *     decreasing charging current?)
 *
 * - Battery type selection 4.2/4.3/4.35V
 *   - + extra margin 0-42mV during constant voltage phase?
 *   - External (via VSET pin) or internal setting (via reg 0x24)
 *
 * - Charging current selection (100mA - 2.3A ?)
 *
 * - Charging status register
 *   - charging state - idle, trickle, constant voltage/current phase, full,
 *     timeout
 *   - LED heavey load indication
 *   - VIN overvoltage indication (> 5.6V)
 *
 * - Button press status
 *   - current state: UP/DOWN
 *   - long press
 *   - short press
 *
 * GPIO:
 *
 * - KEY input
 *   - Long press button time selection 1-4s
 *   - Enable/disable 2x short press shutdown function
 * - L3/L4 function selection:
 *   - GPIO0/1
 *   - normal function
 * - LIGHT pin function selection:
 *   - GPIO2
 *   - VREF
 *   - WLED
 * - VSET
 *   - VSET (normal function to select battery voltage via PIN setting)
 *   - GPIO4
 * - RSET
 *   - GPIO3
 *   - battery internal resistance selection via resistor on the RSET pin
 *
 * - separate input/output enable register for all 5 GPIOs
 * - GPIO data register to read/write values to pins
 *
 * ADC:
 *
 * - 14 bit two register VBAT, IBAT, VBAT_OCV readings
*/

#define SYS_CTL0 0x01
#define SYS_CTL1 0x02
#define SYS_CTL2 0x0c
#define SYS_CTL3 0x03
#define SYS_CTL4 0x04
#define SYS_CTL5 0x07

#define Charger_CTL1 0x22
#define Charger_CTL2 0x24
#define CHG_DIG_CTL4 0x25
#define CHG_DIG_CTL4_2 0x26

#define READ0 0x71
#define READ1 0x72
#define READ2 0x77

// bits 4-0 are mapped to gpio4 - gpio0
#define MFP_CTL0 0x51
#define MFP_CTL1 0x52
#define GPIO_INEN 0x53
#define GPIO_OUTEN 0x54
#define GPIO_DATA 0x55

#define BATVADC_DAT_L 0xa2
#define BATVADC_DAT_H 0xa3
#define BATIADC_DAT_L 0xa4
#define BATIADC_DAT_H 0xa5
#define BATOCV_DAT_L 0xa8
#define BATOCV_DAT_H 0xa9

uint8_t read_power(int fd, uint8_t reg)
{
	int ret;
	uint8_t val;

	// initiate read of data from the charger
	uint8_t buf[4] = { REG_SYS_CHG_ADDR, reg, 0xAA, REG_SYS_COMMAND_CHG_READ };
	struct i2c_msg msgs[] = {
		{ KB_ADDR, 0, 4, buf },
	};

	struct i2c_rdwr_ioctl_data msg = {
		.msgs = msgs,
		.nmsgs = sizeof(msgs) / sizeof(msgs[0])
	};

	ret = ioctl(fd, I2C_RDWR, &msg);
	syscall_error(ret < 0, "I2C_RDWR failed");

	for (int i = 0; i < 5; i++) {
		usleep(700);

		// read the result
		uint8_t buf2[1] = { REG_SYS_CHG_DATA, };
		uint8_t buf3[2] = { };
		struct i2c_msg msgs2[] = {
			{ KB_ADDR, 0, 1, buf2 },
			{ KB_ADDR, I2C_M_RD, sizeof(buf3), buf3 },
		};

		struct i2c_rdwr_ioctl_data msg2 = {
			.msgs = msgs2,
			.nmsgs = sizeof(msgs2) / sizeof(msgs2[0])
		};

		ret = ioctl(fd, I2C_RDWR, &msg2);
		syscall_error(ret < 0, "I2C_RDWR failed");

//		debug("rd %02x %02x\n", buf3[0], buf3[1]);

		if (buf3[1] == REG_SYS_COMMAND_CHG_READ)
			continue;
		
		if (buf3[1] == 0)
			return buf3[0];
		if (buf3[1] == 0xff)
			error("Proxy read failed with %x\n", buf3[1]);
	}

	error("Proxy read timed out\n");
	return 0;
}

void write_power(int fd, uint8_t reg, uint8_t val)
{
	int ret;

	uint8_t buf[4] = { REG_SYS_CHG_ADDR, reg, val, REG_SYS_COMMAND_CHG_WRITE };
	struct i2c_msg msgs[] = {
		{ KB_ADDR, 0, 4, buf },
	};

//	debug("wr 0x%02hhx: %02hhx\n", reg, val);

	struct i2c_rdwr_ioctl_data msg = {
		.msgs = msgs,
		.nmsgs = sizeof(msgs) / sizeof(msgs[0])
	};

	ret = ioctl(fd, I2C_RDWR, &msg);
	syscall_error(ret < 0, "I2C_RDWR failed");

	for (int i = 0; i < 5; i++) {
		usleep(700);

		// read the result
		uint8_t buf2[1] = { REG_SYS_COMMAND, };
		uint8_t buf3[1] = { };
		struct i2c_msg msgs2[] = {
			{ KB_ADDR, 0, 1, buf2 },
			{ KB_ADDR, I2C_M_RD, sizeof(buf3), buf3 },
		};

		struct i2c_rdwr_ioctl_data msg2 = {
			.msgs = msgs2,
			.nmsgs = sizeof(msgs2) / sizeof(msgs2[0])
		};

		ret = ioctl(fd, I2C_RDWR, &msg2);
		syscall_error(ret < 0, "I2C_RDWR failed");

		if (buf3[0] == REG_SYS_COMMAND_CHG_WRITE)
			continue;
		
		if (buf3[0] == 0)
			return;
		if (buf3[0] == 0xff)
			error("Proxy write failed with %x\n", buf3[0]);
	}
}

void update_power(int fd, uint8_t reg, uint8_t mask, uint8_t val)
{
	uint8_t tmp;

	tmp = read_power(fd, reg);
	tmp &= ~mask;
	tmp |= val & mask;
	write_power(fd, reg, tmp);
}

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
	uint8_t s0 = read_power(fd, SYS_CTL0);

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

	// this has some nice undocummneted status bits
	printf("0x70: %02hhx\n", read_power(fd, 0x70));

	update_power(fd, READ2, 0x7, 0x7);
}

// dump registers

struct bitinfo {
	const char* name;
	uint8_t shift;
	uint8_t len;
	void (*fmt)(char* out, size_t out_size, uint8_t val);
};

struct reginfo {
	uint8_t reg;
	const char* name;
	struct bitinfo* bits;
};

#define REG_START(addr, name) [addr] = { addr, name, (struct bitinfo[]){
#define REG_END {} }},
#define REG_BITS(name, s, l) { #name, s, l },
#define REG(addr, name) REG_START(addr, name) { name, 0, 8 }, REG_END
#define REG_SIMPLE(n) REG(n, #n)

struct reginfo regs[256] = {
	REG_START(SYS_CTL0, "SYS_CTL0")
		REG_BITS(CHARGER_EN, 1, 1)
		REG_BITS(BOOST_EN, 2, 1)
		REG_BITS(LIGHT_EN, 3, 1)
		REG_BITS(FLASHLIGHT_DET_EN, 4, 1)
	REG_END
	REG_START(SYS_CTL1, "SYS_CTL1")
		REG_BITS(AUTO_POWERON_ON_VIN_INSERT_EN, 0, 1)
		REG_BITS(LIGHT_LOAD_AUTO_SHUTDOWN_EN, 1, 1)
	REG_END
	REG_START(SYS_CTL2, "SYS_CTL2")
		REG_BITS(LIGHT_SHUTDOWN_CURRENT, 3, 5)
	REG_END
	REG_START(SYS_CTL3, "SYS_CTL3")
		REG_BITS(DOUBLE_PRESS_SHUTDOWN_EN, 5, 1)
		REG_BITS(LONG_PRESS_TIME, 6, 2)
	REG_END
	REG_START(SYS_CTL4, "SYS_CTL4")
		REG_BITS(SHUTDOWN_TIME, 6, 2)
		REG_BITS(VIN_PULLOUT_BOOST_ON, 5, 1)
	REG_END
	REG_START(SYS_CTL5, "SYS_CTL5")
		REG_BITS(NTC_EN, 6, 1)
		REG_BITS(FLASH_LED_EN_0_LONG_PRESS_1_DOUBLE_PRESS, 1, 1)
		REG_BITS(SHUTDOWN_1_LONG_PRESS_0_DOUBLE_PRESS, 0, 1)
	REG_END
	REG_START(Charger_CTL1, "Charger_CTL1")
		REG_BITS(UV_LOOP, 2, 2)
	REG_END
	REG_START(Charger_CTL2, "Charger_CTL2")
		REG_BITS(BAT_TYPE, 5, 2)
		REG_BITS(CV_PRESSURE, 1, 2)
	REG_END
	REG_START(CHG_DIG_CTL4_2, "CHG_DIG_CTL4_2")
		REG_BITS(BAT_TYPE_SEL_1_VSET_PIN_0_REGISTER, 6, 1)
	REG_END
	REG_START(CHG_DIG_CTL4, "CHG_DIG_CTL4")
		REG_BITS(CHG_CURRENT, 0, 5)
	REG_END
	REG_SIMPLE(MFP_CTL0)
	REG_SIMPLE(MFP_CTL1)
	REG_SIMPLE(GPIO_INEN)
	REG_SIMPLE(GPIO_OUTEN)
	REG_SIMPLE(GPIO_DATA)
	REG_SIMPLE(BATVADC_DAT_L)
	REG_SIMPLE(BATVADC_DAT_H)
	REG_SIMPLE(BATOCV_DAT_L)
	REG_SIMPLE(BATOCV_DAT_H)
	REG_SIMPLE(BATIADC_DAT_L)
	REG_SIMPLE(BATIADC_DAT_H)

	REG_START(0x70, "READ_70")
//		REG_BITS(DISCHARGING, 2, 1)
		REG_BITS(VOUT_BOOST, 2, 1)
		REG_BITS(CHARGING, 3, 1)
		REG_BITS(VIN_INSERTED, 4, 1)
		REG_BITS(VIN_NOT_INSERTED, 5, 1)
	REG_END

	REG_SIMPLE(READ0)
	REG_SIMPLE(READ1)
	REG_SIMPLE(READ2)
};

static void dump_regs(int fd)
{
	for (int addr = 0; addr <= 0xff; addr++) {
		struct reginfo* ri = &regs[addr];

		uint8_t val = read_power(fd, addr);
		if (val == 0 && !ri->name)
			continue;

		printf("%02x: %02hhx", addr, val);
		if (ri->name) {
			printf(" (%s)", ri->name);

			for (int i = 0; ri->bits[i].name; i++) {
				struct bitinfo* bi = &ri->bits[i];
				uint8_t bval = (val >> bi->shift) & (((1u) << (bi->len)) - 1);

				printf(" %s=0x%02hhx", bi->name, bval);
			}
		}
		printf("\n");
	}
}

static void usage(void)
{
	printf(
	       "Usage: ppkb-charger-ctl [--verbose] [--help]\n"
	       "                        [<info|power-on|power-off|charger-on|charger-off|auto>...]\n"
	       "\n"
	       "Options:\n"
	       "  -c, --current         Change the charging current (mA).\n"
	       "  -v, --verbose         Show details of what's going on.\n"
	       "  -h, --help            This help.\n"
	       "\n"
	       "Commands:\n"
	       "  info        Display information about the current state of the charger chip.\n"
	       "  power-on    Power on VOUT (boost output to the phone and keyboard).\n"
	       "  power-off   Power off VOUT.\n"
	       "  charger-on  Start charging the battery.\n"
	       "  charger-off Stop charging the battery.\n"
	       "  auto        Switch to automatic control of VOUT/Charging (default configuration).\n"
	       "  dump        Dump charger chip registers.\n"
	       "\n"
	       "Pinephone keyboard charger control tool " VERSION "\n"
	       "Written by Ondrej Jirman <megi@xff.cz>, 2021\n"
	       "Licensed under GPLv3, see https://xff.cz/git/pinephone-keyboard/ for\n"
	       "more information.\n"
	);

	exit(2);
}

int main(int ac, char* av[])
{
	int fd, ret;
	int current = -1;

	while (1) {
		int option_index = 0;
		struct option long_options[] = {
			{ "current", required_argument, 0, 'c' },
			{ "verbose", no_argument,       0, 'v' },
			{ "help",    no_argument,       0, 'h' },
			{ 0,         0,                 0,  0  }
		};

		int c = getopt_long(ac, av, "c:vh", long_options, &option_index);
		if (c == -1)
			break;

		switch (c) {
		case 'c':
			errno = 0;
			char* next = NULL;
			current = strtol(optarg, &next, 10);
			if (errno || next == optarg) {
				printf("ERROR: Can't parse --current %s\n\n", optarg);
				usage();
			}
			break;
		case 'v':
			verbose = 1;
			break;
		case 'h':
		case '?':
		default:
			usage();
			break;
		}
	}

	if (optind == ac)
		usage();

	if (current > 2000) {
		printf("ERROR: --current %d too big\n\n", current);
		usage();
	}

	if (current != -1 && current < 100) {
		printf("ERROR: --current %d too small\n\n", current);
		usage();
	}

	fd = pogo_i2c_open();

	if (current != -1) {
		//update_power(fd, SYS_CTL0, BIT(2), BIT(2));
	}

	int lfd = gpio_setup_pl12(GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_BIAS_PULL_UP | /*GPIO_V2_LINE_FLAG_ACTIVE_HIGH |*/ GPIO_V2_LINE_FLAG_EDGE_FALLING);

//	update_power(fd, SYS_CTL1, 0x03, 0x00); // disable automatic control based on load detection
//	update_power(fd, SYS_CTL0, 0x1e, BIT(1) | BIT(2)); // 2=boost 1=charger enable
//	update_power(fd, SYS_CTL3, BIT(5), 0); // disable "2x key press = shutdown" function
//	update_power(fd, SYS_CTL4, BIT(5), 0); // disable "VIN pull out -> VOUT auto-enable" function
//	update_power(fd, CHG_DIG_CTL4, 0x1f, 15); // set charging current (in 100mA steps)

	for (int i = optind; i < ac; i++) {
		if (!strcmp(av[i], "power-on")) {
			update_power(fd, SYS_CTL0, BIT(2), BIT(2));
		} else if (!strcmp(av[i], "power-off")) {
			update_power(fd, SYS_CTL0, BIT(2), 0);
			update_power(fd, SYS_CTL1, 0x03, 0x00); // disable automatic control based on load detection
			update_power(fd, SYS_CTL4, BIT(5), 0); // disable "VIN pull out -> VOUT auto-enable" function
		} else if (!strcmp(av[i], "charger-on")) {
			update_power(fd, SYS_CTL0, BIT(1), BIT(1));
		} else if (!strcmp(av[i], "charger-off")) {
			update_power(fd, SYS_CTL0, BIT(1), 0);
		} else if (!strcmp(av[i], "info")) {
			power_status(fd);
			printf("V=%u mV (OCV %u mV) I=%d mA\n",
			       get_bat_voltage(fd),
			       get_bat_oc_voltage(fd),
			       get_bat_current(fd));
		} else if (!strcmp(av[i], "dump")) {
			dump_regs(fd);
		} else if (!strcmp(av[i], "auto")) {
			// enable automatic control based on load detection
			update_power(fd, SYS_CTL1, 0x03, 0x03);
			// disable "2x key press = shutdown" function
			update_power(fd, SYS_CTL3, BIT(5), BIT(5));
			// disable "VIN pull out -> VOUT auto-enable" function
			update_power(fd, SYS_CTL4, BIT(5), BIT(5));
		} else {
			printf("ERROR: Unknown command: %s\n\n", av[i]);
			usage();
		}
	}

	return 0;
}
