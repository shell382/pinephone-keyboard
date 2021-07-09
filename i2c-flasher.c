/*
 * Pinephone keyboard I2C flashing tool.
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

static int iic_fd = -1;

static void dump_log(void)
{
	int ret;
	uint8_t addr = 0xff;
	uint8_t buf[64];
	struct i2c_msg msgs[] = {
		{ KB_ADDR, 0, 1, &addr }, // set 0xff address
		{ KB_ADDR, I2C_M_RD, sizeof(buf), buf },
	};

	struct i2c_rdwr_ioctl_data msg = {
		.msgs = msgs,
		.nmsgs = sizeof(msgs) / sizeof(msgs[0])
	};

	ret = ioctl(iic_fd, I2C_RDWR, &msg);
	syscall_error(ret < 0, "I2C_RDWR failed");

	int i;
	for (i = 0; i < sizeof(buf) && buf[i]; i++);

	if (i > 0)
		xwrite(1, buf, i);
}

static void wr_buf(uint8_t addr, uint8_t* buf, size_t size)
{
	int ret;
	uint8_t mbuf[size + 1];

	mbuf[0] = addr;
	memcpy(mbuf + 1, buf, size);

	struct i2c_msg msgs[] = {
		{ KB_ADDR, 0, size + 1, mbuf },
	};

	struct i2c_rdwr_ioctl_data msg = {
		.msgs = msgs,
		.nmsgs = sizeof(msgs) / sizeof(msgs[0])
	};

	debug("WR[%02hhx]:", addr);
	for (int i = 0; i < size; i++)
		debug(" %02hhx", buf[i]);
	debug("\n");

	ret = ioctl(iic_fd, I2C_RDWR, &msg);
	syscall_error(ret < 0, "I2C_RDWR failed");
}

static void rd_buf(uint8_t addr, uint8_t* buf, size_t size)
{
	int ret;
	struct i2c_msg msgs[] = {
		{ KB_ADDR, 0, 1, &addr },
		{ KB_ADDR, I2C_M_RD, size, buf },
	};

	struct i2c_rdwr_ioctl_data msg = {
		.msgs = msgs,
		.nmsgs = sizeof(msgs) / sizeof(msgs[0])
	};

	ret = ioctl(iic_fd, I2C_RDWR, &msg);
	syscall_error(ret < 0, "I2C_RDWR failed");

	debug("RD[%02hhx]:", addr);
	for (int i = 0; i < size; i++) {
		if (i % 8 == 0 && i > 0)
			debug("\n       ");
		debug(" %02hhx", buf[i]);
	}
	debug("\n");
}

static int rd_buf_nofail(uint8_t addr, uint8_t* buf, size_t size)
{
	int ret;
	struct i2c_msg msgs[] = {
		{ KB_ADDR, 0, 1, &addr },
		{ KB_ADDR, I2C_M_RD, size, buf },
	};

	struct i2c_rdwr_ioctl_data msg = {
		.msgs = msgs,
		.nmsgs = sizeof(msgs) / sizeof(msgs[0])
	};

	ret = ioctl(iic_fd, I2C_RDWR, &msg);

	if (ret == 2) {
		debug("RD[%02hhx]:", addr);
		for (int i = 0; i < size; i++) {
			if (i % 8 == 0 && i > 0)
				debug("\n       ");
			debug(" %02hhx", buf[i]);
		}
		debug("\n");
	}

	return ret == 2 ? 0 : -1;
}

static uint8_t rd_reg(uint8_t addr)
{
	uint8_t reg;
	rd_buf(addr, &reg, 1);
	return reg;
}

static int wait_flash_cmd_done(uint8_t cmd)
{
	int ret;

	for (int i = 0; i < 10; i++) {
		uint8_t status;

		ret = rd_buf_nofail(REG_FLASH_CMD, &status, 1);
		if (ret == 0) {
			if (status == 0xffu)
				error("Flashing command 0x%02hhx failed", cmd);
			else if (status == 0x00)
				return 0;
		}

		usleep(5000);
	}
	
	error("Flashing command 0x%02hhx timed out", cmd);
}

static void read_rom_block(uint8_t* out, uint16_t addr)
{
	uint8_t read_rom_start[] = {
		addr & 0xff, // Addr L
		(addr >> 8) & 0xff, // Addr H
		0x00, // CRC-8
		REG_FLASH_UNLOCK_MAGIC, // Unlock
		REG_FLASH_CMD_READ_ROM, // Read ROM
	};

	wr_buf(REG_FLASH_ADDR_L, read_rom_start, sizeof read_rom_start);

	wait_flash_cmd_done(REG_FLASH_CMD_READ_ROM);

	rd_buf(REG_FLASH_DATA_START, out, 128);

	if (rd_reg(REG_FLASH_CRC8) != crc8(out, 128))
		error("CRC8 failure on ROM read");
}

static void write_rom_block(uint16_t addr, uint8_t* data)
{
	uint8_t write_rom_start[5] = {
		addr & 0xff, // Addr L
		(addr >> 8) & 0xff, // Addr H
		crc8(data, 128), // CRC-8
		REG_FLASH_UNLOCK_MAGIC, // Unlock
		REG_FLASH_CMD_WRITE_ROM, // Write ROM
	};

	wr_buf(REG_FLASH_DATA_START, data, 128);
	wr_buf(REG_FLASH_ADDR_L, write_rom_start, sizeof write_rom_start);

	usleep(5000);
	wait_flash_cmd_done(REG_FLASH_CMD_WRITE_ROM);
}

static void run_flash_cmd(uint8_t cmd)
{
	uint8_t cmd_data[] = {
		REG_FLASH_UNLOCK_MAGIC, // Unlock
		cmd, // Command
	};

	wr_buf(REG_FLASH_UNLOCK, cmd_data, sizeof cmd_data);

	usleep(10000);
	wait_flash_cmd_done(cmd);
}

static bool is_block_empty(uint8_t* buf)
{
	for (unsigned i = 0; i < 128; i++)
		if (buf[i] != 0xff)
			return false;

	return true;
}

static int is_kb_stock_connected(void)
{
	uint8_t devid[5];
	int ret;

	ret = rd_buf_nofail(REG_DEVID_K, devid, sizeof devid);
	if (ret)
		return 0;

	if (devid[REG_DEVID_K] != 'K' || devid[REG_DEVID_B] != 'B') // keyboard firmware magic
		return 0;
	if (!(devid[REG_FW_FEATURES] & REG_FW_FEATURES_STOCK_FW)) // stock firmware flag
		return 0;

	if (devid[REG_FW_REVISION] != 0x10)
		error("Unsupported stock pinephone keyboard firmware version %02hhx, expecting 0x10\n", devid[2]);
	if (!(devid[REG_FW_FEATURES] & REG_FW_FEATURES_FLASHING_MODE))
		error("Your stock pinephone keyboard firmware doesn't have flashing support\n");

	return 1;
}

static void usage(void)
{
	printf(
	       "Usage: ppkb-i2c-flasher [--rom-in <path>] [--rom-out <path>] [--verbose]\n"
	       "                        [--help] [<read|write|erase|info|reset>...]\n"
	       "\n"
	       "Options:\n"
	       "  -i, --rom-in <path>   Specify path to binary file you want to flash.\n"
	       "  -o, --rom-out <path>  Specify path where you want to store the contents\n"
	       "                        of code ROM read from the device.\n"
	       "  -s, --size <size>     Specify how many bytes of code rom to flash\n"
	       "                        starting from offset 0x4000 in the rom file.\n"
	       "  -e, --entry <manual|i2c|none>\n"
	       "                        Specify how to enter the stock firmware:\n"
	       "                        - manual: Ask the user to power-cycle the keyboard\n"
	       "                        - i2c: Send I2C command to make supporting user\n"
	       "                        - none: Assume stock firmware is already running\n"
	       "  -v, --verbose         Show details of what's going on.\n"
	       "  -h, --help            This help.\n"
	       "\n"
	       "Commands:\n"
	       "  info      Display information about the current firmware.\n"
	       "  read      Read ROM from the device to --rom-out file.\n"
	       "  write     Flash ROM file to the device from --rom-in.\n"
	       "  erase     Erase the user firmware.\n"
	       "  reset     Perform software reset of the MCU.\n"
	       "  usbiap    Restart to USB IAP mode.\n"
	       "\n"
	       "Format of the ROM files is a flat binary. Only the part of it starting\n"
	       "from 0x4000 will be flashed. Use -s to specify how many bytes to write.\n"
	       "The stock firmware between 0x2000 and 0x4000 will be preserved.\n"
	       "\n"
	       "Pinephone keyboard I2C flashing tool " VERSION "\n"
	       "Written by Ondrej Jirman <megi@xff.cz>, 2021\n"
	       "Licensed under GPLv3, see https://xff.cz/git/pinephone-keyboard/ for\n"
	       "more information.\n"
	);

	exit(2);
}

int main(int ac, char* av[])
{
	char* rom_in = NULL;
	char* rom_out = NULL;
	char* entry_type = "i2c";
	int size = 0x1200;
	int ret;

	while (1) {
		int option_index = 0;
		struct option long_options[] = {
			{ "rom-in",  required_argument, 0, 'i' },
			{ "rom-out", required_argument, 0, 'o' },
			{ "size",    required_argument, 0, 's' },
			{ "entry",   required_argument, 0, 'e' },
			{ "verbose", no_argument,       0, 'v' },
			{ "help",    no_argument,       0, 'h' },
			{ 0,         0,                 0,  0  }
		};

		int c = getopt_long(ac, av, "i:o:s:e:vh", long_options, &option_index);
		if (c == -1)
			break;

		switch (c) {
		case 'o':
			rom_out = strdup(optarg);
			break;
		case 'i':
			rom_in = strdup(optarg);
			break;
		case 'e':
			entry_type = strdup(optarg);
			break;
		case 's':
			if (strstr(optarg, "0x") == optarg) {
				errno = 0;
				char* next = NULL;
				size = strtol(optarg + 2, &next, 16);
				if (errno || next == optarg + 2) {
					printf("ERROR: Can't parse --size %s\n\n", optarg);
					usage();
				}
			} else {
				errno = 0;
				char* next = NULL;
				size = strtol(optarg, &next, 10);
				if (errno || next == optarg) {
					printf("ERROR: Can't parse --size %s\n\n", optarg);
					usage();
				}
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

	if (size < 128) {
		printf("ERROR: --size 0x%04x too small\n\n", size);
		usage();
	}

	if (size > 0x4000) {
		printf("ERROR: --size 0x%04x too large\n\n", size);
		usage();
	}

	if (size % 128 != 0) {
		printf("ERROR: --size 0x%04x is not multiple of 128\n\n", size);
		usage();
	}

	for (int i = optind; i < ac; i++) {
		if (!strcmp(av[i], "read")) {
			if (!rom_out) {
				printf("ERROR: You must specify target file to write rom contents to via --rom-out\n\n");
				usage();
			}
		} else if (!strcmp(av[i], "write")) {
			if (!rom_in) {
				printf("ERROR: You must source file for flashing via --rom-in\n\n");
				usage();
			}
		} else if (!strcmp(av[i], "info")) {
			;
		} else if (!strcmp(av[i], "reset")) {
			;
		} else if (!strcmp(av[i], "erase")) {
			;
		} else if (!strcmp(av[i], "usbiap")) {
			;
		} else {
			printf("ERROR: Unknown command: %s\n\n", av[i]);
			usage();
		}
	}

	printf("Opening keyboard I2C device\n");
	iic_fd = pogo_i2c_open();

	if (!is_kb_stock_connected()) {
		if (!strcmp(entry_type, "i2c")) {
			// send MCU reset command
			uint8_t cmd[] = {REG_SYS_COMMAND_MCU_RESET};
			wr_buf(REG_SYS_COMMAND, cmd, sizeof cmd);

			// stock firmware should report itself quickly
			//
			// tell firmware to block enrty to user app (we have 1s
			// window to do this after reset)
			int i;
			for (i = 0; i < 10; i++) {
				if (is_kb_stock_connected()) {
					uint8_t cmd[] = {REG_SYS_USER_APP_BLOCK_MAGIC};
					wr_buf(REG_SYS_USER_APP_BLOCK, cmd, sizeof cmd);
					break;
				}

				usleep(250000);
			}

			if (i == 10)
				error("Reset command issued over I2C failed, stock firmware failed to report itself within 2.5s");
		} else if (!strcmp(entry_type, "manual")) {
			printf("Please power off the keyboard by holding the keyboard power key for > 12s, then release the power key and press it shortly, once, to power it on again.\n");

			while (true) {
				if (is_kb_stock_connected()) {
					uint8_t cmd[] = {REG_SYS_USER_APP_BLOCK_MAGIC};
					wr_buf(REG_SYS_USER_APP_BLOCK, cmd, sizeof cmd);
					break;
				}

				usleep(250000);
			}
		} else if (!strcmp(entry_type, "none")) {
			error("Stock pinephone keyboard firmware not detected running on the keyboard\n");
		} else {
			error("Unknown entry method %s", entry_type);
		}

		// if after 1s the stock firmware is still running,
		// everything is ok
		usleep(1000000);
		if (!is_kb_stock_connected())
			error("Failed to block the user app from running");
	}

	for (int i = optind; i < ac; i++) {
		if (!strcmp(av[i], "read")) {
			printf("Reading code ROM\n");
			uint8_t rom[0x8000];
			for (unsigned i = 0; i < sizeof(rom); i += 128)
				read_rom_block(rom + i, i);

			int fd = open(rom_out, O_WRONLY | O_CREAT | O_TRUNC, 0666);
			if (fd >= 0) {
				ssize_t wr = write(fd, rom, sizeof rom);
				syscall_error(wr != sizeof(rom), "write failed");
				close(fd);
			}
		} else if (!strcmp(av[i], "write")) {
			int fd;

			uint8_t rom[0x8000];
			memset(rom, 0xff, sizeof rom);

			fd = open(rom_in, O_RDONLY);
			syscall_error(fd < 0, "open(%s) failed", rom_in);
			ssize_t len = read(fd, rom, 0x8000);
			syscall_error(len < 0, "read failed");
			close(fd);
			if (len != 0x8000)
				error("Invalid ROM file (%s) size (%d), must be 32768 bytes", rom_in, (int)len);

			printf("Flashing code ROM\n");
			for (unsigned i = 0x4000; i < 0x4000 + size; i += 128)
				write_rom_block(i, rom + i);

			uint8_t rd_rom[0x8000];
			for (unsigned i = 0x4000; i < 0x4000 + size; i += 128) {
				read_rom_block(rd_rom + i, i);

				if (memcmp(rd_rom + i, rom + i, 128)) {
					printf("WARNING: Block 0x%04x write failed, retrying...\n", i);
					error("Retries disabled");
				}
			}

			printf("Finishing flashing\n");
			run_flash_cmd(REG_FLASH_CMD_COMMIT);
		} else if (!strcmp(av[i], "info")) {
			uint8_t devid[5];

			rd_buf(0x00, devid, sizeof devid);

			printf("DEVID Register dump:\n");
			for (int i = 0; i < sizeof(devid); i++)
				printf("0x%02x: 0x%02hhx\n", i, devid[i]);
		} else if (!strcmp(av[i], "reset")) {
			printf("Restarting the MCU\n");

			// send MCU reset command
			uint8_t cmd[] = {REG_SYS_COMMAND_MCU_RESET};
			wr_buf(REG_SYS_COMMAND, cmd, sizeof cmd);
		} else if (!strcmp(av[i], "usbiap")) {
			printf("Restarting to USB IAP mode, if you don't have USB interface soldered on, you'll have to power-cycle the keyboard to get out of this flashing mode.\n");

			// send MCU reset command
			uint8_t cmd[] = {REG_SYS_COMMAND_USB_IAP};
			wr_buf(REG_SYS_COMMAND, cmd, sizeof cmd);
		} else if (!strcmp(av[i], "erase")) {
			run_flash_cmd(REG_FLASH_CMD_ERASE_ROM);
		} else {
			printf("ERROR: Unknown command: %s\n\n", av[i]);
			usage();
		}
	}

	return 0;
}
