/*
 * Pinephone keyboard I2C debugging tool.
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
	uint8_t addr = REG_DEBUG_LOG;
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

int main(int ac, char* av[])
{
	iic_fd = pogo_i2c_open();

	uint8_t cmd[] = {REG_SYS_COMMAND_SELFTEST};
	wr_buf(REG_SYS_COMMAND, cmd, sizeof cmd);

	while (1) {
		dump_log();
		usleep(10000);
	}

	return 0;
}
