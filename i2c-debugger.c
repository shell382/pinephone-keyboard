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

void dump_log(int fd)
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

	ret = ioctl(fd, I2C_RDWR, &msg);
	if (ret < 0) {
		printf("ERROR: (%"PRIu64") I2C_RDWR failed (%d)\n", time_abs(), errno);
		fflush(stdout);
		usleep(50000);
		return;
	}

	int i;
	for (i = 0; i < sizeof(buf) && buf[i]; i++);
	
	if (i > 0)
		xwrite(1, buf, i);
}

int main(int ac, char* av[])
{
	int fd, ret;

	fd = pogo_i2c_open();

	while (1) {
		dump_log(fd);
		usleep(10000);
	}

	return 0;
}
