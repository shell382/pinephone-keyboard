/*
 * Pinephone keyboard userspace input device daemon.
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

#include "common.c"
#include "firmware/registers.h"

#include <linux/input.h>
#include <linux/uinput.h>

#define MEGI_PROTO_BUG 1

int read_kb(int fd, uint8_t data[16])
{
	int ret;
	uint8_t b = 5;
	struct i2c_msg msgs[] = {
		{ KB_ADDR, 0, 1, &b },
		{ KB_ADDR, I2C_M_RD, 16, data },
	};

	struct i2c_rdwr_ioctl_data msg = {
		.msgs = msgs,
		.nmsgs = sizeof(msgs) / sizeof(msgs[0])
	};

	ret = ioctl(fd, I2C_RDWR, &msg);
	syscall_error(ret < 0, "I2C_RDWR failed");

	return ret == 1 ? 0 : -1;
}

int write_kb(int fd, uint8_t* data)
{
	int ret;
	struct i2c_msg msgs[] = {
		{ KB_ADDR, 0, 4, data },
	};

	struct i2c_rdwr_ioctl_data msg = {
		.msgs = msgs,
		.nmsgs = sizeof(msgs) / sizeof(msgs[0])
	};

	ret = ioctl(fd, I2C_RDWR, &msg);
	syscall_error(ret < 0, "I2C_RDWR failed");

	return ret == 1 ? 0 : -1;
}

#include "kmap.h"

int open_uinput_dev(void)
{
	int fd, ret;

	fd = open("/dev/uinput", O_WRONLY/* | O_NONBLOCK*/);
	syscall_error(fd < 0, "open(/dev/uinput) failed");

	struct uinput_setup setup = {
		.name = "ppkbd",
		.id = {
			.bustype = BUS_USB,
			.vendor = 0x1234,
			.product = 0x5678,
		},
	};

	ret = ioctl(fd, UI_SET_EVBIT, EV_KEY);
	syscall_error(ret < 0, "UI_SET_EVBIT failed");

	for (int i = 0; i < sizeof(used_keys) / sizeof(used_keys[0]); i++) {
		ret = ioctl(fd, UI_SET_KEYBIT, used_keys[i]);
		syscall_error(ret < 0, "UI_SET_KEYBIT failed");
	}

	ret = ioctl(fd, UI_DEV_SETUP, &setup);
	syscall_error(ret < 0, "UI_DEV_SETUP failed");

	ret = ioctl(fd, UI_DEV_CREATE);
	syscall_error(ret < 0, "UI_DEV_CREATE failed");

	//ioctl(fd, UI_DEV_DESTROY);
	//close(fd);

	return fd;
}

void emit_ev(int fd, int type, int code, int val)
{
	struct input_event ev = {
		.type = type,
		.code = code,
		.value = val,
	};

	ssize_t ret = write(fd, &ev, sizeof ev);
	syscall_error(ret < 0, "write event failed");
}

void print_bitmap(uint8_t* map)
{
	printf("\033[H");
	for (int r = 0; r < 6; r++) {
		if (r == 0) {
			printf("   C");
			for (int c = 0; c < 12; c++)
				printf("%-3d", c + 1);
			printf("\n");
		}

		printf("R%d", r + 1);
		for (int c = 0; c < 12; c++)
			printf("  %s", map[c] & (1u << r) ? "X" : ".");
		printf("\n");
	}
}

int get_index(int* keys, int len, int key)
{
	for (int i = 0; i < len; i++) {
		if (keys[i] == key)
			return i;
	}
	return -1;
}

int compact(int* keys, int len)
{
	int ckeys[len];
	int ci = 0;
	
	memset(ckeys, 0, len * sizeof(int));
	
	for (int i = 0; i < len; i++)
		if (keys[i])
			ckeys[ci++] = keys[i];

	memcpy(keys, ckeys, sizeof ckeys);
	return ci;
}

static int uinput_fd = -1;
static int pressed_keys[128]; // contains currently pressed phys_idxs in press order
static int pressed_count;
static int fn_mode = 0;
static int pine_mode = 0;

void on_press(uint8_t phys_idx)
{
	int key = keymap_base[phys_idx][0];
//	printf("press %02hhx %s\n", phys_idx, key ? key_names[key] : "");

	if (key == KEY_FN || key == KEY_LEFTMETA) {
		return;
	}

	//XXX: make sure fn/pine is the last presssed key prior to this one?
	int fn_idx = get_index(pressed_keys, pressed_count, 0x52);
	int pine_idx = get_index(pressed_keys, pressed_count, 0x31);

	if (key == KEY_ESC && (fn_mode || pine_mode)) {
		fn_mode = pine_mode = 0;
		return;
	}

	const int* keys = keymap_base[phys_idx];
	if (fn_idx >= 0 || fn_mode) {
		if (key == KEY_ESC) {
			fn_mode = 1;
			return;
		}

		keys = keymap_fn[phys_idx];
	} else if (pine_idx >= 0 || pine_mode) {
		if (key == KEY_ESC) {
			pine_mode = 1;
			return;
		}

		keys = keymap_pine[phys_idx];
	}
	
	if (!keys[0])
		keys = keymap_base[phys_idx];

	if (keys[0]) {
		emit_ev(uinput_fd, EV_KEY, keys[0], 1);
		emit_ev(uinput_fd, EV_SYN, SYN_REPORT, 0);
	}
		
	if (keys[1]) {
		emit_ev(uinput_fd, EV_KEY, keys[1], 1);
		emit_ev(uinput_fd, EV_SYN, SYN_REPORT, 0);
	}
}

void on_release(uint8_t phys_idx)
{
	int key = keymap_base[phys_idx][0];
//	printf("release %02hhx %s\n", phys_idx, key ? key_names[key] : "");

	if (key == KEY_FN || key == KEY_LEFTMETA) {
		return;
	}

	int fn_idx = get_index(pressed_keys, pressed_count, 0x52);
	int pine_idx = get_index(pressed_keys, pressed_count, 0x31);

	const int* keys = keymap_base[phys_idx];
	if (fn_idx >= 0 || fn_mode) {
		keys = keymap_fn[phys_idx];
	} else if (pine_idx >= 0 || pine_mode) {
		keys = keymap_pine[phys_idx];
	}

	if (!keys[0])
		keys = keymap_base[phys_idx];

	if (keys[0]) {
		emit_ev(uinput_fd, EV_KEY, keys[0], 0);
		emit_ev(uinput_fd, EV_SYN, SYN_REPORT, 0);
	}
		
	if (keys[1]) {
		emit_ev(uinput_fd, EV_KEY, keys[1], 0);
		emit_ev(uinput_fd, EV_SYN, SYN_REPORT, 0);
	}
}

void update_keys(uint8_t* map)
{
	// physical indices of pressed keys reported over I2C
	int keys[128];
	int n_keys = 0;
	
	for (int c = 0; c < 12; c++) {
		for (int r = 0; r < 6; r++) {
			if (map[c] != 0xff && map[c] & (1u << r)) {
				uint8_t el_idx = (r << 4) | c;
				uint8_t phys_idx = el_phys_map[el_idx];
				if (phys_idx != 0xff && n_keys < 128) {
#if MEGI_PROTO_BUG
					int key = keymap_base[phys_idx][0];
					if (key == KEY_LEFTCTRL || key == KEY_Z) // ignore these keys on my keyboard
						continue;
#endif

					keys[n_keys++] = phys_idx;
				}
			}
		}
	}

	// which pressed keys are no longer pressed?
	for (int j = 0; j < pressed_count; j++) {
		int key = pressed_keys[j];

		int idx = get_index(keys, n_keys, key);
		if (idx < 0) {
			pressed_keys[j] = 0;
			pressed_count = compact(pressed_keys, 128);
			on_release(key);
		}
	}

	// which new keys are pressed?
	for (int i = 0; i < n_keys; i++) {
		int key = keys[i];
		
		// if the key was not pressed, handle a new press event
		int pressed_idx = get_index(pressed_keys, pressed_count, key);
		if (pressed_idx < 0 && pressed_count < 128) {
			pressed_keys[pressed_count++] = key;
			on_press(key);
		}
	}
}

int main(int ac, char* av[])
{
	int fd, ret;

	fd = pogo_i2c_open();
	uinput_fd = open_uinput_dev();

	int lfd = gpio_setup_pl12(GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_BIAS_PULL_UP | /*GPIO_V2_LINE_FLAG_ACTIVE_HIGH |*/ GPIO_V2_LINE_FLAG_EDGE_FALLING);

	struct pollfd fds[2] = {
		{ .fd = lfd, .events = POLLIN, },
	};

	debug("\033[2J");

	uint8_t buf[4] = {1, 2, 3, 4};
	ret = write_kb(fd, buf);

	while (1) {
		ret = poll(fds, 1, 10000);
		syscall_error(ret < 0, "poll failed");

		if (fds[0].revents & POLLIN) {
			struct gpio_v2_line_event ev;
			ssize_t len = read(lfd, &ev, sizeof ev);
			syscall_error(len != sizeof ev, "Invalid event size");

			// read keyboard data
			uint8_t buf[16];
			ret = read_kb(fd, buf);
			if (ret)
				continue;

//			print_bitmap(buf + 4);
			update_keys(buf + 4);
		}
	}

	return 0;
}
