/**
 * USB debuggign tool for Pinephone keyboard
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

#define DEBUG 1
#include "common.c"

int kb_open(void)
{
	int ret, fd;

	// first check if keyboard USB device is available, if it is
	// we need to first switch to bootloader mode

	fd = open_usb_dev(0x04f3, 0xb001);
	if (fd >= 0) {
		struct usbdevfs_disconnect_claim dc = {
			.interface = 0,
		};

		ret = ioctl(fd, USBDEVFS_DISCONNECT_CLAIM, &dc);
		syscall_error(ret < 0, "USBDEVFS_DISCONNECT_CLAIM failed");
	}

	return fd;
}

static int usb_fd = -1;

int command(uint8_t req[8])
{
	int ret;
	struct usbdevfs_urb urb = {
		.type = USBDEVFS_URB_TYPE_INTERRUPT,
		.endpoint = 0x01,
//		.flags = USBDEVFS_URB_ZERO_PACKET,
		.buffer = req,
		.buffer_length = 8,
		.actual_length = 8,
	};

	ret = handle_urb(usb_fd, &urb, 100);
	if (ret)
		return ret;

	debug("CMD:");
	for (int i = 0; i < urb.actual_length; i++)
		debug(" %02hhx", req[i]);
	debug("\n");
	
	return 0;
}

int response(uint8_t res[8])
{
	int ret;
	struct usbdevfs_urb urb = {
		.type = USBDEVFS_URB_TYPE_INTERRUPT,
		.endpoint = 0x82,
		.flags = USBDEVFS_URB_SHORT_NOT_OK,
		.buffer = res,
		.buffer_length = 8,
		.actual_length = 0,
	};

	ret = handle_urb(usb_fd, &urb, 100);
	if (ret)
		return ret;

	if (urb.actual_length != 8)
		error("Status response size invalid, must be 8 bytes, got %d", urb.actual_length);

	debug("RES:");
	for (int i = 0; i < urb.actual_length; i++)
		debug(" %02hhx", res[i]);
	debug("\n");
	
	return 0;
}

ssize_t xwrite(int fd, uint8_t* buf, size_t len)
{
	size_t off = 0;
	
	while (off < len) {
		ssize_t ret = write(fd, buf + off, len - off);
		if (ret < 0)
			return ret;
		
		off += ret;
	}
	
	return off;
}

int read_stdout(void)
{
	int ret;

	uint8_t buf[64];
	struct usbdevfs_urb urb = {
		.type = USBDEVFS_URB_TYPE_INTERRUPT,
		.endpoint = 0x83,
		.buffer = buf,
		.buffer_length = 64,
		.actual_length = 0,
	};

	ret = handle_urb(usb_fd, &urb, 10);
	if (ret)
		return ret;

	if (urb.actual_length > 0) {
		ssize_t rv = xwrite(1, buf, urb.actual_length);
		if (rv < 0)
			return -1;
	}

	return 0;
}

int read_keys(uint8_t buf[12])
{
	int ret;
	struct usbdevfs_urb urb = {
		.type = USBDEVFS_URB_TYPE_INTERRUPT,
		.endpoint = 0x84,
		.flags = USBDEVFS_URB_SHORT_NOT_OK,
		.buffer = buf,
		.buffer_length = 12,
		.actual_length = 0,
	};

	ret = handle_urb(usb_fd, &urb, 10);
	if (ret)
		return ret;

	return 0;
}

void print_bitmap(uint8_t* map)
{
//	printf("\033[H");
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

int main(int ac, char* av[])
{
	int ret;
	uint8_t keys[12];
	
	usb_fd = kb_open();
	if (usb_fd < 0)
		error("Failed to open the keyboard");
	
	int i = 0;
	while (1) {
		ret = read_stdout();

		ret = read_keys(keys);
		if (ret == 0)
			print_bitmap(keys);
		
		i++;
	}

	return 0;
}
