/**
 * USB Programming tool for Pinephone keyboard
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

#include <linux/usbdevice_fs.h>

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

static int open_usb_dev(uint16_t vid, uint16_t pid)
{
	char path[256], buf[256];
	struct dirent *e;
	unsigned e_vid, e_pid, bus, dev;
	int fd = -1, ret;
	DIR* d;

        d = opendir("/sys/bus/usb/devices");
	syscall_error(d == NULL, "opendir(/sys/bus/usb/devices) failed");

	while (true) {
		errno = 0;
		e = readdir(d);
		syscall_error(e == NULL && errno, "readdir(/sys/bus/usb/devices) failed");
		if (!e)
			break;

                if (!strcmp(e->d_name, ".") || !strcmp(e->d_name, ".."))
			continue;

		snprintf(path, sizeof path,
			 "/sys/bus/usb/devices/%s/idVendor", e->d_name);
		if (!read_file(path, buf, sizeof buf))
			continue;

		ret = sscanf(buf, "%x", &e_vid);
		if (ret != 1)
			error("Failed to parse %s", path);

		snprintf(path, sizeof path,
			 "/sys/bus/usb/devices/%s/idProduct", e->d_name);
		if (!read_file(path, buf, sizeof buf))
			continue;

		ret = sscanf(buf, "%x", &e_pid);
		if (ret != 1)
			error("Failed to parse %s", path);

		if (e_vid == vid && e_pid == pid) {
			snprintf(path, sizeof path,
				 "/sys/bus/usb/devices/%s/busnum", e->d_name);
			if (!read_file(path, buf, sizeof buf))
				error("Failed to read %s", path);

			ret = sscanf(buf, "%u", &bus);
			if (ret != 1)
				error("Failed to parse %s", path);

			snprintf(path, sizeof path,
				 "/sys/bus/usb/devices/%s/devnum", e->d_name);
			if (!read_file(path, buf, sizeof buf))
				error("Failed to read %s", path);

			ret = sscanf(buf, "%u", &dev);
			if (ret != 1)
				error("Failed to parse %s", path);

			snprintf(path, sizeof path,
				 "/dev/bus/usb/%03u/%03u", bus, dev);

			debug("Found %04x:%04x at %s\n", e_vid, e_pid, path);

			fd = open(path, O_RDWR);
			syscall_error(fd < 0, "open(%s) failed", path);
			break;
		}
	}

	errno = ENOENT;
	closedir(d);
	return fd;
}

int bootloader_open(void)
{
	int ret, fd;
	bool had_switch = false;

	// first check if keyboard USB device is available, if it is
	// we need to first switch to bootloader mode

	fd = open_usb_dev(0x04f3, 0x1812);
	if (fd >= 0) {
		for (unsigned i = 0; i <= 1; i++) {
			struct usbdevfs_disconnect_claim dc = {
				.interface = i,
			};

			ret = ioctl(fd, USBDEVFS_DISCONNECT_CLAIM, &dc);
			syscall_error(ret < 0, "USBDEVFS_DISCONNECT_CLAIM failed");
		}

                /*
		 * Bootloader mode is enabled via HID SET_REPORT:SET_FEATURE
		 * control transfer on EP0.
		 */

		/* Enter IAP command payload */
		uint8_t buf[8] = {
			0xbc,
			0x01,
		};
		struct usbdevfs_ctrltransfer ctrl = {
			.bRequestType =
				  (0u << 7) // host->device
				| (1u << 5) // class command
				| (1u << 0), // to interface
			.bRequest = 0x09, // HID SET_REPORT class command
			.wValue = 0x03bc, // HID SET_FEATURE sub-command / 0xbc = enter IAP
			.wIndex = 0,
			.wLength = 8,
			.timeout = 300,
			.data = buf,
		};

		ret = ioctl(fd, USBDEVFS_CONTROL, &ctrl);
		if (ret == 0)
			error("Failed to switch keyboard to IAP programming mode");

		had_switch = true;
		close(fd);
	}

	// open the bootloader USB device (wait for it if we just switched to bootloader mode)

	for (int i = 0;; i++) {
		fd = open_usb_dev(0x04f3, 0x0905);
		if (fd >= 0)
			break;

		if (!had_switch)
			error("Bootloader USB device not found");

		if (i > 16)
			error("Bootloader USB device did not appear after switching keyboard to IAP mode");

		usleep(250000);
	}

	for (unsigned i = 0; i <= 3; i++) {
		struct usbdevfs_disconnect_claim dc = {
			.interface = i,
		};

		ret = ioctl(fd, USBDEVFS_DISCONNECT_CLAIM, &dc);
		syscall_error(ret < 0, "USBDEVFS_DISCONNECT_CLAIM failed");
	}

	return fd;
}

// Bootloader endpoints:
enum {
	EP_CMD = 0x01,
	EP_STATUS = 0x82,
	EP_DATAOUT = 0x03,
	EP_DATAIN = 0x84,
};

static int bootloader_fd = -1;

void bootloader_command(uint8_t req[8])
{
	int ret;
	struct usbdevfs_urb* reaped_urb;
	struct usbdevfs_urb urb = {
		.type = USBDEVFS_URB_TYPE_INTERRUPT,
		.endpoint = EP_CMD,
		.flags = USBDEVFS_URB_ZERO_PACKET,
		.buffer = req,
		.buffer_length = 8,
		.actual_length = 8,
	};

        ret = ioctl(bootloader_fd, USBDEVFS_SUBMITURB, &urb);
	syscall_error(ret < 0, "USBDEVFS_SUBMITURB failed");

	ret = ioctl(bootloader_fd, USBDEVFS_REAPURB, &reaped_urb);
	syscall_error(ret < 0, "USBDEVFS_REAPURB failed");

	debug("CMD:");
	for (int i = 0; i < urb.actual_length; i++)
		debug(" %02hhx", req[i]);
	debug("\n");
}

void bootloader_status(uint8_t res[4])
{
	int ret;
	struct usbdevfs_urb* reaped_urb;
	struct usbdevfs_urb urb = {
		.type = USBDEVFS_URB_TYPE_INTERRUPT,
		.endpoint = EP_STATUS,
		.flags = USBDEVFS_URB_SHORT_NOT_OK,
		.buffer = res,
		.buffer_length = 4,
		.actual_length = 0,
	};

        ret = ioctl(bootloader_fd, USBDEVFS_SUBMITURB, &urb);
	syscall_error(ret < 0, "USBDEVFS_SUBMITURB failed");

	ret = ioctl(bootloader_fd, USBDEVFS_REAPURB, &reaped_urb);
	syscall_error(ret < 0, "USBDEVFS_REAPURB failed");

	debug("RES:");
	for (int i = 0; i < urb.actual_length; i++)
		debug(" %02hhx", res[i]);
	debug("\n");
}

enum {
	STATUS_READY = 1,
	STATUS_BUSY = 2,
	STATUS_SUCCESS = 3,
	STATUS_FAIL = 4,
	STATUS_ERROR = 5,
};

void bootloader_standard_status_check(void)
{
	uint8_t res[4];
	const char* msg = "Unknown";

	bootloader_status(res);

	uint16_t status = res[0] << 8 | res[1];
	uint8_t err = res[2];

	switch (status) {
	case STATUS_BUSY:
		printf("Busy\n");
		break;

	case STATUS_FAIL:
		error("Fail status returned");

	case STATUS_ERROR:
		switch (err) {
			case 0x1: msg = "Command is unknown"; break;
			case 0x2: msg = "Command stage is error"; break;
			case 0x3: msg = "Data stage is error"; break;
			case 0x4: msg = "ROM address is error"; break;
			case 0x5: msg = "Authority Key is incorrect"; break;
			case 0x6: msg = "Write ROM is not finish"; break;
			case 0x7: msg = "Write Option is not finish"; break;
			case 0x8: msg = "Length is over"; break;
			case 0x9: msg = "Length is less"; break;
			case 0xa: msg = "CheckSum is incorrect"; break;
			case 0xb: msg = "Write Flash is abnormal"; break;
			case 0xc: msg = "It is over ROM area"; break;
			case 0xd: msg = "ROM page is error"; break;
			case 0xe: msg = "Flash Key is error"; break;
			case 0xf: msg = "Option ROM address range error"; break;
		}

		error("Error status returned: %s", msg);
	}
}

int bootloader_read_data(uint8_t res[64])
{
	int ret;
	struct usbdevfs_urb* reaped_urb;
	struct usbdevfs_urb urb = {
		.type = USBDEVFS_URB_TYPE_INTERRUPT,
		.endpoint = EP_DATAIN,
		.flags = USBDEVFS_URB_SHORT_NOT_OK,
		.buffer = res,
		.buffer_length = 64,
		.actual_length = 0,
//		.usercontext = (void*)(uintptr_t)0,
	};

        ret = ioctl(bootloader_fd, USBDEVFS_SUBMITURB, &urb);
	syscall_error(ret < 0, "USBDEVFS_SUBMITURB failed");

	ret = ioctl(bootloader_fd, USBDEVFS_REAPURB, &reaped_urb);
	syscall_error(ret < 0, "USBDEVFS_REAPURB failed");

	debug("DATA:");
	for (int i = 0; i < urb.actual_length; i++)
		debug(" %02hhx", res[i]);
	debug("\n");

	return urb.actual_length;
}

void bootloader_write_data(uint8_t res[64])
{
	int ret;
	struct usbdevfs_urb* reaped_urb;
	struct usbdevfs_urb urb = {
		.type = USBDEVFS_URB_TYPE_INTERRUPT,
		.endpoint = EP_DATAOUT,
		.flags = 0,
		.buffer = res,
		.buffer_length = 64,
		.actual_length = 64,
//		.usercontext = (void*)(uintptr_t)0,
	};

        ret = ioctl(bootloader_fd, USBDEVFS_SUBMITURB, &urb);
	syscall_error(ret < 0, "USBDEVFS_SUBMITURB failed");

	ret = ioctl(bootloader_fd, USBDEVFS_REAPURB, &reaped_urb);
	syscall_error(ret < 0, "USBDEVFS_REAPURB failed");

	debug("DATA:");
	for (int i = 0; i < urb.actual_length; i++)
		debug(" %02hhx", res[i]);
	debug("\n");
}

#define CMD_TAG				0xc1

#define CMD_GETVERSPEC			0x40
#define CMD_GETVERFW			0x41
#define CMD_GETSTATUS			0x42
#define CMD_EXITAUTHMODE		0x43
#define CMD_GETAUTHLOCK			0x44
#define CMD_SETAUTHLOCK			0x45
#define CMD_ABORT			0x46
#define CMD_GETCHECKSUM			0x47
#define CMD_ENTRYIAP			0x20
#define CMD_FINISHEDIAP			0x21
#define CMD_CANCELIAP			0x23
#define CMD_SOFTWARERESET		0x24
#define CMD_BOOTCONDITION		0x25

#define CMD_WRITEROM			0xa0
#define CMD_WRITEROMFINISH		0xa1
#define CMD_WRITEOPTION			0xa2
#define CMD_WRITEOPTIONFINISH		0xa3
#define CMD_WRITECHECKSUM		0xa4

/*
 * - called from WriteOptData in some cases (getVerSpec >= 0x170) or A chips
 * - we probably don't need this?
 */
#define CMD_WRITECUSTOMINFO		0xa5 // 3:addr_l=0 4:addr_h=1 5:len_l 6:len_h 7:0xa9 8 :0x7f (sec key)
#define CMD_WRITECUSTOMINFOFINISH	0xa6 // 3:csum

#define CMD_READROM			0xe0
#define CMD_READROMFINISH		0xe1
#define CMD_READOPTION			0xe2
#define CMD_READOPTIONFINISH		0xe3

#define CMD_READDATAREQUEST		0xe4 // not implemented

#define AUTH_KEY [6] = 0xa9, [7] = 0x7f

uint16_t cmd_get_ver_spec(void)
{
	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_GETVERSPEC, });
	uint8_t res[4];
	bootloader_status(res);
	return res[0] << 8 | res[1];
}

uint16_t cmd_get_ver_fw(void)
{
	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_GETVERFW, });
	uint8_t res[4];
	bootloader_status(res);
	return res[0] << 8 | res[1];
}

enum {
	DEV_STATUS_IDLE = 1,
	DEV_STATUS_IAP,
	DEV_STATUS_WR_ROM,
	DEV_STATUS_WR_OPT,
	DEV_STATUS_WR_CSUM,
	DEV_STATUS_RD_ROM,
	DEV_STATUS_RD_OPT,
};

uint8_t cmd_get_status(void)
{
	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_GETSTATUS, });
	uint8_t res[4];
	bootloader_status(res);
	return res[2];
}

void cmd_exit_auth_mode(void)
{
	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_EXITAUTHMODE, });
	bootloader_standard_status_check();
}

/* returns AUTHKEY */
uint8_t cmd_get_auth_lock(void)
{
	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_GETAUTHLOCK, });
	uint8_t res[4];
	bootloader_status(res);
	return res[0] ^ 0x24;
}

/* expects AUTHKEY */
void cmd_set_auth_lock(uint8_t key)
{
	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_SETAUTHLOCK, key ^ 0x58, });
	bootloader_standard_status_check();
}

void cmd_abort(void)
{
	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_ABORT, });
	bootloader_standard_status_check();
}

void cmd_unlock(void)
{
	cmd_set_auth_lock(cmd_get_auth_lock());
}

enum {
	CHECKSUM_TYPE_BOOT = 0,
	CHECKSUM_TYPE_MAIN = 1,
	CHECKSUM_TYPE_ALL = 2, // may not work
};

uint16_t cmd_get_checksum(uint8_t type)
{
	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_GETCHECKSUM, type, });
	uint8_t res[4];
	bootloader_status(res);
	return res[0] << 8 | res[1];
}

void cmd_entry_iap(void)
{
	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_ENTRYIAP, AUTH_KEY, });
	bootloader_standard_status_check();
}

void cmd_finished_iap(void)
{
	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_FINISHEDIAP, AUTH_KEY, });
	bootloader_standard_status_check();
}

void cmd_cancel_iap(void)
{
	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_CANCELIAP, });
	bootloader_standard_status_check();
}

void cmd_software_reset(void)
{
	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_SOFTWARERESET, });
	bootloader_standard_status_check();
}

enum {
	BOOT_COND1_P80_ENTRY = 1,
	BOOT_COND1_NO_APP_ENTRY = 2,
	BOOT_COND1_APP_JUMP_ENTRY = 4,
};

uint8_t cmd_boot_condition(void)
{
	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_BOOTCONDITION, });
	uint8_t res[4];
	bootloader_status(res);
	return res[2];
}

void cmd_read_option(uint8_t opts[128])
{
	uint16_t addr = 128;
	uint16_t len = 128;

	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_READOPTION,
					  addr & 0xff, addr >> 8,
					  len & 0xff, len >> 8, });
	bootloader_standard_status_check();

	bootloader_read_data(opts);
	bootloader_read_data(opts + 64);

	uint8_t csum = 0;
	for (int i = 0; i < len; i++)
		csum += opts[i];

	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_READOPTIONFINISH, csum, });
	bootloader_standard_status_check();
}

void cmd_read_rom(uint8_t* data, uint16_t addr, uint16_t len)
{
	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_READROM,
					  addr & 0xff, addr >> 8,
					  len & 0xff, len >> 8, });
	bootloader_standard_status_check();

	for (int i = 0; i < len / 64; i++)
		bootloader_read_data(data + 64 * i);

	uint8_t csum = 0;
	for (int i = 0; i < len; i++)
		csum += data[i];

	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_READROMFINISH, csum, });
	bootloader_standard_status_check();
}

void cmd_write_rom(uint8_t* data, uint16_t addr, uint16_t len)
{
	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_WRITEROM,
					  addr & 0xff, addr >> 8,
					  len & 0xff, len >> 8,
					  AUTH_KEY, });
	bootloader_standard_status_check();

	for (int i = 0; i < len / 64; i++)
		bootloader_write_data(data + 64 * i);

	uint8_t csum = 0;
	for (int i = 0; i < len; i++)
		csum += data[i];

	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_WRITEROMFINISH, csum, });
	bootloader_standard_status_check();
}

// risky function, cmd_write_option(opts, 0x80, 128)
void cmd_write_option(uint8_t* data, uint16_t addr, uint16_t len)
{
	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_WRITEOPTION,
					  addr & 0xff, addr >> 8,
					  len & 0xff, len >> 8,
					  AUTH_KEY, });
	bootloader_standard_status_check();

	for (int i = 0; i < len / 64; i++)
		bootloader_write_data(data + 64 * i);

	uint8_t csum = 0;
	for (int i = 0; i < len; i++)
		csum += data[i];

	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_WRITEOPTIONFINISH, csum, });
	bootloader_standard_status_check();
}

// writes checksum into the correct location in the option eeprom
void cmd_write_checksum(uint16_t csum)
{
	uint16_t addr = 0xfc;

	bootloader_command((uint8_t[8]) { CMD_TAG, CMD_WRITECHECKSUM,
					  addr & 0xff, addr >> 8,
					  csum & 0xff, csum >> 8,
					  AUTH_KEY, });
	bootloader_standard_status_check();
}

// }}}

const char* dev_status_text(uint8_t status)
{
	switch (status) {
	case DEV_STATUS_IDLE:    return "Idle";
	case DEV_STATUS_IAP:     return "IAP";
	case DEV_STATUS_WR_ROM:  return "Write ROM";
	case DEV_STATUS_WR_OPT:  return "Write Option";
	case DEV_STATUS_WR_CSUM: return "Write Checksum";
	case DEV_STATUS_RD_ROM:  return "Read ROM";
	case DEV_STATUS_RD_OPT:  return "Read Option";
	default:                 return "None";
	}
}

const char* boot_cond_text(uint8_t status)
{
	switch (status) {
	case BOOT_COND1_P80_ENTRY:       return "P80";
	case BOOT_COND1_NO_APP_ENTRY:    return "NO APP";
	case BOOT_COND1_APP_JUMP_ENTRY:  return "APP JUMP";
	default:                         return "Unknown";
	}
}

int main(int ac, char* av[])
{
	bootloader_fd = bootloader_open();

	if (ac == 2 && !strcmp(av[1], "reset")) {
		cmd_software_reset();
		return 0;
	}

	cmd_abort();

	printf("FW=0x%04hx BOOT=0x%04hx\n", cmd_get_ver_fw(), cmd_get_ver_spec());

	uint8_t opts[128];
	cmd_read_option(opts);

	uint16_t icid = (opts[124] | opts[125] << 8) ^ (opts[121] | opts[122] << 8);
	printf("ICID=%04hx\n", icid);

	cmd_unlock();

	uint8_t bootcond = cmd_boot_condition();
	printf("Booted via %s\n", boot_cond_text(bootcond));

	uint16_t csum_boot = cmd_get_checksum(CHECKSUM_TYPE_BOOT);
	uint16_t csum_app = cmd_get_checksum(CHECKSUM_TYPE_MAIN);
	printf("Checksums: boot=%04hx app=%04hx\n", csum_boot, csum_app);

	/*
	 * Checksums: boot=d355 app=449b
         *
	 * Option ROM from factory:
         *
	 *  CODE0 at 116: 0xff
	 *   - 24MHz intosc mode, WDT disabled, 256kHz low freq mode
	 *  CODE3 at 119: 0xff
	 *   - eeprom and hw reset disabled
	 *   - BITS(2..0) = reset button config
	 *   - BIT(3) = eeprom enable
	 *
	 * 0: fc 39 01 7f
	 * 4: fe ff ff ff
         *
         * ...: ff ff ff ff
         *
	 * 120: df     // read by bootloader & 0x6 = 0x6: 24MHz,
	 * 121: 0a 4f  // part of ICID (ICID is this value XORed with CSUM at 124, wtf?)
	 * 123: 20     // ???
	 * 124: 9b 44  // checksum (written by CMD_WRITECHECKSUM)
	 * 126: 49     // ???
	 * 127: aa     // app OK flag (0xaa = ok)
	 */

	int fd;
	uint8_t rom[0x8000];
	memset(rom, 0xff, sizeof rom);

	fd = open("fw.bin", O_RDONLY);
	syscall_error(fd < 0, "open(fw.bin) failed");
	ssize_t len = read(fd, rom, 0x8000);
	close(fd);
	if (len != 0x8000)
		error("Invalid rom size, must be 32768 bytes");

	cmd_entry_iap();
	cmd_write_rom(rom + 0x2000, 0x2000, 0x600);
	cmd_finished_iap();

        cmd_read_rom(rom, 0, 0x8000);
	fd = open("rom.bin", O_WRONLY | O_CREAT | O_TRUNC, 0666);
	if (fd >= 0) {
		write(fd, rom, 0x8000);
		close(fd);
	}

	cmd_software_reset();

	return 0;
}
