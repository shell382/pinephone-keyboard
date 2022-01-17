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
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/types.h>
#include <dirent.h>
#include <getopt.h>

#include <linux/usbdevice_fs.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/gpio.h>

#define BIT(n) (1u << (n))

#define KB_ADDR 0x15
#define POWER_ADDR 0x75

static bool verbose;
#define debug(args...) { if (verbose) printf(args); }

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

__attribute__((noreturn))
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

static bool read_file(const char* path, char* buf, size_t size)
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
	char path[512], buf[256];
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

static int handle_urb(int usb_fd, struct usbdevfs_urb* urb, int timeout)
{
	int ret;
	struct usbdevfs_urb* reaped_urb;
	int retries = 0;

retry:
        ret = ioctl(usb_fd, USBDEVFS_SUBMITURB, urb);
        if (ret < 0)
        	return ret;

	struct pollfd fd = {
		.fd = usb_fd,
		.events = POLLOUT,
	};

	ret = poll(&fd, 1, timeout);
	if (ret <= 0) {
		if (ret == 0)
			errno = ETIMEDOUT;

		int save_errno = errno;

		// on timeout or other poll error, we need to discard and reap the submitted URB
		ret = ioctl(usb_fd, USBDEVFS_DISCARDURB, urb);

		// even if discard fails, URB may still be reapable, we need to try reaping anyway
		ret = ioctl(usb_fd, USBDEVFS_REAPURBNDELAY, &reaped_urb);

		// reap must immediately succeed, otherwise this is fatal
		syscall_error(ret < 0, "USBDEVFS_REAPURBNDELAY failed");

		errno = save_errno;
		return -1;
	}

	// hopefully POLLERR means we get some error immediately on reap

	ret = ioctl(usb_fd, USBDEVFS_REAPURB, &reaped_urb);
        if (ret < 0)
        	return ret;

	// EPROTO errors are recoverable
	if (urb->status == -71 && retries < 3) {
		retries++;
		goto retry;
	}

	if (urb->status != 0) {
		errno = -urb->status;
		return -1;
	}

	return 0;
}

static int pogo_i2c_open(void)
{
	int ret;
	char path[256], buf[1024];
	int fd = -1;

	for (int i = 0; i < 8; i++) {
		snprintf(path, sizeof path, "/sys/class/i2c-adapter/i2c-%d/uevent", i);
		if (!read_file(path, buf, sizeof buf))
			continue;
		
		if (!strstr(buf, "OF_FULLNAME=/soc/i2c@1c2b400") && !strstr(buf, "OF_FULLNAME=/i2c@ff140000"))
			continue;
		
		snprintf(path, sizeof path, "/dev/i2c-%d", i);
		
		int fd = open(path, O_RDWR);
		if (fd < 0 && errno == ENOENT)
			printf("WARNING: You may need to load the i2c-dev module (modprobe i2c-dev).\n");
		syscall_error(fd < 0, "open(%s) failed", path);

		return fd;		
	}
	
	error("Can't find POGO I2C adapter");
	return -1;
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

static const uint8_t crc8_0x7_table[] = {
	0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15,
	0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
	0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
	0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
	0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5,
	0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
	0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85,
	0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
	0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
	0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
	0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2,
	0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
	0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32,
	0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
	0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
	0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
	0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c,
	0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
	0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec,
	0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
	0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
	0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
	0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c,
	0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
	0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b,
	0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
	0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
	0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
	0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb,
	0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
	0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb,
	0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
};

static uint8_t crc8(const uint8_t *pdata, size_t nbytes)
{
	unsigned int idx;
	uint8_t crc = 0xff;

	while (nbytes--) {
		idx = crc ^ *pdata;
		crc = crc8_0x7_table[idx];
		pdata++;
	}

	return crc;
}

uint64_t time_abs(void)
{
	struct timespec tmp;
	int ret;

	ret = clock_gettime(CLOCK_MONOTONIC, &tmp);
	if (ret < 0)
		return 0;

	return tmp.tv_sec * 1000000000ull + tmp.tv_nsec;
}

static int gpiochip_open(const char* match)
{
	int ret;
	char path[256], buf[1024];
	int fd = -1;

	for (int i = 0; i < 8; i++) {
		snprintf(path, sizeof path, "/sys/bus/gpio/devices/gpiochip%d/uevent", i);
		if (!read_file(path, buf, sizeof buf))
			continue;

		if (!strstr(buf, match))
			continue;

		snprintf(path, sizeof path, "/dev/gpiochip%d", i);

		int fd = open(path, O_RDWR);
		syscall_error(fd < 0, "open(%s) failed", path);

		return fd;
	}

	return -1;
}

static int gpio_setup_pogo_int(unsigned flags)
{
	int ret;
	struct gpio_v2_line_request req = {
		.num_lines = 1,
		.offsets[0] = 12,
		.config.flags = flags,
		.consumer = "ppkbd",
	};

	int fd = gpiochip_open("OF_FULLNAME=/soc/pinctrl@1f02c00");
	if (fd < 0) {
		fd = gpiochip_open("OF_FULLNAME=/pinctrl/gpio@ff788000");
		if (fd < 0)
			error("Can't find gpiochip for POGO interrupt pin");
		
		// On Pinephone Pro, POGO-INT is GPIO3_A0
		req.offsets[0] = 0;
	}

	ret = ioctl(fd, GPIO_V2_GET_LINE_IOCTL, &req);
	if (ret < 0 && errno == EBUSY)
		printf("WARNING: The distribution you are using probably uses the kernel driver for pinephone keyboard, and is blocking access to the POGO pins from userspace. Or something else in your userspace already claimed the POGO interrupt pin.\n");
	syscall_error(ret < 0, "GPIO_V2_GET_LINE_IOCTL failed");

	close(fd);

	return req.fd;
}

static int gpio_get_value(int lfd)
{
	int ret;
	struct gpio_v2_line_values vals = {
		.mask = 1,
	};

	ret = ioctl(lfd, GPIO_V2_LINE_GET_VALUES_IOCTL, &vals);
	syscall_error(ret < 0, "GPIO_V2_GET_LINE_IOCTL failed");

	return vals.bits & 0x1;
}

static int gpio_set_value(int lfd, int val)
{
	int ret;
	struct gpio_v2_line_values vals = {
		.mask = 1,
	};

	ret = ioctl(lfd, GPIO_V2_LINE_GET_VALUES_IOCTL, &vals);
	syscall_error(ret < 0, "GPIO_V2_GET_LINE_IOCTL failed");

	return vals.bits & 0x1;
}
