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
#include <sys/poll.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/types.h>
#include <dirent.h>
#include <getopt.h>

#include <linux/usbdevice_fs.h>

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
