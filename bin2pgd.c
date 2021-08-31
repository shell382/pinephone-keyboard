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

int main(int ac, char* av[])
{
	int fd, ret;

	uint8_t rom[0x8100];
	memset(rom, 0xff, sizeof rom);

	fd = open(av[1], O_RDONLY);
	syscall_error(fd < 0, "open(%s) failed", av[1]);
	ssize_t len = read(fd, rom, 0x8000);
	syscall_error(len < 0, "read failed");
	close(fd);
	if (len != 0x8000)
		error("Invalid ROM file (%s) size (%d), must be 32768 bytes", av[1], (int)len);
	
	uint16_t csum = 0;
	for (int i = 0x0; i < 0x8000; i++)
		csum += rom[i];

	uint8_t* opts = rom + 0x8000;

	opts[0] = 0xfc;
	opts[1] = 0x39;
	opts[2] = 0x01;

	opts[120] = 0xdf;
	opts[123] = 0x20;
	opts[124] = csum;
	opts[125] = csum >> 8;
	opts[121] = 0x91 ^ opts[124];
	opts[122] = 0x0b ^ opts[125];
	opts[126] = 0x44; // IAP tool identifier (will be overwritten by bootloader to 0x49)
	opts[127] = 0xaa;

	fd = open(av[2], O_WRONLY | O_CREAT | O_TRUNC, 0666);
	if (fd >= 0) {
		ssize_t wr = write(fd, rom, 0x8100);
		syscall_error(wr < 0, "write failed");
		close(fd);
	}

	return 0;
}
