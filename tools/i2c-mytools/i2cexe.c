/* I2C extra tools
 *
 * i2cexe.c: user-space program to execute i2c commands
 *
 * 2012 - Designed by Fabrizio Ghiringhelli
 *        (inspired by the popular i2c-tools)
 *
 */

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <getopt.h>
#include <linux/i2c-dev.h>

#define ADAPTER_NR	0	/* Use I2C-0 */
#define	NBYTES_MAX	32

/* Command options */
static const char *short_options = "hr:w:o:n:";	/* short options */
static const struct option long_options[] = {
	{ "help",	0, NULL, 'h' },
	{ "read",	1, NULL, 'r' },
	{ "write",	1, NULL, 'w' },
	{ "offset",	1, NULL, 'o' },
	{ "nbytes",	1, NULL, 'n' },
	{ NULL,		0, NULL,  0  }
};

/* Help screen */
static void print_help(void)
{
	fprintf(stderr,
		"Usage: i2cexe -r SLAVE_ADDR [-o offset] [-n nbytes]\n"
		"       i2cexe -w SLAVE_ADDR [-o offset] VALUE\n"
		"  SLAVE_ADDR is a short integer that corresponds to the slave address (hex format)\n"
		"  'offset' is a short integer that correponds to the address to access (hex format)\n"
		"  'nbytes' is the number of bytes to read (up to %d, default 1)\n"
		"  VALUE is the value to write (hex format).\n", NBYTES_MAX);
}

/* Main */
int main(int argc, char *argv[])
{
	int file, option, i;
	char filename[20];
	__s32 res;
	unsigned short addr, offset, wr_value;
	unsigned short rd_nbytes = 1;
	unsigned short rd = 0;
	unsigned short wr = 0;
	unsigned short use_offset = 0;


	/* Parse command options */
	do {
		option = getopt_long(argc, argv, short_options, long_options, NULL);
		switch (option) {
		case 'h':	/* help */
			print_help();
			exit(0);

		case 'r':	/* read */
			if (sscanf(optarg, "%hx", &addr) != 1) {
				fprintf(stderr, "Error: Invalid format for 'slave address'\n");
				exit(1);
			}
			rd = 1;
			break;

		case 'w':	/* write */
			if (sscanf(optarg, "%hx", &addr) != 1) {
				fprintf(stderr, "Error: Invalid format for 'slave address'\n");
				exit(1);
			}
			wr = 1;
			break;

		case 'o':	/* offset */
			if (sscanf(optarg, "%hx", &offset) != 1) {
				fprintf(stderr, "Error: Invalid format for 'offset'\n");
				exit(1);
			}
			use_offset = 1;
			break;

		case 'n':	/* # of bytes to read */
			/* Parse number of bytes to read */
			if (sscanf(optarg, "%hu", &rd_nbytes) != 1) {
				fprintf(stderr, "Error: Invalid number of bytes to read\n");
				exit(1);
			}
			if (rd_nbytes > NBYTES_MAX)
				rd_nbytes = NBYTES_MAX;
			break;

		case -1:	/* done with options */
			break;

		default:	/* not a valid option */
			print_help();
			exit(1);
		}
	} while (option != -1);

	/* Check option consistency */
	if (wr && rd) {
		fprintf(stderr, "Error: Read-Write option conflict.\n");
		print_help();
		exit(1);
	}
	if (!wr && !rd) {
		fprintf(stderr, "Error: Read-Write option is mandatory.\n");
		print_help();
		exit(1);
	}

	if (wr) {
		if (optind >= argc) {
			fprintf(stderr, "Error: missing value to write\n");
			print_help();
			exit(1);
		}

		/* Parse value to write */
		if (sscanf(argv[optind], "%hx", &wr_value) != 1) {
			fprintf(stderr, "Error: Invalid value to write\n");
			exit(1);
		}
	}

	/* Open adapter device file */
	snprintf(filename, 19, "/dev/i2c%d", ADAPTER_NR);
	file = open(filename, O_RDWR);
	if (file < 0) {
		fprintf(stderr, "Error: Could not open adapter "
			"%s: %s\n", filename, strerror(errno));
		exit(1);
	}

	/* Set up the slave address */
	if (ioctl(file, I2C_SLAVE, addr) < 0) {
		fprintf(stderr, "Error: Could not communicate with slave "
			"address 0x%02X: %s\n", addr, strerror(errno));
		exit(1);
	}

	/* Handle read */
	if (rd) {
		if (use_offset) {
			/* Offset is given */
			fprintf(stdout, "Reading from slave address 0x%02X:\n"
					"[offset] : value\n"
					"----------------\n", addr);
			for (i = 0, res = 0; i < rd_nbytes && res >= 0; i++) {
				res = i2c_smbus_read_byte_data(file, (__u8) (offset+i));
				if (res < 0)
					fprintf(stderr, "[0x%02X] : failed\n", offset+i);
				else
					fprintf(stdout, "[0x%02X] : 0x%02X\n", offset+i, res);
			}
		} else {
			/* No offset */
			res = i2c_smbus_read_byte(file);
			if (res < 0)
				fprintf(stderr, "Error: read slave address 0x%02X failed\n", addr);
			else
				fprintf(stdout, "0x%02X\n", res);
		}
	}

	/* Handle write */
	if (wr) {
		if (use_offset) {
			/* Offset is given */
			res = i2c_smbus_write_byte_data(file, (__u8) offset, (__u8) wr_value);
			if (res < 0)
				fprintf(stderr, "Error: write at offset 0x%02X failed\n", offset);
		} else {
			/* No offset */
			res = i2c_smbus_write_byte(file, (__u8) wr_value);
			if (res < 0)
				fprintf(stderr, "Error: write slave address 0x%02X failed\n", addr);
		}
	}

	close(file);
	return res < 0 ? res : 0;
}

