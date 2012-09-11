#
# I2C MyTools
#

CC = $(CROSS_COMPILE)gcc
CFLAGS	+= -Wall

MYI2C_INCLUDE_DIR	:= include
MYI2C_CFLAGS	:= -Wstrict-prototypes -Wshadow -Wpointer-arith -Wcast-qual \
		   -Wcast-align -Wwrite-strings -Wnested-externs -Winline \
		   -W -Wundef -Wmissing-prototypes -I$(MYI2C_INCLUDE_DIR)
RM := rm -f


.PHONY: all clean install uninstall

#
# Programs
#

i2cexe: i2cexe.o
	$(CC) $(LDFLAGS) -o $@ $^

#
# Objects
#

i2cexe.o: i2cexe.c $(MYI2C_INCLUDE_DIR)/linux/i2c-dev.h
	$(CC) $(CFLAGS) $(MYI2C_CFLAGS) -c $< -o $@


#
# Commands
#

all: $(MYI2C_TARGETS)

clean:
	$(RM) i2cexe i2cexe.o

