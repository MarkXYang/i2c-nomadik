
CC = $(CROSS_COMPILE)gcc
CFLAGS	+= -Wall

RM := rm -f

.PHONY: all clean install uninstall

#
# Programs
#

evtest: evtest.o
	$(CC) $(LDFLAGS) -o $@ $^

#
# Objects
#

evtest.o: evtest.c
	$(CC) $(CFLAGS) -c $< -o $@


#
# Commands
#

all: evtest

clean:
	$(RM) evtest evtest.o

