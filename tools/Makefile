CC = $(CROSS_COMPILE)gcc

# build evtest executable when user executes "make"
#all: evtest
#evtest: evtest.o
#	$(CC) $(LDFLAGS) evtest.o -o evtest
#evtest.o:
#	$(CC) $(CFLAGS) -c evtest.c

# remove object files and executable when user executes "make clean"
#clean:
#	$(RM) *.o evtest


C = $(wildcard *.c)
X = $(C:.c=)

all: $(X)

clean:
	rm -f $(X) *~
