
NAME = cntrl
VERSION = 0.0.1
DISTNAME = $(NAME)-$(VERSION)


CFLAGS = -O  -Wextra  -I.
CC = gcc
C++ = g++

BINARIES = cntrl 

all: $(BINARIES)

clean:
	rm -f *.o $(BINARIES)
	#rm -f $(DISTNAME).tar.gz

#.c.o:
#	$(CC) $(CFLAGS) -c $^ 

.cc.o:
	$(C++) $(CFLAGS) -c $^


cntrl: cntrl.o roombalib.o 
	$(C++) $(CFLAGS) -lm  $^ -o $@ 

dist:
	echo "Building dist $(DISTNAME).tar.gz"
	tar -C .. --exclude=*.tar.gz --exclude=*.svn --exclude=*~ \
        -cvzf $(DISTNAME).tar.gz $(NAME)
