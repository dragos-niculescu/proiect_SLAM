
NAME = libroomba
VERSION = 0.1
DISTNAME = $(NAME)-$(VERSION)

include ../build_rule.mk

CPPFLAGS = -O -Wextra -fPIC  -I.
#C++ = g++

BINARIES = libroomba.a libroomba.so

all: $(BINARIES)

clean:
	rm -f *.o $(BINARIES)
	#rm -f $(DISTNAME).tar.gz

.cc.o:
	$(CXX) $(CPPFLAGS) -c $^


libroomba.a: roombalib.o 
	$(AR) rv $@ $^ 

libroomba.so: roombalib.o 
	$(CXX) $(CPPFLAGS) -lm -shared -o $@ $^

dist:
	echo "Building dist $(DISTNAME).tar.gz"
	tar -C .. --exclude=*.tar.gz --exclude=*.git --exclude=*~ \
        -cvzf $(DISTNAME).tar.gz $(NAME)
