
NAME = cntrl
VERSION = 0.1
DISTNAME = $(NAME)-$(VERSION)

include ../build_rule.mk

CPPFLAGS = -O -Wextra -fPIC  -I../roomba -I../urg_library-1.1.8/include/cpp -I../urg_library-1.1.8/include/c

LIBROOMBA = ../roomba/libroomba.a
LIBURG = ../urg_library-1.1.8/src/liburg_cpp.a
BINARIES = cntrl


all: $(BINARIES) 

clean:
	rm -f *.o $(BINARIES)
	#rm -f $(DISTNAME).tar.gz

.cc.o:
	$(CXX) $(CPPFLAGS) -c $^

cntrl: cntrl.o Connection_information.o $(LIBROOMBA) $(LIBURG)
	$(CXX) $(CPPFLAGS) -lm -o $@ $^

$(LIBROOMBA) :
	cd $(@D)/ && $(MAKE) $(@F)

$(LIBURG) :
	cd $(@D)/ && $(MAKE) $(@F)


dist:
	echo "Building dist $(DISTNAME).tar.gz"
	tar -C .. --exclude=*.tar.gz --exclude=*.git --exclude=*~ \
        -cvzf $(DISTNAME).tar.gz $(NAME)
