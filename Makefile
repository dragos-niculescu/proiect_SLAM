all :
	cd roomba/ && $(MAKE)
	cd urg_library-1.1.8/ && $(MAKE)
	cd src/ && $(MAKE)

clean :
	cd roomba/ && $(MAKE) clean
	cd urg_library-1.1.8/ && $(MAKE) clean
	cd src/ && $(MAKE) clean

.PHONY : all clean
