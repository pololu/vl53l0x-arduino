TARGET ?= NUCLEO_F411RE
TOOLCHAIN ?= GCC_ARM

.PHONY: all

all:
	mbed compile --library -m $(TARGET) -t $(TOOLCHAIN)

debug:
	mbed compile --library --options debug-info -m $(TARGET) -t $(TOOLCHAIN)

clean-build:
	mbed -c compile --library -m $(TARGET) -t $(TOOLCHAIN)
