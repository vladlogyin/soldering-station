# Figure out what OS we're running on
ifeq ($(OS),Windows_NT)
  HOST= Windows
else
ifeq ($(shell which cmd.exe || echo "notwsl"),notwsl)
    HOST=$(shell uname -s)
else
    HOST=WSL
endif
endif

# Change this to a source file containing main()
PROJECT_SRC:= main.cpp
PROJECT:= $(basename $(PROJECT_SRC))
PROJECT_OBJ:= $(PROJECT).o
PROJECT_ELF:= $(PROJECT).elf
PROJECT_BIN:= $(PROJECT).bin

SOURCES_CPP= drivers/delay.cpp drivers/ui/st7739/st7739.cpp drivers/ui/st7739/font.cpp drivers/input/encoder/encoder.cpp
SOURCES_C=

OBJECTS:= $(patsubst %.cpp, %.o, $(SOURCES_CPP)) $(patsubst %.c, %.o, $(SOURCES_C))
DEPENDS:= $(patsubst %.cpp,%.d,$(SOURCES_CPP)) $(patsubst %.c, %.d, $(SOURCES_C)) $(PROJECT).d
# setup
INCLUDE_DIRS= -I. -I./drivers -I./libopencm3/include
LIBRARY_DIRS= -L./libopencm3/lib
LIBRARIES= -lgcc -lm -lopencm3_gd32e23 -lstdc++

# Select c++17 and turn optimizations on
COMPILE_OPTIONS= -fno-exceptions -fno-non-call-exceptions -fno-common -ffunction-sections -fdata-sections -flto -std=c++17 -O2 -fno-rtti -finline-small-functions -findirect-inlining
# Include debugging symbols and and turn off optimizations
#COMPILE_OPTIONS+= -g3 -O0

CROSS= arm-none-eabi-

CC= $(CROSS)gcc
CXX= $(CROSS)g++
AS= $(CROSS)as
LD= $(CROSS)g++
OBJCP= $(CROSS)objcopy
AR= $(CROSS)ar
SIZE= $(CROSS)size

ifeq ($(shell which $(CROSS)gdb || echo "multiarch"),multiarch)
# If a cross gdb doesn't exist, try the normal gdb in the hopes that it was built with multitarget enabled
ifeq ($(shell which gdb-multiarch || echo "normalgdb"),normalgdb)
GDB= gdb
else
GDB= gdb-multiarch
endif
else
GDB= $(CROSS)gdb
endif

# Use the native windows exe if running in WSL
ifeq ($(HOST) , WSL)
OPENOCD=./toolchain/openocd-win/bin/openocd.exe
OPENOCD_IP=$(shell cat /etc/resolv.conf | grep nameserver | cut -d ' ' -f 2)
else
OPENOCD=$(shell which openocd)
OPENOCD_IP=localhost
endif

# Define MCU specific flags here
LIBOPENCM3=libopencm3/lib/libopencm3_gd32e23.a
ARCHITECTURE_FLAGS= -mcpu=cortex-m23 -mthumb -msoft-float -DGD32E23

CXXFLAGS= $(COMPILE_OPTIONS) $(ARCHITECTURE_FLAGS) $(INCLUDE_DIRS)
CFLAGS= $(COMPILE_OPTIONS) $(ARCHITECTURE_FLAGS) $(INCLUDE_DIRS)
ASFLAGS= $(COMPILE_OPTIONS) -c

LDFLAGS= -mthumb -nostartfiles --static --specs=nosys.specs --specs=nano.specs
# Select linker script
LDFLAGS+= -T toolchain/ldscripts/gd32e230f6p6.ld
# Add linker optimizations
LDFLAGS+= $(CXXFLAGS)
# Add libraries
LDFLAGS+= $(LIBRARY_DIRS) $(LIBRARIES)
OBJCPFLAGS= -O binary
ARFLAGS= cr



# all

all: build

install: flash
# Include header dependencies
-include $(DEPENDS)

drivers/ui/st7739/font.h drivers/ui/st7739/font.cpp: font-generation

font-generation: toolchain/font-generator/*
	$(MAKE) -C toolchain/font-generator -q font-deps || OUTPUT=$(shell pwd)/drivers/ui/st7739/ $(MAKE) -C toolchain/font-generator font


# Recompile objects when Makefile changes
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@
%.o: %.c
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@
$(PROJECT_OBJ) $(OBJECTS): Makefile $(LIBOPENCM3)

$(PROJECT_ELF): $(PROJECT_OBJ) $(OBJECTS) $(LIBOPENCM3)
	$(LD) $(LDFLAGS) $^ -o $@
	$(SIZE) $@

$(PROJECT_BIN): $(PROJECT_ELF)
	$(OBJCP) $(OBJCPFLAGS) $< $@


# flash
build: $(PROJECT_BIN)




flash: build $(OPENOCD)
	-pkill openocd -9
	-rm target.bin
	ln -s $(PROJECT_BIN) target.bin
	$(OPENOCD) -f toolchain/openocd/gd32e23_flash.cfg
debug: flash
	-pkill openocd -9
ifeq ($(HOST),WSL)
#	netsh.exe interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=3333 connectaddress=localhost connectport=3333
endif
	$(OPENOCD) -f toolchain/openocd/gd32e23_debug.cfg &
	$(GDB) -iex "target extended-remote $(OPENOCD_IP):3333" $(PROJECT_ELF)
# libopencm3

libopencm3/Makefile:
	@echo "Initializing libopencm3 submodule"
	git submodule update --init --remote
newlib/Makefile:
	@echo "Initializing newlib submodule"
	git submodule update --init --remote

libopencm3/lib/libopencm3_%.a: libopencm3/Makefile toolchain/.checktoolchain
	$(MAKE) -C libopencm3

clean:
	-rm $(PROJECT_ELF) $(PROJECT_BIN) $(PROJECT_OBJ) *.bin *.elf *.elf.map *.o $(OBJECTS) $(DEPENDS)

cleanall: clean
	$(MAKE) -C libopencm3 clean
	-rm toolchain/.checktoolchain

checktoolchain: toolchain/.checktoolchain

# This rule is here so that checktoolchain.sh isn't executed every time a build is started
toolchain/.checktoolchain:
	@bash toolchain/checktoolchain.sh $(HOST)
	echo $(HOST) > toolchain/.checktoolchain

$(OPENOCD): checktoolchain
	@echo "Checking for openocd"

.PHONY: clean install flash debug
