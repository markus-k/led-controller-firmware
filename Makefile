#
# Copyright (C) 2018  Markus Kasten
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.
#

PROJECT_NAME := led-controller

########################################

UNAME := $(shell uname)
ifeq ($(UNAME),Linux)
	PREFIX ?= /usr/bin/arm-none-eabi
	LIBDIR ?= /usr/arm-none-eabi/lib
endif
ifeq ($(UNAME),Darwin)
	PREFIX ?= /usr/local/gcc-arm-none-eabi-6-2017-q2-update/bin/arm-none-eabi
	LIBDIR ?= /usr/local/gcc-arm-none-eabi-6-2017-q2-update/arm-none-eabi/lib
endif

OPENCM3_DIR ?= $(realpath libopencm3)

HOSTCC ?= gcc
OPENOCD ?= openocd

########################################

OBJS += src/main.o \
	src/board.o \
        src/gamma-lut.o \
        src/clock.o \
        src/eeprom.o \
        src/led.o \
        src/wifi.o \
        src/mqtt.o \
        src/mqtt_device.o \
        src/debug.o

WINC1500_OBJS := contrib/winc1500/common/source/nm_common.o \
                 contrib/winc1500/driver/source/m2m_ate_mode.o \
                 contrib/winc1500/driver/source/m2m_crypto.o \
                 contrib/winc1500/driver/source/m2m_hif.o \
                 contrib/winc1500/driver/source/m2m_ota.o \
                 contrib/winc1500/driver/source/m2m_periph.o \
                 contrib/winc1500/driver/source/m2m_ssl.o \
                 contrib/winc1500/driver/source/m2m_wifi.o \
                 contrib/winc1500/driver/source/nmasic.o \
                 contrib/winc1500/driver/source/nmbus.o \
                 contrib/winc1500/driver/source/nmdrv.o \
                 contrib/winc1500/driver/source/nmspi.o \
                 contrib/winc1500/socket/source/socket.o \
                 contrib/winc1500/spi_flash/source/spi_flash.o \
                 contrib/winc1500/bsp/source/nm_bsp_locm3.o \
                 contrib/winc1500/bus_wrapper/source/nm_bus_wrapper_locm3.o

MQTT_OBJS := contrib/wolfMQTT/src/mqtt_packet.o \
             contrib/wolfMQTT/src/mqtt_client.o \
             contrib/wolfMQTT/src/mqtt_socket.o

OBJS += $(WINC1500_OBJS)
OBJS += $(MQTT_OBJS)

########################################

DEVICE = stm32f101c8t6
CFLAGS          += -std=c99 -Icontrib -Icontrib/winc1500 -Icontrib/wolfMQTT/ -Isrc  -DWOLFMQTT_NONBLOCK=1 -DHAVE_CONFIG_H=1
CPPFLAGS	+= -MD
LDFLAGS         += -static -nostartfiles -L $(LIBDIR)/thumb/v7-m/ --specs=nano.specs
LDLIBS          += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

# remove unused functions
CFLAGS += -ffunction-sections -fdata-sections
LDFLAGS += -Wl,--gc-sections

DEBUG = 1

# -g3 also includes macros, which are helpful for debugging registers
DBGFLAGS := -g3 -O0 -DDEBUG=1

ifeq ($(DEBUG),1)
CFLAGS += $(DBGFLAGS)
endif

OPENOCD_FLAGS := -f interface/stlink-v2.cfg -f target/stm32f1x.cfg

OPENCM3_FLAGS := PREFIX="$(PREFIX)"

include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

########################################

.PHONY: clean
.SECONDARY: $(OBJS)

all: $(PROJECT_NAME).elf $(PROJECT_NAME).hex
	$(SIZE) $(PROJECT_NAME).elf

locm3:
	$(MAKE) $(OPENCM3_FLAGS) TARGETS="stm32/f1" -C $(OPENCM3_DIR)

src/gamma-lut.c: src/gamma-lut-gen
	./src/gamma-lut-gen 2.0 10 8 gamma_lut > $@

src/gamma-lut-gen: src/gamma-lut-gen.c
	$(HOSTCC) $< -lm -o $@

disasm: $(PROJECT_NAME).elf
	$(OBJDUMP) -S -d $^

debug: $(PROJECT_NAME).elf
	$(GDB) -ex "target remote localhost:3333" -ex "monitor reset halt" $(PROJECT_NAME).elf

openocd:
	$(OPENOCD) $(OPENOCD_FLAGS)

flash: $(PROJECT_NAME).elf
	$(OPENOCD) $(OPENOCD_FLAGS) -c "init" -c "targets" \
		-c "reset halt" -c "flash write_image erase $(PROJECT_NAME).elf" \
		-c "verify_image $(PROJECT_NAME).elf" -c "reset run" -c "shutdown"

clean:
	rm -f $(OBJS) $(OBJS:.o=.d)
	rm -f $(PROJECT_NAME).{elf,hex,bin,map}
	rm -f generated.*.ld
	rm -f src/gamma-lut-gen

mrproper: clean
	$(MAKE) -C $(OPENCM3_DIR) clean

########################################

include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk
