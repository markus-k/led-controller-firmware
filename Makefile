#
# Copyright (C) 2017  Markus Kasten
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

SOURCES := src/main.c \
           src/board.c \
           src/gamma-lut.c \
           src/clock.c \
           src/led.c \
           src/wifi.c \
           src/mqtt.c \
           src/mqtt_device.c \
           src/debug.c
#           src/fast_hsv2rgb_8bit.c

WINC1500_SOURCES := contrib/winc1500/common/source/nm_common.c \
                    contrib/winc1500/driver/source/m2m_ate_mode.c \
                    contrib/winc1500/driver/source/m2m_crypto.c \
                    contrib/winc1500/driver/source/m2m_hif.c \
                    contrib/winc1500/driver/source/m2m_ota.c \
                    contrib/winc1500/driver/source/m2m_periph.c \
                    contrib/winc1500/driver/source/m2m_ssl.c \
                    contrib/winc1500/driver/source/m2m_wifi.c \
                    contrib/winc1500/driver/source/nmasic.c \
                    contrib/winc1500/driver/source/nmbus.c \
                    contrib/winc1500/driver/source/nmdrv.c \
                    contrib/winc1500/driver/source/nmspi.c \
                    contrib/winc1500/socket/source/socket.c \
                    contrib/winc1500/spi_flash/source/spi_flash.c \
                    contrib/winc1500/bsp/source/nm_bsp_locm3.c \
                    contrib/winc1500/bus_wrapper/source/nm_bus_wrapper_locm3.c

#MQTT_SOURCES := contrib/paho-mqtt/MQTTClient-C/src/MQTTClient.c \
#                contrib/paho-mqtt/MQTTPacket/src/MQTTPacket.c
MQTT_SOURCES := contrib/wolfMQTT/src/mqtt_packet.c \
                contrib/wolfMQTT/src/mqtt_client.c \
                contrib/wolfMQTT/src/mqtt_socket.c

SOURCES += $(WINC1500_SOURCES)
SOURCES += $(MQTT_SOURCES)

HARDFP = 0

DEBUG = 1

############################################################

OPENCM3_DIR ?= $(realpath libopencm3)

UNAME := $(shell uname)

ifeq ($(UNAME),Linux)
	TOOLCHAIN_PREFIX ?= /usr/bin/arm-none-eabi
	TOOLCHAIN_LIBDIR ?= /usr/arm-none-eabi/lib
endif
ifeq ($(UNAME),Darwin)
	TOOLCHAIN_PREFIX ?= /usr/local/gcc-arm-none-eabi-6-2017-q2-update/bin/arm-none-eabi
	TOOLCHAIN_LIBDIR ?= /usr/local/gcc-arm-none-eabi-6-2017-q2-update/arm-none-eabi/lib
endif

CC      := $(TOOLCHAIN_PREFIX)-gcc
LD      := $(TOOLCHAIN_PREFIX)-gcc
OBJCOPY := $(TOOLCHAIN_PREFIX)-objcopy
OBJDUMP := $(TOOLCHAIN_PREFIX)-objdump
OBJSIZE := $(TOOLCHAIN_PREFIX)-size
GDB     := $(TOOLCHAIN_PREFIX)-gdb
MAKE    := make
OPENOCD := openocd

HOSTCC  := gcc

CPU      := cortex-m3
PLATFORM := stm32f1

#OPENCM3_TARGETS  := stm32/f3
#OPENCM3_LDSCRIPT := stm32/f3/stm32f303xc.ld
OPENCM3_TARGETS  := stm32/f1
OPENCM3_LDSCRIPT := stm32/f1/stm32f100x8.ld

CM3MAKEFLAGS := PREFIX="$(TOOLCHAIN_PREFIX)"

#OPENOCD_FLAGS := -f board/stm32f3discovery.cfg
OPENOCD_FLAGS := -f interface/stlink-v2.cfg -f target/stm32f1x.cfg

ifeq ($(HARDFP),1)
FP_FLAGS := -mfloat-abi=hard -mfpu=fpv4-sp-d16
else
FP_FLAGS := -mfloat-abi=soft
endif

WINC_CFLAGS := -Icontrib/winc1500
#MQTT_CFLAGS := -Icontrib/paho-mqtt/MQTTClient-C/src \
#               -Icontrib/paho-mqtt/MQTTPacket/src \
#               -DMQTTCLIENT_PLATFORM_HEADER=mqtt.h
MQTT_CFLAGS := -Icontrib/wolfMQTT/ -DWOLFMQTT_NONBLOCK=1 -DHAVE_CONFIG_H=1

CFLAGS := -I$(OPENCM3_DIR)/include \
          $(WINC_CFLAGS) \
          $(MQTT_CFLAGS) \
          -Icontrib \
          -Isrc \
          -std=c99 \
          -mcpu=$(CPU) \
          -mthumb \
          $(FP_FLAGS) \
          -mfix-cortex-m3-ldrd \
          -fno-builtin \
          -ffunction-sections \
          -fdata-sections \
          -fverbose-asm \
          -fno-common \
          -MD -D$(shell echo $(PLATFORM) | tr a-z A-Z)

LDFLAGS := --static \
           -nostartfiles \
           -L$(OPENCM3_DIR)/lib \
           -L$(dir $(OPENCM3_DIR)/lib/$(OPENCM3_LDSCRIPT)) \
           -T$(OPENCM3_DIR)/lib/$(OPENCM3_LDSCRIPT) \
           -lopencm3_$(PLATFORM) \
           -Wl,--gc-sections \
           -Wl,--Map=$(PROJECT_NAME).map \
           -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

# use include path for hardfp with hardfloat
ifeq ($(HARDFP),1)
LDFLAGS += -L $(TOOLCHAIN_LIBDIR)/gcc/arm-none-eabi/6.3.1/hard/ \
           -L $(TOOLCHAIN_LIBDIR)/hard/
endif

LDFLAGS += -L $(TOOLCHAIN_LIBDIR)/thumb/v7-m/ --specs=nano.specs

DBGFLAGS := -g -O0 -DDEBUG=1

ifeq ($(DEBUG),1)
CFLAGS += $(DBGFLAGS)
endif

OBJECTS := $(SOURCES:%.c=%.o)

all: $(BUILD_DIR) $(PROJECT_NAME).elf
	$(OBJSIZE) $(PROJECT_NAME).elf

# C files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Asm files
%.o: %.s
	$(CC) $(CFLAGS) -c $< -o $@

src/gamma-lut.c: src/gamma-lut-gen
	./src/gamma-lut-gen 2.0 10 8 gamma_lut > $@

src/gamma-lut-gen: src/gamma-lut-gen.c
	$(HOSTCC) $< -lm -o $@

sigmoid-lut.c: sigmoid-lut-gen
	./sigmoid-lut-gen 2.0 8 8 > sigmoid-lut.c

sigmoid-lut-gen: sigmoid-lut-gen.c
	$(HOSTCC) $< -lm -o sigmoid-lut-gen

# output files
$(PROJECT_NAME).elf \
$(PROJECT_NAME).hex \
$(PROJECT_NAME).bin: $(OPENCM3_DIR)/lib/libopencm3_$(PLATFORM).a $(OBJECTS)
	$(LD) $(OBJECTS) $(LDFLAGS) -o $(PROJECT_NAME).elf
	$(OBJCOPY) -O ihex $(PROJECT_NAME).elf $(PROJECT_NAME).hex
	$(OBJCOPY) -O binary $(PROJECT_NAME).elf $(PROJECT_NAME).bin

$(OPENCM3_DIR)/lib/libopencm3_$(PLATFORM).a:
	$(MAKE) $(CM3MAKEFLAGS) TARGETS="$(OPENCM3_TARGETS)" FP_FLAGS="$(FP_FLAGS)" -C $(OPENCM3_DIR)

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

.PHONY: clean

clean:
	rm -rf $(OBJECTS) $(OBJECTS:.o=.d)
	rm -f $(PROJECT_NAME).{elf,hex,bin}
	rm -f src/gamma-lut-gen

mrproper: clean
	$(MAKE) $(CM3MAKEFLAGS) TARGETS="$(OPENCM3_TARGETS)" -C $(OPENCM3_DIR) clean

-include $(OBJECTS:.o=.d)
