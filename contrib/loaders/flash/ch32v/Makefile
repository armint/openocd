# SPDX-License-Identifier: GPL-2.0-or-later

BIN2C = ../../../../src/helper/bin2char.sh

CROSS_COMPILE ?= riscv-none-elf-

CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump

AFLAGS = -march=rv32e -mabi=ilp32e -static -nostartfiles -nostdlib

all: ch32v.inc

.PHONY: clean

%.elf: %.S
	$(CC) $(AFLAGS) $< -o $@

%.lst: %.elf
	$(OBJDUMP) -S $< > $@

%.bin: %.elf
	$(OBJCOPY) -Obinary $< $@

%.inc: %.bin
	$(BIN2C) < $< > $@

clean:
	-rm -f *.elf *.lst *.bin *.inc
