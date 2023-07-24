#!/bin/bash

make NAME=ekausb
avr-objcopy -O ihex -R .signature -R .fuse -R .eeprom ekausb.elf ekausb.hex
dfu-programmer atmega32u2 erase
dfu-programmer atmega32u2 flash ekausb.hex
dfu-programmer atmega32u2 reset
