#!/bin/bash

set -e

rm -rf build
mkdir -p build

sdcc -mmcs51 --iram-size 256 --xram-size 2048 --code-size 0x6000 --code-loc 0x2000 --opt-code-size -I. main.c -o build/fw.ihx
makebin build/fw.ihx build/fw.bin
dd if=bootloader.bin of=build/fw.bin conv=notrunc &>/dev/null