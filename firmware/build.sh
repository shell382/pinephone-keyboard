#!/bin/bash

set -e

rm -rf build
mkdir -p build

hex2bin()
{
	local name="$1"
	
	makebin build/$name.ihx build/$name.bin
	dd if=bootloader.bin of=build/$name.bin conv=notrunc &>/dev/null
}

# build stock FW

cpp -P -nostdinc -I. -D__ASM_ONLY__ stock-ivt.asm build/stock-ivt.asm
sdas8051 -plosgff build/stock-ivt.rel build/stock-ivt.asm

echo Stock FW
sdcc \
	-mmcs51 --iram-size 256 --xram-size 2048 \
	--code-size 0x2000 --code-loc 0x2130 \
	-Wl-bIVECT=0x2000 \
	-I. \
	-DFW_REVISION_STR="\"$(git describe) $(git log -1 --format=%cd --date=iso)\"" \
	-DCONFIG_STOCK_FW=1 \
	-DCONFIG_I2C_A=0 \
	build/stock-ivt.rel main.c \
	-o build/fw-stock-proto-v3.ihx

hex2bin fw-stock-proto-v3

# build stock FW (hw mod)

cpp -P -nostdinc -I. -D__ASM_ONLY__ stock-ivt.asm build/stock-ivt.asm
sdas8051 -plosgff build/stock-ivt.rel build/stock-ivt.asm

echo "Stock FW Final"
sdcc \
	-mmcs51 --iram-size 256 --xram-size 2048 \
	--code-size 0x2000 --code-loc 0x2130 \
	-Wl-bIVECT=0x2000 \
	-I. \
	-DFW_REVISION_STR="\"$(git describe) $(git log -1 --format=%cd --date=iso)\"" \
	-DCONFIG_STOCK_FW=1 \
	-DCONFIG_I2C_A=1 \
	build/stock-ivt.rel main.c \
	-o build/fw-stock.ihx

hex2bin fw-stock

# build user FW

echo User FW
sdcc \
	-mmcs51 --iram-size 255 --xram-size 2048 \
	--code-size 0x4000 --code-loc 0x4000 \
	-I. \
	-DFW_REVISION_STR="\"$(git describe) $(git log -1 --format=%cd --date=iso)\"" \
	-DCONFIG_STOCK_FW=0 \
	-DCONFIG_USB_STACK=0 \
	-DCONFIG_DEBUG_LOG=1 \
	-DCONFIG_SELFTEST=0 \
	main.c \
	-o build/fw-user.ihx

hex2bin fw-user
