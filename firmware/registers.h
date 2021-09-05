/**
 * Pinephone Keyboard Firmware
 *
 * Copyright (C) 2021  Ondřej Jirman <megi@xff.cz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __PPKB_I2C_REGISTERS__
#define __PPKB_I2C_REGISTERS__

// defines register API in BCD format (currently 1.0)
// on incompatible change this may need to be changed
#define FW_REVISION		0x10

#define REG_DEVID_K		0x00
#define REG_DEVID_B		0x01
#define REG_FW_REVISION		0x02
#define REG_FW_FEATURES		0x03
#define REG_FW_FEATURES_USB_DEBUGGER	BIT(0)
#define REG_FW_FEATURES_FLASHING_MODE	BIT(1)
#define REG_FW_FEATURES_SELF_TEST	BIT(2)
#define REG_FW_FEATURES_STOCK_FW	BIT(3)
#define REG_FW_FEATURES_I2CA		BIT(4)

#define REG_KEYMATRIX_SIZE	0x06
#define REG_KEYMATRIX_STATE_CRC8 0x07
#define REG_KEYMATRIX_STATE	0x08
#define REG_KEYMATRIX_STATE_END	0x13

#define REG_SYS_CONFIG		0x20
#define REG_SYS_CONFIG_SCAN_BLOCK	BIT(0)

#define REG_SYS_CHG_ADDR	0x21
#define REG_SYS_CHG_DATA	0x22

#define REG_SYS_COMMAND		0x23
#define REG_SYS_COMMAND_MCU_RESET	'r'
#define REG_SYS_COMMAND_USB_IAP		'i'
#define REG_SYS_COMMAND_SELFTEST	't'
#define REG_SYS_COMMAND_CHG_READ	0x91
#define REG_SYS_COMMAND_CHG_WRITE	0xA1

#define REG_SYS_USER_APP_BLOCK	0x24
#define REG_SYS_USER_APP_BLOCK_MAGIC	0x53

#define REG_FLASH_DATA_START	0x70
#define REG_FLASH_DATA_END	0xef
#define REG_FLASH_ADDR_L	0xf0
#define REG_FLASH_ADDR_H	0xf1
#define REG_FLASH_CRC8		0xf2

#define REG_FLASH_UNLOCK	0xf3
#define REG_FLASH_UNLOCK_MAGIC	0x46

#define REG_FLASH_CMD		0xf4
#define REG_FLASH_CMD_READ_ROM	0x52
#define REG_FLASH_CMD_WRITE_ROM	0x57
#define REG_FLASH_CMD_ERASE_ROM	0x45
#define REG_FLASH_CMD_COMMIT	0x43

#define REG_DEBUG_LOG		0xff

#endif
