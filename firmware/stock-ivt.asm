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

#include "em85f684a.h"

	.area RSEG    (ABS,DATA)
	.org 0x0000
	ar0 = 0x00
	psw0 = 0xd0

.macro irq_user irq
	. = (ivct_start + irq * 8 + 3)
	ljmp (irq * 8 + 0x4003)
	.ds 5
.endm

.macro irq_ignore irq
	. = (ivct_start + irq * 8 + 3)
	reti
	.ds 7
.endm

.macro irq_stock irq, name
	. = (ivct_start + irq * 8 + 3)
	push    psw0
	push    ar0
	mov	r0, #_stock_flag
	ajmp    name
.endm

.macro irq_stock_fwd irq, name
name'_fwd:
	cjne    @r0, #1, 001$
	pop 	ar0
	pop 	psw0
	ljmp 	name
001$:
	pop 	ar0
	pop 	psw0
	ljmp (irq * 8 + 0x4003)
.endm

	.module ivect
	.area IVECT (REL)

	ivct_start = .

	ljmp	      __sdcc_gsinit_startup

	irq_ignore    5
	irq_ignore    9
	irq_ignore    13
	irq_ignore    19
	irq_ignore    21
	irq_ignore    22
	irq_ignore    25
	irq_ignore    26
	irq_ignore    27

        irq_user   IRQ_EINT0
        irq_user   IRQ_TIMER0
        irq_user   IRQ_EINT1
        irq_stock  IRQ_TIMER1, _timer1_interrupt_fwd
        irq_user   IRQ_UART0
        irq_stock  IRQ_PINCHANGE, _pinchange_interrupt_fwd
        irq_user   IRQ_LVD
        irq_user   IRQ_SYSTEMHOLD
        irq_user   IRQ_INT2_3
        irq_user   IRQ_SPI
        irq_user   IRQ_PWMD
        irq_user   IRQ_PWME
        irq_user   IRQ_PWMF
        irq_user   IRQ_TIMER3
        irq_user   IRQ_PWMA
        irq_user   IRQ_USB
        irq_stock  IRQ_USB _usb_interrupt_fwd
        irq_user   IRQ_I2CA
        irq_user   IRQ_PWMB
        irq_user   IRQ_PWMC
        irq_stock  IRQ_I2CB _i2c_b_interrupt_fwd

	irq_stock_fwd IRQ_TIMER1, _timer1_interrupt
	irq_stock_fwd IRQ_PINCHANGE, _pinchange_interrupt
	irq_stock_fwd IRQ_I2CB, _i2c_b_interrupt
	irq_stock_fwd IRQ_USB, _usb_interrupt
