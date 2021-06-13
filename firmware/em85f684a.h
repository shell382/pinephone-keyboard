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

#ifndef __EM85F684A_H__
#define __EM85F684A_H__

__sfr __at(0x87) PCON;                // Power Control
__sfr __at(0xc0) RSTSC;               // Reset Source
__sfr __at(0xbf) P0_PRST;             // Peripheral Reset
__sfr __at(0xee) P0_WDTKEY;           // WDT Key
__sfr __at(0xef) P0_WDTCR;            // WDT Control Register
__sfr __at(0x88) TCON;                // Timer Control
__sfr __at(0x89) TMOD;                // Timer Mode
__sfr __at(0x8a) TL0;                 // Timer 0 Low Byte
__sfr __at(0x8b) TL1;                 // Timer 1 Low Byte
__sfr __at(0x8c) TH0;                 // Timer 0 High Byte
__sfr __at(0x8d) TH1;                 // Timer 1 High Byte
__sfr __at(0x8e) CKCON0;              // Clock Control 0
__sfr __at(0x8f) CKCON1;              // Clock Control 1 (LOWCLK)
__sfr __at(0xa8) IE;                  // Interrupt Enable
__sfr __at(0xb8) IP;                  // Interrupt Priority
__sfr __at(0x81) SP;                  // Stack Pointer
__sfr __at(0x82) DPL;                 // Data Pointer Low
__sfr __at(0x83) DPH;                 // Data Pointer High
__sfr __at(0x84) DPL1;                // Data Pointer Low Byte 1
__sfr __at(0x85) DPH1;                // Data Pointer High Byte 1
__sfr __at(0x86) PAGESW;              // Page Switch
__sfr __at(0xd0) PSW0;                // Program Status Word 0
__sfr __at(0xd8) PSW1;                // Program Status Word 1
__sfr __at(0xe0) ACC;                 // Accumulator
__sfr __at(0xf0) B;                   // B Register
__sfr __at(0x98) SCON;                // Serial Control
__sfr __at(0x99) SBUF;                // Serial Data Buffer
__sfr __at(0x80) P5;                  // Port 5 Data Registers
__sfr __at(0x90) P6;                  // Port 6 Data Registers
__sfr __at(0xa0) P7;                  // Port 7 Data Registers
__sfr __at(0xb0) P8;                  // Port 8 Data Registers
__sfr __at(0xe8) P9;                  // Port 9 Data Registers
__sfr __at(0xf5) P0_PHCON1;           // Pull-High Control 1
__sfr __at(0xf7) P0_PHDSC1;           // Port High Drive/Sink Control 1
__sfr __at(0xf9) P0_P5M0;             // Port 5 Configuration Mode 0 Registers
__sfr __at(0xfa) P0_P6M0;             // Port 6 Configuration Mode 0 Registers
__sfr __at(0xfb) P0_P7M0;             // Port 7 Configuration Mode 0 Registers
__sfr __at(0xfc) P0_P8M0;             // Port 8 Configuration Mode 0 Registers
__sfr __at(0xfd) P0_PHCON0;           // Pull-High Control 0
__sfr __at(0xff) P0_PHDSC0;           // Port High Drive/Sink Control 0
__sfr __at(0xf9) P1_P9M0;             // Port 9 Configuration Mode 0 Registers
__sfr __at(0xfd) P1_PHCON2;           // Pull-High Control 2
__sfr __at(0xff) P1_PHDSC2;           // Port High Drive/Sink Control 2
__sfr __at(0xe9) P0_FLKEY;            // Flash Key
__sfr __at(0xea) P0_FLCR;             // Flash Control Register
__sfr __at(0xeb) P0_FPKEY;            // Enhanced Protection Key
__sfr __at(0xec) P0_EPPOINTL;         // Enhanced Protection Point Low Byte
__sfr __at(0xed) P0_EPPOINTH;         // Enhanced Protection Point High Byte
__sfr __at(0x91) P0_ADCS;             // ADCS(ADC Channel Select)
__sfr __at(0x92) P0_ADCR1;            // ADC Control Register 1
__sfr __at(0x93) P0_ADCR2;            // ADC Control Register 2
__sfr __at(0x94) P0_ADDH;             // ADDH (Converted value AD11~AD4 of ADC)
__sfr __at(0x95) P0_ADDL;             // ADDH (Converted value AD11~AD4 of ADC)
__sfr __at(0xa9) P0_TC3CR1;           // TC3CR1: Timer 3 Control Register 1
__sfr __at(0xaa) P0_TC3CR2;           // TC3CR1: Timer 3 Control Register 2
__sfr __at(0xab) P0_TC3DA;            // Timer 3 DATA Buffer A
__sfr __at(0xac) P0_TC3DB;            // Timer 3 DATA Buffer B
__sfr __at(0x9a) P0_SPIRX;            // SPI Receive Data Buffer Control Register
__sfr __at(0x9b) P0_SPIBUFPTR1;       // SPI Read FIFO counter(pointer) Register 1
__sfr __at(0x9c) P0_SPIBUFPTR2;       // SPI Read FIFO counter(pointer) Register 2
__sfr __at(0xa1) P0_SPICON1;          // SPICON1: SPI Control Register 1
__sfr __at(0xa2) P0_SPICON2;          // SPICON2: SPI Control Register 2
__sfr __at(0xa3) P0_SPITDBR;          // SPITDBR: SPI Transmit Data Buffer Register
__sfr __at(0xa4) P0_SPIRDBR;          // SPIRDBR: SPI Receive Data Buffer Register
__sfr __at(0xa5) P0_SPISR1;           // SPISR1: SPI Status Register 1
__sfr __at(0xa6) P0_SPISR2;           // SPISR1: SPI Status Register 2
__sfr __at(0xa7) P0_SPITX;            // SPI Transmit Data Buffer Control Register
__sfr __at(0xae) P0_ICEN;             // Input-Change Enable
__sfr __at(0xaf) P0_EEXSF;            // Extended External Interrupt Status Flag.
__sfr __at(0xb1) P0_EIE1;             // Extended Interrupt Enable 1
__sfr __at(0xb2) P0_EIE2;             // Extended Interrupt Enable 2
__sfr __at(0xb3) P0_EIE3;             // Extended Interrupt Enable 3
__sfr __at(0xb5) P0_EXEN;             // External Interrupt Pin Enable.
__sfr __at(0xb6) P0_EIESC1;           // External Interrupt Edge Select Control 1
__sfr __at(0xb7) P0_EIESC2;           // External Interrupt Edge Select Control 2
__sfr __at(0xb9) P0_EIP1;             // Extended Interrupt Priority 1
__sfr __at(0xba) P0_EIP2;             // Extended Interrupt Priority 2
__sfr __at(0xbb) P0_EIP3;             // Extended Interrupt Priority 3
__sfr __at(0xc1) P0_I2CACR1;          // I2CA Status and Control Register 1
__sfr __at(0xc2) P0_I2CACR2;          // I2CA Status and Control Register 2
__sfr __at(0xc3) P0_I2CASA;           // I2CA Slave Address Register
__sfr __at(0xc4) P0_I2CADB;           // I2CA Data Buffer Register
__sfr __at(0xc5) P0_I2CADAL;          // I2CA Device Address Register L
__sfr __at(0xc6) P0_I2CADAH;          // I2CA Device Address Register H
__sfr __at(0xc7) P0_I2CASF;           // I2CA status flag

__sfr __at(0xcd) P0_DEVPD1;           // Peripheral power down
__sfr __at(0xce) P0_DEVPD2;           // Peripheral power down
__sfr __at(0xcf) P0_DEVPD3;           // Peripheral power down

__sfr __at(0xd1) P0_SMBTO1;           // SMbus Time Out 1 Register
__sfr __at(0xd2) P0_SMBTR1;           // SMbus Timer reload 1 Register
__sfr __at(0xd3) P0_SMBTO2;           // SMbus Time Out 2 Register
__sfr __at(0xd4) P0_SMBTR2;           // SMbus Timer reload 2 Register
__sfr __at(0xd5) P0_SMBTO3;           // SMbus Time Out 3 Register
__sfr __at(0xd6) P0_SMBTR3;           // SMbus Timer reload 3 Register
__sfr __at(0xd7) P0_I2CBINT;          // I2C interrupt status and control register
__sfr __at(0xd9) P0_I2CBCR1;          // I2CB status and control register 1
__sfr __at(0xda) P0_I2CBCR2;          // I2CB status and control register 2
__sfr __at(0xdb) P0_I2CBSA;           // I2CB Slave Address Register
__sfr __at(0xdc) P0_I2CBDB;           // I2CB Data Buffer Register
__sfr __at(0xdd) P0_I2CBDAL;          // I2CB Device Address Register L
__sfr __at(0xde) P0_I2CBDAH;          // I2CB Device Address Register H
__sfr __at(0xdf) P0_I2CBCR3;          // I2CB status and control register 3
__sfr __at(0xe1) P0_I2CBCR4;          // I2CB status and control register 4
__sfr __at(0xe2) P0_I2CBAUTOCNT;      // I2C SCL auto-counter
__sfr __at(0xe3) P0_I2CBSCLFIR;       // I2CB SCL digital filter
__sfr __at(0xe4) P0_I2CBSDAFIR;       // I2CB SDA digital filter
__sfr __at(0xe5) P0_I2CBSTASU;        // I2C start/stop setup time timing register
__sfr __at(0xe6) P0_I2CBSTADH;        // I2C start/stop hold time timing register
__sfr __at(0x91) P1_IRCTEST;          // IRC Test Control Register
__sfr __at(0xa1) P1_STBCNT;           // IRC Stable Count Register
__sfr __at(0xa2) P1_IRCCTRL;          // IRC Control Register
__sfr __at(0xa3) P1_RETRIMTIME;       // IRC Re-Trim Period Register
__sfr __at(0xa4) P1_IRCFCVAL;         // IRC FC Value Register
__sfr __at(0xa5) P1_IRCCAVAL;         // IRC CA Value Register
__sfr __at(0xa6) P1_IRCFRVAL;         // IRC FR Value Register
__sfr __at(0xa7) P1_AUTOTRIMSTA;      // IRC Auto-Trim Status Register
__sfr __at(0xa9) P1_MTFCCTRL;         // IRC Manual-Trim FC Value Control Register
__sfr __at(0xaa) P1_MTCACTRL;         // IRC Manual-Trim CA Value Control Register
__sfr __at(0xab) P1_MTFRCTRL;         // IRC Manual-Trim FR Value Control Register
__sfr __at(0xac) P1_SOFCNTL;          // IRC SOF Count Low Byte Register
__sfr __at(0xad) P1_SOFCNTH;          // IRC SOF Count Low Byte Register
__sfr __at(0xae) P1_UDCACKCTRL;       // UDC ACK Interrupt Control Register
__sfr __at(0xaf) P1_EPASCTRL;         // Endpoint Access Control Register
__sfr __at(0xb1) P1_UDCCTRL;          // UDCCTRL: UDC Control Register
__sfr __at(0xb2) P1_UDCSTA;           // UDC Status Register
__sfr __at(0xb3) P1_UDCCFSTA;         // UDC Load Configuration Status Register
__sfr __at(0xb4) P1_UDCCFDATA;        // UDC Load Configuration Data Register
__sfr __at(0xb5) P1_UDCINT0EN;        // UDC Interrupt0 Enable Register
__sfr __at(0xb6) P1_UDCINT1EN;        // UDC Interrupt1 Enable Register
__sfr __at(0xb7) P1_UDCINT2EN;        // UDC Interrupt2 Enable Register
__sfr __at(0xb9) P1_UDCINT0STA;       // UDC Interrupt0 Status Register
__sfr __at(0xba) P1_UDCINT1STA;       // UDC Interrupt1 Status Register
__sfr __at(0xbb) P1_UDCINT2STA;       // UDC Interrupt2 Status Register
__sfr __at(0xbc) P1_UDCEPCTRL;        // Device End-point Control Register
__sfr __at(0xbd) P1_UDCEPBUF0CTRL;    // Device End-point Buffer0 Control Register
__sfr __at(0xbe) P1_UDCEPBUF1CTRL;    // Device End-point Buffer1 Control Register
__sfr __at(0xbf) P1_UDCEP0BUFDATA;    // UDC Endpoint0 Data Register
__sfr __at(0xc1) P1_UDCEP1BUFDATA;    // UDC Endpoint1 Data Register
__sfr __at(0xc2) P1_UDCEP2BUFDATA;    // UDC Endpoint2 Data Register
__sfr __at(0xc3) P1_UDCEP3BUFDATA;    // UDC Endpoint3 Data Register
__sfr __at(0xc4) P1_UDCEP4BUFDATA;    // UDC Endpoint4 Data Register
__sfr __at(0xc5) P1_UDCBUFSTA;        // UDC Buffer Status Register
__sfr __at(0xc9) P1_UDCEP1DATAINCNT;  // UDC Endpoint1 Data In Count Register
__sfr __at(0xca) P1_UDCEP1DATAOUTCNT; // UDC Endpoint1 Data Out Count Register
__sfr __at(0xcb) P1_UDCEP2DATAINCNT;  // UDC Endpoint2 Data In Count Register
__sfr __at(0xcc) P1_UDCEP2DATAOUTCNT; // UDC Endpoint2 Data Out Count Register
__sfr __at(0xcd) P1_UDCEP3DATAINCNT;  // UDC Endpoint3 Data In Count Register
__sfr __at(0xce) P1_UDCEP3DATAOUTCNT; // UDC Endpoint3 Data Out Count Register
__sfr __at(0xcf) P1_UDCEP4DATAINCNT;  // UDC Endpoint4 Data In Count Register
__sfr __at(0xd1) P1_UDCEP4DATAOUTCNT; // UDC Endpoint4 Data Out Count Register
__sfr __at(0xd2) P1_UDCEP1BUFDEPTH;   // UDC Endpoint1 Buffer Depth Register
__sfr __at(0xd3) P1_UDCEP2BUFDEPTH;   // UDC Endpoint2 Buffer Depth Register
__sfr __at(0xd4) P1_UDCEP3BUFDEPTH;   // UDC Endpoint3 Buffer Depth Register
__sfr __at(0xd5) P1_UDCEP4BUFDEPTH;   // UDC Endpoint4 Buffer Depth Register
__sfr __at(0xd6) P1_PHYTEST0;         // PHY Test0 Mode Register
__sfr __at(0xd7) P1_PHYTEST1;         // PHY Test1 Mode Register
__sfr __at(0xd9) P1_UDCRESCTRL;       // UDC Response Control Register
__sfr __at(0xda) P1_USBCTRL;          // UDC ACK Interrupt Control Register
__sfr __at(0xdb) P1_SE1CTL;           // SE1 Control register
__sfr __at(0xdc) P1_PWMDTR;           // PWMA Control Register
__sfr __at(0xdd) P1_PWMDTCR;          // Dead Time Control Register
__sfr __at(0xde) P1_TMREN;            // PWM Source Clock Control Register
__sfr __at(0xdf) P1_PWMSF;            // PWM Status Flag
__sfr __at(0xe1) P1_PWMACR;           // PWMA Control Register
__sfr __at(0xe2) P1_PRDAL;            // Low Byte of PWMA Period
__sfr __at(0xe4) P1_DTAL;             // Low Byte of PWMA Duty
__sfr __at(0xe6) P1_TMRAL;            // Low Byte of PWMA Timer
__sfr __at(0xe9) P1_PWMBCR;           // PWMB Control Register
__sfr __at(0xea) P1_PRDBL;            // Low Byte of PWMB Period
__sfr __at(0xec) P1_DTBL;             // Low Byte of PWMB Timer
__sfr __at(0xee) P1_TMRBL;            // Low Byte of PWMB Timer
__sfr __at(0xf1) P1_PWMCCR;           // PWMC Control Register
__sfr __at(0xf2) P1_PRDCL;            // Low Byte of PWMC Period
__sfr __at(0xf4) P1_DTCL;             // Low Byte of PWMB Timer
__sfr __at(0xf6) P1_TMRCL;            // Low Byte of PWMB Timer

__sbit __at(0xc7) SWRSF;  // Software Reset Force and Flag
__sbit __at(0xc2) WTSF;   // Watchdog Reset Flag.
__sbit __at(0xc0) PORSF;  // Power-On Flag
__sbit __at(0x8f) TF1;    // Timer 1 Overflow Flag.
__sbit __at(0x8e) TR1;    // Timer 1 Run Control.
__sbit __at(0x8d) TF0;    // Timer 0 Overflow Flag.
__sbit __at(0x8c) TR0;    // Timer 0 Run Control.
__sbit __at(0x8b) IE1;    // External Interrupt 1 Status Flag
__sbit __at(0x8a) IT1;    // External Interrupt 1 Type Select
__sbit __at(0x89) IE0;    // External Interrupt 0 Status Flag
__sbit __at(0x88) IT0;    // External Interrupt 0 Type Select
__sbit __at(0xaf) EA;     // Enable All Interrupt
__sbit __at(0xae) ICIE;   // Input-Change Interrupts
__sbit __at(0xac) ES0;    // Enable UART Interrupt
__sbit __at(0xab) ET1;    // Enable Timer 1 Interrupt
__sbit __at(0xaa) EX1;    // Enable External Interrupt 1.
__sbit __at(0xa9) ET0;    // Enable Timer 0 Interrupt
__sbit __at(0xa8) EX0;    // Enable External Interrupt 0
__sbit __at(0xbe) PICIE;  // Input-Change Interrupt Priority Control.
__sbit __at(0xbc) PS0;    // UART Interrupt Priority Control.
__sbit __at(0xbb) PT1;    // Timer 1 Interrupt Priority Control.
__sbit __at(0xba) PX1;    // External Interrupt 1 Priority Control.
__sbit __at(0xb9) PT0;    // Timer 0 Interrupt Priority Control.
__sbit __at(0xb8) PX0;    // External Interrupt 0 Priority Control
__sbit __at(0xd7) CY;     // Carry Flag.
__sbit __at(0xd6) AC;     // Auxiliary Carry Flag
__sbit __at(0xd5) F0;     // User Flag 0.
__sbit __at(0xd2) OV;     // Overflow Flag.
__sbit __at(0xd1) F1;     // User Flag 1
__sbit __at(0xd0) PARITY; // Parity Flag
__sbit __at(0xd8) SHSF;   // System-Hold Flag

// GPIO port data register bits

__sbit __at(0x87) P57;
__sbit __at(0x86) P56;
__sbit __at(0x85) P55;
__sbit __at(0x84) P54;
__sbit __at(0x83) P53;
__sbit __at(0x82) P52;
__sbit __at(0x81) P51;
__sbit __at(0x80) P50;
__sbit __at(0x97) P67;
__sbit __at(0x96) P66;
__sbit __at(0x95) P65;
__sbit __at(0x94) P64;
__sbit __at(0x93) P63;
__sbit __at(0x92) P62;
__sbit __at(0x91) P61;
__sbit __at(0x90) P60;
__sbit __at(0xa7) P77;
__sbit __at(0xa6) P76;
__sbit __at(0xa5) P75;
__sbit __at(0xa4) P74;
__sbit __at(0xa3) P73;
__sbit __at(0xa2) P72;
__sbit __at(0xa1) P71;
__sbit __at(0xa0) P70;
__sbit __at(0xb7) P87;
__sbit __at(0xb6) P86;
__sbit __at(0xb5) P85;
__sbit __at(0xb4) P84;
__sbit __at(0xb3) P83;
__sbit __at(0xb2) P82;
__sbit __at(0xb1) P81;
__sbit __at(0xb0) P80;
__sbit __at(0xef) P97;
__sbit __at(0xee) P96;
__sbit __at(0xed) P95;
__sbit __at(0xec) P94;
__sbit __at(0xeb) P93;
__sbit __at(0xea) P92;
__sbit __at(0xe9) P91;
__sbit __at(0xe8) P90;

#define IRQ_EINT0      0  // External Interrupt 0
#define IRQ_TIMER0     1  // Timer0 Overflow
#define IRQ_EINT1      2  // External Interrupt 1
#define IRQ_TIMER1     3  // Timer1 Overflow
#define IRQ_UART0      4  // Serial Port 0
#define IRQ_PINCHANGE  6  // PIN CHANGE Interrupt 0
#define IRQ_SYSTEMHOLD 8  // System Hold Interrupt
#define IRQ_INT2_3     10 // External Interrupt 2~3
#define IRQ_SPI	       11 // SPI Interrupt
#define IRQ_ADC	       13 // ADC Conversion Complete
#define IRQ_TIMER2     14 // Timer2 Overflow
#define IRQ_PWMA       15 // PWMA Interrupt
#define IRQ_USB	       17 // USB Interrupt
#define IRQ_I2CA       20 // I2CA Interrupt
#define IRQ_PWMB       23 // PWMB Interrupt
#define IRQ_PWMC       24 // PWMC Interrupt
#define IRQ_I2CB       28 // I2CB Interrupt

#endif
