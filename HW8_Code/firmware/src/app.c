/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include <math.h>       // I like to do math
#include <stdio.h>
//#include "ILI9163C.h"
//#include "I2C2_Commands.h"

// code for ILI9163C on the PIC32
// adapted from https://github.com/sumotoy/TFT_ILI9163C/blob/master/TFT_ILI9163C.cpp

#ifndef ILI9163C_H__
#define ILI9163C_H__

// lookup table for all of the ascii characters
static const char ASCII[96][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00} // 20  (space)
    ,
    {0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
    ,
    {0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
    ,
    {0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
    ,
    {0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
    ,
    {0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
    ,
    {0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
    ,
    {0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
    ,
    {0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
    ,
    {0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
    ,
    {0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
    ,
    {0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
    ,
    {0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
    ,
    {0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
    ,
    {0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
    ,
    {0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
    ,
    {0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
    ,
    {0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
    ,
    {0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
    ,
    {0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
    ,
    {0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
    ,
    {0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
    ,
    {0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
    ,
    {0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
    ,
    {0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
    ,
    {0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
    ,
    {0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
    ,
    {0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
    ,
    {0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
    ,
    {0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
    ,
    {0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
    ,
    {0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
    ,
    {0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
    ,
    {0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
    ,
    {0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
    ,
    {0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
    ,
    {0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
    ,
    {0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
    ,
    {0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
    ,
    {0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
    ,
    {0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
    ,
    {0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
    ,
    {0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
    ,
    {0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
    ,
    {0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
    ,
    {0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
    ,
    {0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
    ,
    {0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
    ,
    {0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
    ,
    {0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
    ,
    {0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
    ,
    {0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
    ,
    {0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
    ,
    {0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
    ,
    {0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
    ,
    {0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
    ,
    {0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
    ,
    {0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
    ,
    {0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
    ,
    {0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
    ,
    {0x02, 0x04, 0x08, 0x10, 0x20} // 5c ¥
    ,
    {0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
    ,
    {0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
    ,
    {0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
    ,
    {0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
    ,
    {0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
    ,
    {0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
    ,
    {0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
    ,
    {0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
    ,
    {0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
    ,
    {0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
    ,
    {0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
    ,
    {0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
    ,
    {0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
    ,
    {0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j
    ,
    {0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
    ,
    {0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
    ,
    {0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
    ,
    {0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
    ,
    {0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
    ,
    {0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
    ,
    {0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
    ,
    {0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
    ,
    {0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
    ,
    {0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
    ,
    {0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
    ,
    {0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
    ,
    {0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
    ,
    {0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
    ,
    {0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
    ,
    {0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
    ,
    {0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
    ,
    {0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
    ,
    {0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
    ,
    {0x10, 0x08, 0x08, 0x10, 0x08} // 7e ?
    ,
    {0x00, 0x06, 0x09, 0x09, 0x06} // 7f ?
}; // end char ASCII[96][5]

// ILI9163C registers
#define CMD_NOP     	0x00//Non operation
#define CMD_SWRESET 	0x01//Soft Reset
#define CMD_SLPIN   	0x10//Sleep ON
#define CMD_SLPOUT  	0x11//Sleep OFF
#define CMD_PTLON   	0x12//Partial Mode ON
#define CMD_NORML   	0x13//Normal Display ON
#define CMD_DINVOF  	0x20//Display Inversion OFF
#define CMD_DINVON   	0x21//Display Inversion ON
#define CMD_GAMMASET 	0x26//Gamma Set (0x01[1],0x02[2],0x04[3],0x08[4])
#define CMD_DISPOFF 	0x28//Display OFF
#define CMD_DISPON  	0x29//Display ON
#define CMD_IDLEON  	0x39//Idle Mode ON
#define CMD_IDLEOF  	0x38//Idle Mode OFF
#define CMD_CLMADRS   	0x2A//Column Address Set
#define CMD_PGEADRS   	0x2B//Page Address Set

#define CMD_RAMWR   	0x2C//Memory Write
#define CMD_RAMRD   	0x2E//Memory Read
#define CMD_CLRSPACE   	0x2D//Color Space : 4K/65K/262K
#define CMD_PARTAREA	0x30//Partial Area
#define CMD_VSCLLDEF	0x33//Vertical Scroll Definition
#define CMD_TEFXLON		0x35//Tearing Effect Line ON
#define CMD_TEFXLOF		0x34//Tearing Effect Line OFF
#define CMD_MADCTL  	0x36//Memory Access Control
#define CMD_VSSTADRS	0x37//Vertical Scrolling Start address
#define CMD_PIXFMT  	0x3A//Interface Pixel Format
#define CMD_FRMCTR1 	0xB1//Frame Rate Control (In normal mode/Full colors)
#define CMD_FRMCTR2 	0xB2//Frame Rate Control(In Idle mode/8-colors)
#define CMD_FRMCTR3 	0xB3//Frame Rate Control(In Partial mode/full colors)
#define CMD_DINVCTR		0xB4//Display Inversion Control
#define CMD_RGBBLK		0xB5//RGB Interface Blanking Porch setting
#define CMD_DFUNCTR 	0xB6//Display Fuction set 5
#define CMD_SDRVDIR 	0xB7//Source Driver Direction Control
#define CMD_GDRVDIR 	0xB8//Gate Driver Direction Control

#define CMD_PWCTR1  	0xC0//Power_Control1
#define CMD_PWCTR2  	0xC1//Power_Control2
#define CMD_PWCTR3  	0xC2//Power_Control3
#define CMD_PWCTR4  	0xC3//Power_Control4
#define CMD_PWCTR5  	0xC4//Power_Control5
#define CMD_VCOMCTR1  	0xC5//VCOM_Control 1
#define CMD_VCOMCTR2  	0xC6//VCOM_Control 2
#define CMD_VCOMOFFS  	0xC7//VCOM Offset Control
#define CMD_PGAMMAC		0xE0//Positive Gamma Correction Setting
#define CMD_NGAMMAC		0xE1//Negative Gamma Correction Setting
#define CMD_GAMRSEL		0xF2//GAM_R_SEL

#define _GRAMWIDTH 130//128
#define _GRAMHEIGH 130//128 //160
#define _GRAMSIZE  _GRAMWIDTH * _GRAMHEIGH

#define	BLACK     0x0000
#define WHITE     0xFFFF
#define	BLUE      0x001F
#define	RED       0xF800
#define	GREEN     0x07E0
#define CYAN      0x07FF
#define MAGENTA   0xF81F
#define YELLOW    0xFFE0

static unsigned char pGammaSet[15] = {0x36, 0x29, 0x12, 0x22, 0x1C, 0x15, 0x42, 0xB7, 0x2F, 0x13, 0x12, 0x0A, 0x11, 0x0B, 0x06};
static unsigned char nGammaSet[15] = {0x09, 0x16, 0x2D, 0x0D, 0x13, 0x15, 0x40, 0x48, 0x53, 0x0C, 0x1D, 0x25, 0x2E, 0x34, 0x39};

unsigned char spi_io(unsigned char); // send and rx a byte over spi
void LCD_command(unsigned char); // send a command to the LCD
void LCD_data(unsigned char); // send data to the LCD
void LCD_data16(unsigned short); // send 16 bit data to the LCD
void LCD_init(void); // send the initializations to the LCD
void LCD_drawPixel(unsigned short, unsigned short, unsigned short); // set the x,y pixel to a color
void LCD_setAddr(unsigned short, unsigned short, unsigned short, unsigned short); // set the memory address you are writing to
void LCD_clearScreen(unsigned short); // set the color of every pixel

#endif

// I2C Master utilities, 100 kHz, using polling rather than interrupts
// The functions must be callled in the correct order as per the I2C protocol
// Change I2C2 to the I2C channel you are using
// I2C pins need pull-up resistors, 2k-10k

void i2c_master_setup(void) {

    I2C2BRG = 233; //(1.00/(2*(float)Fsck)-(float)PGD)*(float)Pblck-2;            // I2CBRG = [1/(2*Fsck) - PGD]*Pblck - 2 
    // look up PGD for your PIC32
    I2C2CONbits.ON = 1; // turn on the I2C2 module
}

// Start a transmission on the I2C bus

void i2c_master_start(void) {
    I2C2CONbits.SEN = 1; // send the start bit
    while (I2C2CONbits.SEN) {
        ;
    } // wait for the start bit to be sent
}

void i2c_master_restart(void) {
    I2C2CONbits.RSEN = 1; // send a restart 
    while (I2C2CONbits.RSEN) {
        ;
    } // wait for the restart to clear
}

void i2c_master_send(unsigned char byte) { // send a byte to slave
    I2C2TRN = byte; // if an address, bit 0 = 0 for write, 1 for read
    while (I2C2STATbits.TRSTAT) {
        ;
    } // wait for the transmission to finish
    if (I2C2STATbits.ACKSTAT) { // if this is high, slave has not acknowledged
        ; // ("I2C2 Master: failed to receive ACK\r\n");
    }
}

unsigned char i2c_master_recv(void) { // receive a byte from the slave
    I2C2CONbits.RCEN = 1; // start receiving data
    while (!I2C2STATbits.RBF) {
        ;
    } // wait to receive the data
    return I2C2RCV; // read and return the data
}

void i2c_master_ack(int val) { // sends ACK = 0 (slave should send another byte)
    // or NACK = 1 (no more bytes requested from slave)
    I2C2CONbits.ACKDT = val; // store ACK/NACK in ACKDT
    I2C2CONbits.ACKEN = 1; // send ACKDT
    while (I2C2CONbits.ACKEN) {
        ;
    } // wait for ACK/NACK to be sent
}

void i2c_master_stop(void) { // send a STOP:
    I2C2CONbits.PEN = 1; // comm is complete and master relinquishes bus
    while (I2C2CONbits.PEN) {
        ;
    } // wait for STOP to complete
}

// Definitions to make life easier
#define CS LATBbits.LATB7       // chip select pin
#define IMU_Address 0b1101011 // Address for the IMU
#define reg_Accel 0x10
#define reg_Gyro 0x11
#define reg_Contr 0x12
#define reg_OUT_TEMP_L 0x20

// Color Definitions
// Color: RRRRR GGGGGG BBBBB
#define colorRED  0b1111100000000000
#define colorGREEN 0b0000001111100000
#define colorBLUE 0b0000000000011111
#define colorPURPLE 0b1011100000011111
#define colorCYAN 0b0000011100011100
#define colorYELLOW 0b1111111111100000
#define colorORANGE 0b1111101011100000
#define colorBLACK 0b0000000000000000
#define colorWHITE 0b1111111111111111

// Character Dimensions
#define charWidth 5
#define charHeight 8
#define screenWidth 130
#define screenHeight 130

void SPI1_init() {
    SDI1Rbits.SDI1R = 0b0100; // B8 is SDI1
    RPA1Rbits.RPA1R = 0b0011; // A1 is SDO1
    TRISBbits.TRISB7 = 0; // SS is B7
    LATBbits.LATB7 = 1; // SS starts high

    // A0 / DAT pin
    ANSELBbits.ANSB15 = 0;
    TRISBbits.TRISB15 = 0;
    LATBbits.LATB15 = 0;

    SPI1CON = 0; // turn off the spi module and reset it
    SPI1BUF; // clear the rx buffer by reading from it
    SPI1BRG = 1; // baud rate to 12 MHz [SPI1BRG = (48000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0; // clear the overflow bit
    SPI1CONbits.CKE = 1; // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1; // master operation
    SPI1CONbits.ON = 1; // turn on spi1
}

unsigned char spi_io(unsigned char o) {
    SPI1BUF = o;
    while (!SPI1STATbits.SPIRBF) { // wait to receive the byte
        ;
    }
    return SPI1BUF;
}

void LCD_command(unsigned char com) {
    LATBbits.LATB15 = 0; // DAT
    CS = 0; // CS
    spi_io(com);
    CS = 1; // CS
}

void LCD_data(unsigned char dat) {
    LATBbits.LATB15 = 1; // DAT
    CS = 0; // CS
    spi_io(dat);
    CS = 1; // CS
}

void LCD_data16(unsigned short dat) {
    LATBbits.LATB15 = 1; // DAT
    CS = 0; // CS
    spi_io(dat >> 8);
    spi_io(dat);
    CS = 1; // CS
}

void LCD_init() {
    int time = 0;
    LCD_command(CMD_SWRESET); //software reset
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 2) {
    } //delay(500);

    LCD_command(CMD_SLPOUT); //exit sleep
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 200) {
    } //delay(5);

    LCD_command(CMD_PIXFMT); //Set Color Format 16bit
    LCD_data(0x05);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 200) {
    } //delay(5);

    LCD_command(CMD_GAMMASET); //default gamma curve 3
    LCD_data(0x04); //0x04
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 1000) {
    } //delay(1);

    LCD_command(CMD_GAMRSEL); //Enable Gamma adj
    LCD_data(0x01);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 1000) {
    } //delay(1);

    LCD_command(CMD_NORML);

    LCD_command(CMD_DFUNCTR);
    LCD_data(0b11111111);
    LCD_data(0b00000110);

    int i = 0;
    LCD_command(CMD_PGAMMAC); //Positive Gamma Correction Setting
    for (i = 0; i < 15; i++) {
        LCD_data(pGammaSet[i]);
    }

    LCD_command(CMD_NGAMMAC); //Negative Gamma Correction Setting
    for (i = 0; i < 15; i++) {
        LCD_data(nGammaSet[i]);
    }

    LCD_command(CMD_FRMCTR1); //Frame Rate Control (In normal mode/Full colors)
    LCD_data(0x08); //0x0C//0x08
    LCD_data(0x02); //0x14//0x08
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 1000) {
    } //delay(1);

    LCD_command(CMD_DINVCTR); //display inversion
    LCD_data(0x07);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 1000) {
    } //delay(1);

    LCD_command(CMD_PWCTR1); //Set VRH1[4:0] & VC[2:0] for VCI1 & GVDD
    LCD_data(0x0A); //4.30 - 0x0A
    LCD_data(0x02); //0x05
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 1000) {
    } //delay(1);

    LCD_command(CMD_PWCTR2); //Set BT[2:0] for AVDD & VCL & VGH & VGL
    LCD_data(0x02);
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 1000) {
    } //delay(1);

    LCD_command(CMD_VCOMCTR1); //Set VMH[6:0] & VML[6:0] for VOMH & VCOML
    LCD_data(0x50); //0x50
    LCD_data(99); //0x5b
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 1000) {
    } //delay(1);

    LCD_command(CMD_VCOMOFFS);
    LCD_data(0); //0x40
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 1000) {
    } //delay(1);

    LCD_command(CMD_CLMADRS); //Set Column Address
    LCD_data16(0x00);
    LCD_data16(_GRAMWIDTH);

    LCD_command(CMD_PGEADRS); //Set Page Address
    LCD_data16(0x00);
    LCD_data16(_GRAMHEIGH);

    LCD_command(CMD_VSCLLDEF);
    LCD_data16(0); // __OFFSET
    LCD_data16(_GRAMHEIGH); // _GRAMHEIGH - __OFFSET
    LCD_data16(0);

    LCD_command(CMD_MADCTL); // rotation
    LCD_data(0b00001000); // bit 3 0 for RGB, 1 for GBR, rotation: 0b00001000, 0b01101000, 0b11001000, 0b10101000

    LCD_command(CMD_DISPON); //display ON
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000 / 2 / 1000) {
    } //delay(1);

    LCD_command(CMD_RAMWR); //Memory Write
}

void LCD_drawPixel(unsigned short x, unsigned short y, unsigned short color) {
    // check boundary
    LCD_setAddr(x, y, x + 1, y + 1);
    LCD_data16(color);
}

void LCD_setAddr(unsigned short x0, unsigned short y0, unsigned short x1, unsigned short y1) {
    LCD_command(CMD_CLMADRS); // Column
    LCD_data16(x0);
    LCD_data16(x1);

    LCD_command(CMD_PGEADRS); // Page
    LCD_data16(y0);
    LCD_data16(y1);

    LCD_command(CMD_RAMWR); //Into RAM
}

void LCD_clearScreen(unsigned short color) {
    int i;
    LCD_setAddr(0, 0, _GRAMWIDTH, _GRAMHEIGH);
    for (i = 0; i < _GRAMSIZE; i++) {
        LCD_data16(color);
    }
}

void draw_Character(unsigned short x, unsigned short y, unsigned short character, unsigned short frontColor, unsigned short backColor, float fontSize) {
    // Don't draw off the screen
    if (((x + charWidth) <= screenWidth)&&((y + charHeight) <= screenHeight)) {
        // Initialize counter Variables
        int i = 0;
        int j = 0;
        // Loop through columns
        for (i = 0; i < charWidth * fontSize; i++) {
            // Loop through rows
            for (j = 0; j < charHeight * fontSize; j++) {
                // Check if bit at row,column of selected character is 1 or 0 then assign color
                if (((ASCII[character - 0x20][(int) (i / fontSize)])&(0b00000001 << ((int) (j / fontSize)))) != 0) {
                    LCD_drawPixel(x + i, y + j, frontColor);
                } else {
                    LCD_drawPixel(x + i, y + j, backColor);
                }
            }
        }
    }
}

void draw_Message(unsigned short x, unsigned short y, char* message, unsigned short frontColor, unsigned short backColor, float fontSize) {
    // Loop through all characters in string
    int i = 0;
    while (message[i] != 0) {
        draw_Character(x + i * charWidth*fontSize, y, message[i], frontColor, backColor, fontSize);
        i++;
    }
}

void draw_Rectangle(unsigned short x1, unsigned short x2, unsigned short y1, unsigned short y2, unsigned short frontColor) {
    int i = 0;
    int j = 0;
    // Loop through rows
    for (i = 0; i < (x2 - x1); i++) {
        // Loop through columns
        for (j = 0; j < y2 - y1; j++) {
            LCD_drawPixel(x1 + i, y1 + j, frontColor);
        }
    }
}

void draw_HLine(unsigned short x1, unsigned short y1, int length, int thickness, unsigned short frontColor) {
    int i = 0;
    int j = 0;

    if (length > 0) {
        for (i = 0; i < length; i++) {
            for (j = -thickness / 2; j < thickness / 2; j++) {
                LCD_drawPixel(x1 + i, y1 + j, frontColor);
            }
        }
    } else {
        for (i = 0; i < fabs(length); i++) {
            for (j = -thickness / 2; j < thickness / 2; j++) {
                LCD_drawPixel(x1 - i, y1 + j, frontColor);
            }
        }
    }


}

void draw_VLine(unsigned short x1, unsigned short y1, int length, int thickness, unsigned short frontColor) {
    int i = 0;
    int j = 0;

    if (length > 0) {
        for (i = 0; i < length; i++) {
            for (j = -thickness / 2; j < thickness / 2; j++) {
                LCD_drawPixel(x1 + j, y1 + i, frontColor);
            }
        }
    } else {
        for (i = 0; i < fabs(length); i++) {
            for (j = -thickness / 2; j < thickness / 2; j++) {
                LCD_drawPixel(x1 + j, y1 - i, frontColor);
            }
        }
    }

}

void IMU_init(void) {
    // Initializes the IMU
    unsigned char sendbyte;

    i2c_master_start();
    sendbyte = (IMU_Address << 1) | (0b00000000); // Writing    
    i2c_master_send(sendbyte); // Send Device ID with Write Byte
    i2c_master_send(reg_Accel); // CTRL1_XL Register
    i2c_master_send(0b10000010); //1.66 kHz [1000], 2g [00], 100 Hz Filter [10]
    i2c_master_stop();

    i2c_master_start();
    sendbyte = (IMU_Address << 1) | (0b00000000); // Writing    
    i2c_master_send(sendbyte); // Send Device ID with Write Byte
    i2c_master_send(reg_Gyro); // CTRL2_G Register
    i2c_master_send(0b10001000); // 1.66kHz [1000], 1000 dps sense [10], [00]
    i2c_master_stop();

    i2c_master_start();
    sendbyte = (IMU_Address << 1) | (0b00000000); // Writing    
    i2c_master_send(sendbyte); // Send Device ID with Write Byte
    i2c_master_send(reg_Contr); // CTRL3_C Register
    i2c_master_send(0b00000100); // [00000100]
    i2c_master_stop();
}

void I2C_read_multiple(unsigned char address, unsigned char regRead, unsigned char * data, int length) {
    unsigned char sendbyte;

    // Read from registers
    i2c_master_start();
    sendbyte = (IMU_Address << 1) | (0b00000000); // Writing    
    i2c_master_send(sendbyte); // Send Device ID with Write Byte
    i2c_master_send(regRead); // The Register to read
    i2c_master_restart();
    sendbyte = (IMU_Address << 1) | (0b00000001); // Reading    
    i2c_master_send(sendbyte); // Send Device ID with Read Byte

    int i = 0;
    // Read all data
    for (i = 0; i < length; i++) {
        data[i] = i2c_master_recv(); // Receive current register
        if (i == length - 1) {
            i2c_master_ack(1);
        } else {
            i2c_master_ack(0);
        }
    }

    i2c_master_stop(); // End Comms

}

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
 */

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
 */


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize(void) {
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here

    __builtin_enable_interrupts();

    // Configure RB8 as input
    ANSELB = 0;
    TRISBbits.TRISB8 = 1;

    // Configure AN4 as output
    ANSELA = 0;
    TRISAbits.TRISA4 = 0;

}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {

    /* Check the application's current state. */
    switch (appData.state) {
            /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;


            if (appInitialized) {

                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            // Turn on the LED
            LATAbits.LATA4 = 1;

            // Variable for the LED
            char ledIsOn = 1;

            // Initialize the Timer
            _CP0_SET_COUNT(0);
            int timetoWait = 48000000 * 0.1 / 2;
            
            int timetoWaitL = 48000000 * 0.0005 / 2;

            int numberTimer = 0;

            // Configure Bits
            ANSELA = 0;
            ANSELB = 0;

            // Initialize communications
            SPI1_init();
            i2c_master_setup();
            // Initialize the LCD
            LCD_init();
            LCD_clearScreen(colorWHITE);

            // Draw initial message to screen
            char message[100];
            sprintf(message, "SYSTEM ON");
            draw_Message(10, 10, message, colorRED, colorWHITE, 1);

            // Initialize the IMU
            IMU_init();

            // Read WHOAMI
            // Read current state of the expander
            i2c_master_start();
            unsigned char sendbyte = (IMU_Address << 1) | (0b00000000); // Writing
            i2c_master_send(sendbyte); // Send opcode, write
            i2c_master_send(0x0F); // "WHO AM I" - Jackie Chan
            i2c_master_restart();
            sendbyte = (IMU_Address << 1) | (0b00000001); // Reading
            i2c_master_send(sendbyte); // Send opcode, read
            unsigned char checkWhoAmI = i2c_master_recv(); // Receive current state of GPIO
            i2c_master_ack(1);
            i2c_master_stop();

            sprintf(message, "WHO AM I? 0x%x", checkWhoAmI);
            draw_Message(10, 10, message, colorBLACK, colorWHITE, 1);

            // Define Variables
            short temperature;
            short gyroX;
            short gyroY;
            short gyroZ;
            short accelX;
            short accelY;
            short accelZ;

            while (1) {
                // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
                // remember the core timer runs at half the CPU speed

                // Wait for 0.0005 s
                if (_CP0_GET_COUNT() > timetoWait ) {
                    _CP0_SET_COUNT(0);

                    unsigned char data[14];
                    I2C_read_multiple(IMU_Address, reg_OUT_TEMP_L, data, 14);

                    // Shift Data into variables
                    temperature = (data[1] << 8) | data[0];
                    gyroX = (data[3] << 8) | data[2];
                    gyroY = (data[5] << 8) | data[4];
                    gyroZ = (data[7] << 8) | data[6];
                    accelX = (data[9] << 8) | data[8];
                    accelY = (data[11] << 8) | data[10];
                    accelZ = (data[13] << 8) | data[12];
                    // Display the accelerations
                    sprintf(message, "xAccel: %0.2f g", 2.0 * (float) accelX / 32767.0);
                    draw_Message(10, 110, message, colorRED, colorWHITE, 1);
                    sprintf(message, "yAccel: %0.2f g", 2.0 * (float) accelY / 32767.0);
                    draw_Message(10, 120, message, colorGREEN, colorWHITE, 1);

                    // Clear Line Areas
                    draw_HLine(0, 65, 130, 5, colorWHITE);
                    draw_VLine(65, 20, 90, 5, colorWHITE);

                    // Draw Lines
                    draw_HLine(65, 65, -100.0 * (float) accelX / 32767.0, 5, colorRED);
                    draw_VLine(65, 65, -100.0 * (float) accelY / 32767.0, 5, colorGREEN);

                    if (PORTBbits.RB8 == 1) {
                        LATAbits.LATA4 = ~LATAbits.LATA4;
                        ledIsOn = ~ledIsOn;
                    }

                }


            }
            break;
        }


        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}



/*******************************************************************************
 End of File
 */
