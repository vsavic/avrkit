# Cycle Alpha - AVR Programmer Board - Quick Start Guide
**AVR-based microcontroller board, designed to help in small to medium production run**.

## Board Overview

This board is designed to be a portable, stand-alone programmer tool. We can store pre-compiled HEX files on SD card and burn them on new ATMega328 MCU.

Board supports DIP and SMD version of ATMega328 MCU via ZIF28 and TQFP32 sockets. Also there’s a direct ICSP line connected to these sockets via master \ slave jumper switch configuration.

Here’s a quick summary of some of the most important parts of the board:

1. SD Card
2. 5-Way Joystick
3. Button B
4. USB
5. Button A
6. 1.8” TFT Display
7. Power Jack
8. ICSP Header
9. Slave \ Master Selector
10. TQFP-32 SMT Socket
11. ZIF28 DIP Socket

Please refer to the image below to match the numbers with the coresponding components.

![Cycle Alpha - AVR Programmer Board](https://github.com/vsavic/avrkit/blob/master/Artwork/CA-AVR.png)


## Communcation
Board can be re-programmed via ICSP header, or directly via USB port.

It contains 1.8” TFT display which can be used with an on-board 5-way joystick in order to select the proper firmware which can be flashed on new MCU.

There’s also 2 additional buttons which are used to burn bootloader and to make a backup of the current code from the target MCU.

## Power
Board can be powered via USB cable or with external power supply battery connected to DC jack from 7 to 25 V DC. 

## Roadmap
* Add support to burn bootloader from EEPROM.
* Add support to store current target MCU firmware into EEPROM for further clonning.

At the moment, we can make a backup of the current firmware from target MCU by using "B" button on the board. Firmware will be stored on micro SD card as `Backup.hex` file.

## Notes
Common problem with some micro SD cards is that some of them cannot be initialized. In case that you get an serror screen with your SD card, follow the instruction. For more details, see `serial` console output.

SD card must be in FAT16 or FAT32 format in order to work with [SdFat](https://github.com/greiman/SdFat) library.

I tried a few different manufacturers, but the most reliable results I got with are with a Kingston 8GB class 10 micro SD card.

In case that you need to re-format your card, use this [SD Formatter](https://www.sdcard.org/downloads/formatter_4/index.html).

If you decide to modify the current firmware, keep in mind that SRAM is already very low.

Check this article for more details on [arduino memory problems](https://learn.adafruit.com/memories-of-an-arduino?view=all).

Also, here's one more really good article about [PROGMEM](http://www.gammon.com.au/progmem).

## Special Thanks

Special thanks to [Nick Gammon](https://www.gammon.com.au/contactus.htm), without him, his original project and amazing articles, this board wouldn't exist. 