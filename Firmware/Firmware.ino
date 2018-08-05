// Original Author: Nick Gammon

// Modified by Vladica Savic

/*
 For original project idea, details, photos, wiring, instructions, see:
    http://www.gammon.com.au/forum/?id=11638
 Copyright 2012 Nick Gammon.
 
 
 PERMISSION TO DISTRIBUTE
 
 Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
 and associated documentation files (the "Software"), to deal in the Software without restriction, 
 including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
 and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,   
 subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in 
 all copies or substantial portions of the Software.
 
 
 LIMITATION OF LIABILITY
 
 The software is provided "as is", without warranty of any kind, express or implied, 
 including but not limited to the warranties of merchantability, fitness for a particular 
 purpose and noninfringement. In no event shall the authors or copyright holders be liable 
 for any claim, damages or other liability, whether in an action of contract, 
 tort or otherwise, arising from, out of or in connection with the software 
 or the use or other dealings in the software. 
*/

#include <SdFatConfig.h>
#include <SdFat.h>
#include <SysCall.h>
#include <Adafruit_GFX.h>    
#include <Adafruit_ST7735.h> 
#include <SPI.h>
#include <Wire.h> 
#include "SdFat.h"

#define EEPROM_ADDRESS 0x50    //Address of 24LC256 eeprom chip  

#define TFT_CS    9
#define TFT_RST   A2     
#define TFT_DC    A3
#define TFT_SCLK  13   
#define TFT_MOSI  11   
#define TFT_LED   5

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

#define JOYSTICK_KEY  A6

const PROGMEM bool allowTargetToRun = true;  // if true, programming lines are freed when not programming

// A and B switches, A to burn bootloader, B to backup current (target) MCU firmware
const PROGMEM byte switchA = A1;
const PROGMEM byte switchB = A0;


// status "messages"
typedef enum { 
  MSG_LINE_TOO_LONG,       // line on disk too long to read
  MSG_LINE_TOO_SHORT,      // line too short to be valid
  MSG_LINE_DOES_NOT_START_WITH_COLON,  // line does not start with a colon
  MSG_INVALID_HEX_DIGITS,  // invalid hex where there should be hex
  MSG_BAD_SUMCHECK,        // line fails sumcheck
  MSG_LINE_NOT_EXPECTED_LENGTH,  // record not length expected
  MSG_UNKNOWN_RECORD_TYPE,  // record type not known
  MSG_NO_END_OF_FILE_RECORD,  // no 'end of file' at end of file
  MSG_FILE_TOO_LARGE_FOR_FLASH,  // file will not fit into flash
  MSG_CANNOT_OPEN_FILE, // cannot open file

  MSG_CANNOT_ENTER_PROGRAMMING_MODE,  // cannot program target chip
  MSG_NO_BOOTLOADER_FUSE,             // chip does not have bootloader
  MSG_CANNOT_FIND_SIGNATURE,      // cannot find chip signature
  MSG_UNRECOGNIZED_SIGNATURE,     // signature not known
  MSG_BAD_START_ADDRESS,          // file start address invalid
  MSG_VERIFICATION_ERROR,         // verification error after programming
  MSG_FLASHED_OK,                 // flashed OK
 } msgType;

const PROGMEM unsigned int ENTER_PROGRAMMING_ATTEMPTS = 2;

// Atmega328 --> [[[[[ PINS ARE REPLACED TO USE "REGULAR" PINS (13, 12, 11, 10) ]]]]]
const PROGMEM byte MSPIM_SCK = 13;  // port B bit 5
const PROGMEM byte BB_MISO   = 12;  // port B bit 4
const PROGMEM byte BB_MOSI = 11;    // port B bit 3
const PROGMEM byte MSPIM_SS  = 10;  // port B bit 2

/*

Connect target processor like this:

  D4: (SCK)   --> SCK as per datasheet        [[[[[[[ 13 ]]]]]]
  D5: (SS)    --> goes to /RESET on target    [[[[[[[ 10 ]]]]]]
  D6: (MISO)  --> MISO as per datasheet       [[[[[[[ 12 ]]]]]]
  D7: (MOSI)  --> MOSI as per datasheet       [[[[[[[ 11 ]]]]]]
  
  D9: 8 Mhz clock signal if required by target 9
*/

// Atmega328
  #define BB_MISO_PORT PINB
  #define BB_MOSI_PORT PORTB
  #define BB_SCK_PORT PORTB
  const PROGMEM byte BB_SCK_BIT = 5;  // pin 13
  const PROGMEM byte BB_MISO_BIT = 4; // pin 12
  const PROGMEM byte BB_MOSI_BIT = 3; // pin 11

// control speed of programming
const PROGMEM byte BB_DELAY_MICROSECONDS = 6;

// target board reset goes to here
const PROGMEM byte RESET = MSPIM_SS;

const PROGMEM unsigned long NO_PAGE = 0xFFFFFFFF;
const PROGMEM int MAX_FILENAME = 13;

const PROGMEM uint8_t SOFT_MISO_PIN = 3;
const PROGMEM uint8_t SOFT_MOSI_PIN = 7;
const PROGMEM uint8_t SOFT_SCK_PIN  = 4;
const PROGMEM uint8_t SD_CHIP_SELECT_PIN = 6;

SdFatSoftSpi<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> SD;

SdFile file;

char * selectedHexFile;

// actions to take
enum {
    checkFile,
    verifyFlash,
    writeToFlash,
};

// copy of fuses/lock bytes found for this processor
byte fuses [5];

// meaning of bytes in above array
enum {
      lowFuse,
      highFuse,
      extFuse,
      lockByte,
      calibrationByte
};

// structure to hold signature and other relevant data about each chip
typedef struct {
   byte sig [3];
   const char * desc;
   unsigned long flashSize;
   unsigned int baseBootSize;
   unsigned long pageSize;     // bytes
   byte fuseWithBootloaderSize;  // ie. one of: lowFuse, highFuse, extFuse
   byte timedWrites;    // if pollUntilReady won't work by polling the chip
} signatureType;

const unsigned long kb = 1024;
const byte NO_FUSE = 0xFF;


// see Atmega datasheets
const signatureType signatures [] PROGMEM = 
  {
//     signature        description   flash size   bootloader  flash  fuse
//                                                     size    page    to
//                                                             size   change

  // Atmega328 family
  { { 0x1E, 0x95, 0x0F }, "ATmega328P",  32 * kb,       512,   128,  highFuse },


  };  // end of signatures
       
// number of items in an array
#define NUMITEMS(arg) ((unsigned int) (sizeof (arg) / sizeof (arg [0])))

// programming commands to send via SPI to the chip
enum {
    progamEnable = 0xAC,
    
      // writes are preceded by progamEnable
      chipErase = 0x80,
      writeLockByte = 0xE0,
      writeLowFuseByte = 0xA0,
      writeHighFuseByte = 0xA8,
      writeExtendedFuseByte = 0xA4,
    
    pollReady = 0xF0,
    
    programAcknowledge = 0x53,
    
    readSignatureByte = 0x30,
    readCalibrationByte = 0x38,
    
    readLowFuseByte = 0x50,       readLowFuseByteArg2 = 0x00,
    readExtendedFuseByte = 0x50,  readExtendedFuseByteArg2 = 0x08,
    readHighFuseByte = 0x58,      readHighFuseByteArg2 = 0x08,  
    readLockByte = 0x58,          readLockByteArg2 = 0x00,  
    
    readProgramMemory = 0x20,  
    writeProgramMemory = 0x4C,
    loadExtendedAddressByte = 0x4D,
    loadProgramMemory = 0x40,
    
};  // end of enum

// which program instruction writes which fuse
const byte fuseCommands [4] = { writeLowFuseByte, writeHighFuseByte, writeExtendedFuseByte, writeLockByte };

// types of record in .hex file
enum {
    hexDataRecord,  // 00
    hexEndOfFile,   // 01
    hexExtendedSegmentAddressRecord, // 02
    hexStartSegmentAddressRecord,  // 03
    hexExtendedLinearAddressRecord, // 04
    hexStartLinearAddressRecord // 05
};


void ShowMessage (const byte which)
  {
  
  // now flash an appropriate sequence
  switch (which)
     {
      // problems reading the .hex file
      case MSG_LINE_TOO_LONG:                   Serial.println(F("MSG_LINE_TOO_LONG")); break;
      case MSG_LINE_TOO_SHORT:                  Serial.println(F("MSG_LINE_TOO_SHORT")); break;
      case MSG_LINE_DOES_NOT_START_WITH_COLON:  Serial.println(F("MSG_LINE_DOES_NOT_START_WITH_COLON")); break;
      case MSG_INVALID_HEX_DIGITS:              Serial.println(F("MSG_INVALID_HEX_DIGITS")); break;
      case MSG_BAD_SUMCHECK:                    Serial.println(F("MSG_BAD_SUMCHECK")); break;
      case MSG_LINE_NOT_EXPECTED_LENGTH:        Serial.println(F("MSG_LINE_NOT_EXPECTED_LENGTH")); break;
      case MSG_UNKNOWN_RECORD_TYPE:             Serial.println(F("MSG_UNKNOWN_RECORD_TYPE")); break;
      case MSG_NO_END_OF_FILE_RECORD:           Serial.println(F("MSG_NO_END_OF_FILE_RECORD")); break;
      
      // problems with the file contents
      case MSG_FILE_TOO_LARGE_FOR_FLASH:        Serial.println(F("MSG_FILE_TOO_LARGE_FOR_FLASH")); break;
      case MSG_CANNOT_OPEN_FILE:                Serial.println(F("MSG_CANNOT_OPEN_FILE")); break;
      
      // problems programming the chip
      case MSG_CANNOT_ENTER_PROGRAMMING_MODE:  Serial.println(F("MSG_CANNOT_ENTER_PROGRAMMING_MODE")); break;
      case MSG_NO_BOOTLOADER_FUSE:             Serial.println(F("MSG_NO_BOOTLOADER_FUSE")); break;
      case MSG_CANNOT_FIND_SIGNATURE:          Serial.println(F("MSG_CANNOT_FIND_SIGNATURE")); break;
      case MSG_UNRECOGNIZED_SIGNATURE:         Serial.println(F("MSG_UNRECOGNIZED_SIGNATURE")); break;
      case MSG_BAD_START_ADDRESS:              Serial.println(F("MSG_BAD_START_ADDRESS")); break;
      case MSG_VERIFICATION_ERROR:             Serial.println(F("MSG_VERIFICATION_ERROR")); break;
      case MSG_FLASHED_OK:                     Serial.println(F("MSG_FLASHED_OK")); break;
      
     default:                                  Serial.println(F("UNKNOWN_ERROR"));  break;   // unknown error
     }  // end of switch on which message 
  }  // end of ShowMessage

  
// if signature found in above table, this is its index
int foundSig = -1;
byte lastAddressMSB = 0;
// copy of current signature entry for matching processor
signatureType currentSignature;

// execute one programming instruction ... b1 is command, b2, b3, b4 are arguments
//  processor may return a result on the 4th transfer, this is returned.
byte program (const byte b1, const byte b2 = 0, const byte b3 = 0, const byte b4 = 0)
  {
  noInterrupts ();
  SPI.transfer (b1);
  SPI.transfer (b2);
  SPI.transfer (b3);
  byte b = SPI.transfer (b4);
  interrupts ();
  return b;
  } // end of program



// read a byte from flash memory
byte readFlash (unsigned long addr)
  {
  byte high = (addr & 1) ? 0x08 : 0;  // set if high byte wanted
  addr >>= 1;  // turn into word address

  // set the extended (most significant) address byte if necessary
  byte MSB = (addr >> 16) & 0xFF;
  if (MSB != lastAddressMSB)
    {
    program (loadExtendedAddressByte, 0, MSB); 
    lastAddressMSB = MSB;
    }  // end if different MSB

  return program (readProgramMemory | high, highByte (addr), lowByte (addr));
  } // end of readFlash
  
// write a byte to the flash memory buffer (ready for committing)
void writeFlash (unsigned long addr, const byte data)
  {
  byte high = (addr & 1) ? 0x08 : 0;  // set if high byte wanted
  addr >>= 1;  // turn into word address
  program (loadProgramMemory | high, 0, lowByte (addr), data);
  } // end of writeFlash  
      
   
// convert two hex characters into a byte
//    returns true if error, false if OK
bool hexConv (const char * (& pStr), byte & b)
  {

  if (!isxdigit (pStr [0]) || !isxdigit (pStr [1]))
    {
    ShowMessage (MSG_INVALID_HEX_DIGITS);
    return true;
    } // end not hex
  
  b = *pStr++ - '0';
  if (b > 9)
    b -= 7;
  
  // high-order nybble
  b <<= 4;
  
  byte b1 = *pStr++ - '0';
  if (b1 > 9)
    b1 -= 7;
    
  b |= b1;
  
  return false;  // OK
  }  // end of hexConv

// poll the target device until it is ready to be programmed
void pollUntilReady ()
  {
  if (currentSignature.timedWrites)
    delay (10);  // at least 2 x WD_FLASH which is 4.5 mS
  else
    {  
    while ((program (pollReady) & 1) == 1)
      {}  // wait till ready  
    }  // end of if
  }  // end of pollUntilReady


unsigned long pagesize;
unsigned long pagemask;
unsigned long oldPage;

// shows progress, toggles working LED
void showProgress ()
  {
        
  }  // end of showProgress
  
// clear entire temporary page to 0xFF in case we don't write to all of it 
void clearPage ()
{
  unsigned int len = currentSignature.pageSize;
  for (unsigned int i = 0; i < len; i++)
    writeFlash (i, 0xFF);
}  // end of clearPage
  
// commit page to flash memory
void commitPage (unsigned long addr)
  {
  addr >>= 1;  // turn into word address
  
  // set the extended (most significant) address byte if necessary
  byte MSB = (addr >> 16) & 0xFF;
  if (MSB != lastAddressMSB)
    {
    program (loadExtendedAddressByte, 0, MSB); 
    lastAddressMSB = MSB;
    }  // end if different MSB
    
  showProgress ();
  
  program (writeProgramMemory, highByte (addr), lowByte (addr));
  pollUntilReady (); 
  
  clearPage();  // clear ready for next page full
  }  // end of commitPage
 
// write data to temporary buffer, ready for committing  
void writeData (const unsigned long addr, const byte * pData, const int length)
  {
  // write each byte
  for (int i = 0; i < length; i++)
    {
    unsigned long thisPage = (addr + i) & pagemask;
    // page changed? commit old one
    if (thisPage != oldPage && oldPage != NO_PAGE)
      commitPage (oldPage);
    // now this is the current page
    oldPage = thisPage;
    // put byte into work buffer
    writeFlash (addr + i, pData [i]);
    }  // end of for
    
  }  // end of writeData
  
// count errors
unsigned int errors;

  
void verifyData (const unsigned long addr, const byte * pData, const int length)
  {
  // check each byte
  for (int i = 0; i < length; i++)
    {
    unsigned long thisPage = (addr + i) & pagemask;
    // page changed? show progress
    if (thisPage != oldPage && oldPage != NO_PAGE)
      showProgress ();
    // now this is the current page
    oldPage = thisPage;
      
    byte found = readFlash (addr + i);
    byte expected = pData [i];
    if (found != expected)
      errors++;
    }  // end of for
    
  }  // end of verifyData
  
bool gotEndOfFile;
unsigned long extendedAddress;

unsigned long lowestAddress;
unsigned long highestAddress;
unsigned long bytesWritten;
unsigned int lineCount;

/*
Line format:

  :nnaaaatt(data)ss
  
  Where:
  :      = a colon
  
  (All of below in hex format)
  
  nn     = length of data part
  aaaa   = address (eg. where to write data)
  tt     = transaction type
           00 = data
           01 = end of file
           02 = extended segment address (changes high-order byte of the address)
           03 = start segment address *
           04 = linear address *
           05 = start linear address *
  (data) = variable length data
  ss     = sumcheck

            * We don't use these
   
*/

// returns true if error, false if OK
bool processLine (const char * pLine, const byte action)
  {
  if (*pLine++ != ':')
     {
     ShowMessage (MSG_LINE_DOES_NOT_START_WITH_COLON);
     return true;  // error
     } 
  
  const int maxHexData = 40;
  byte hexBuffer [maxHexData];
  int bytesInLine = 0;
  
  if (action == checkFile){
    if (lineCount++ % 40 == 0){
      showProgress ();
    }
  }
    
  // convert entire line from ASCII into binary
  while (isxdigit (*pLine))
    {
    // can't fit?
    if (bytesInLine >= maxHexData)
      {
      ShowMessage (MSG_LINE_TOO_LONG);
      return true;
      } // end if too long
      
    if (hexConv (pLine, hexBuffer [bytesInLine++]))
      return true;
    }  // end of while
    
  if (bytesInLine < 5)
    {
    ShowMessage (MSG_LINE_TOO_SHORT);
    return true;  
    } 

  // sumcheck it
  
  byte sumCheck = 0;
  for (int i = 0; i < (bytesInLine - 1); i++)
    sumCheck += hexBuffer [i];
    
  // 2's complement
  sumCheck = ~sumCheck + 1;
  
  // check sumcheck
  if (sumCheck != hexBuffer [bytesInLine - 1])
    {
    ShowMessage (MSG_BAD_SUMCHECK);
    return true;
    }
  
  // length of data (eg. how much to write to memory)
  byte len = hexBuffer [0];
  
  // the data length should be the number of bytes, less
  //   length / address (2) / transaction type / sumcheck
  if (len != (bytesInLine - 5))
    {
    ShowMessage (MSG_LINE_NOT_EXPECTED_LENGTH);
    return true;
    }
    
  // two bytes of address
  unsigned long addrH = hexBuffer [1];
  unsigned long addrL = hexBuffer [2];
  
  unsigned long addr = addrL | (addrH << 8);
  
  byte recType = hexBuffer [3];

  switch (recType)
    {
    // stuff to be written to memory
    case hexDataRecord:
      lowestAddress  = min (lowestAddress, addr + extendedAddress);
      highestAddress = max (lowestAddress, addr + extendedAddress + len - 1);
      bytesWritten += len;
    
      switch (action)
        {
        case checkFile:  // nothing much to do, we do the checks anyway
          break;
          
        case verifyFlash:
          verifyData (addr + extendedAddress, &hexBuffer [4], len);
          break;
        
        case writeToFlash:
          writeData (addr + extendedAddress, &hexBuffer [4], len);
          break;      
        } // end of switch on action
      break;
  
    // end of data
    case hexEndOfFile:
      gotEndOfFile = true;
      break;
  
    // we are setting the high-order byte of the address
    case hexExtendedSegmentAddressRecord: 
      extendedAddress = ((unsigned long) hexBuffer [4]) << 12;
      break;
      
    // ignore these, who cares?
    case hexStartSegmentAddressRecord:
    case hexExtendedLinearAddressRecord:
    case hexStartLinearAddressRecord:
      break;
        
    default:  
      ShowMessage (MSG_UNKNOWN_RECORD_TYPE);
      return true;  
    }  // end of switch on recType
    
  return false;
  } // end of processLine

//------------------------------------------------------------------------------
// returns true if error, false if OK
bool readHexFile (const byte action)
  {
  const int maxLine = 80;
  char buffer [80];
  
  int lineNumber = 0;
  gotEndOfFile = false;
  extendedAddress = 0;
  errors = 0;
  lowestAddress = 0xFFFFFFFF;
  highestAddress = 0;
  bytesWritten = 0;

  pagesize = currentSignature.pageSize;
  pagemask = ~(pagesize - 1);
  oldPage = NO_PAGE;
  
  ifstream sdin (selectedHexFile); 
   
  // check for open error
  if (!sdin.is_open()) 
  {
    ShowMessage (MSG_CANNOT_OPEN_FILE);
    return true;
  }
    
  switch (action)
    {
    case checkFile:
      break;
      
    case verifyFlash:
      break;
    
    case writeToFlash:
      program (progamEnable, chipErase);   // erase it
      delay (20);  // for Atmega8
      pollUntilReady (); 
      clearPage();  // clear temporary page
      break;      
    } // end of switch

    while (sdin.getline (buffer, maxLine))
    {
    lineNumber++;
    int count = sdin.gcount();
    if (sdin.fail()) 
      {
      ShowMessage (MSG_LINE_TOO_LONG);
      return true;
      }  // end of fail (line too long?)
      
    // ignore empty lines
    if (count > 1)
      {
      if (processLine (buffer, action))
        {
        return true;  // error
        }
      }
    }    // end of while each line
    
  if (!gotEndOfFile)
    {
    ShowMessage (MSG_NO_END_OF_FILE_RECORD);
    return true;
    }

  switch (action)
    {
    case writeToFlash:
      // commit final page
      if (oldPage != NO_PAGE)
        commitPage (oldPage);
      break;
      
    case verifyFlash:
       if (errors > 0)
          {
          ShowMessage (MSG_VERIFICATION_ERROR);
          return true;
          }  // end if
       break;
        
    case checkFile:
      break;
    }  // end of switch
  
  return false;
}  // end of readHexFile


// returns true if managed to enter programming mode
bool startProgramming ()
  {

  byte confirm;
  pinMode (RESET, OUTPUT);
  digitalWrite (RESET, HIGH);  // ensure SS stays high for now
  SPI.begin ();
  SPI.setClockDivider (SPI_CLOCK_DIV64);
  pinMode (SCK, OUTPUT);
  
  unsigned int timeout = 0;
  
  // we are in sync if we get back programAcknowledge on the third byte
  do 
    {
    // regrouping pause
    delay (100);

    // ensure SCK low
    noInterrupts ();
    digitalWrite (MSPIM_SCK, LOW);
    // then pulse reset, see page 309 of datasheet
    digitalWrite (RESET, HIGH);
    delayMicroseconds (10);  // pulse for at least 2 clock cycles
    digitalWrite (RESET, LOW);
    interrupts ();

    delay (25);  // wait at least 20 mS
    noInterrupts ();
    SPI.transfer (progamEnable);  
    SPI.transfer (programAcknowledge);  
    confirm = SPI.transfer (0);  
    SPI.transfer (0);  
    interrupts ();
    
    if (confirm != programAcknowledge)
      {
      if (timeout++ >= ENTER_PROGRAMMING_ATTEMPTS){
          return false;
        }
      }  // end of not entered programming mode
    
    } while (confirm != programAcknowledge);
  return true;  // entered programming mode OK
}  // end of startProgramming

void stopProgramming ()
  {
    
  digitalWrite (RESET, LOW);
  pinMode (RESET, INPUT);
  SPI.end ();
    
  } // end of stopProgramming
  
void getSignature ()
  {
  foundSig = -1;
  lastAddressMSB = 0;
    
  byte sig [3];
  for (byte i = 0; i < 3; i++)
    {
    sig [i] = program (readSignatureByte, 0, i); 
    }  // end for each signature byte
  
  for (unsigned int j = 0; j < NUMITEMS (signatures); j++)
    {
    memcpy_P (&currentSignature, &signatures [j], sizeof currentSignature);
    
    if (memcmp (sig, currentSignature.sig, sizeof sig) == 0)
      {
      foundSig = j;
      // make sure extended address is zero to match lastAddressMSB variable
      program (loadExtendedAddressByte, 0, 0); 
      return;
      }  // end of signature found
    }  // end of for each signature

  ShowMessage (MSG_UNRECOGNIZED_SIGNATURE);
  }  // end of getSignature
  
void getFuseBytes ()
  {
  fuses [lowFuse]   = program (readLowFuseByte, readLowFuseByteArg2);
  fuses [highFuse]  = program (readHighFuseByte, readHighFuseByteArg2);
  fuses [extFuse]   = program (readExtendedFuseByte, readExtendedFuseByteArg2);
  fuses [lockByte]  = program (readLockByte, readLockByteArg2);
  fuses [calibrationByte]  = program (readCalibrationByte);
  }  // end of getFuseBytes

  
// write specified value to specified fuse/lock byte
void writeFuse (const byte newValue, const byte instruction)
  {
  if (newValue == 0)
    return;  // ignore
  
  program (progamEnable, instruction, 0, newValue);
  pollUntilReady (); 
  }  // end of writeFuse
  
// returns true if error, false if OK
bool updateFuses (const bool writeIt)
  {
  unsigned long addr;
  unsigned int  len;
  
  byte fusenumber = currentSignature.fuseWithBootloaderSize;
  
  // if no fuse, can't change it
  if (fusenumber == NO_FUSE)
    {
//    ShowMessage (MSG_NO_BOOTLOADER_FUSE);   // maybe this doesn't matter?
    return false;  // ok return
    }
    
  addr = currentSignature.flashSize;
  len = currentSignature.baseBootSize;
    
  if (lowestAddress == 0)
    {
    // don't use bootloader  
    fuses [fusenumber] |= 1;
    }
  else 
    {
    byte newval = 0xFF;
    
    if (lowestAddress == (addr - len))
      newval = 3;
    else if (lowestAddress == (addr - len * 2))
      newval = 2;
    else if (lowestAddress == (addr - len * 4))
      newval = 1;
    else if (lowestAddress == (addr - len * 8))
      newval = 0;
    else
      {
      ShowMessage (MSG_BAD_START_ADDRESS);
      return true;
      }
      
    if (newval != 0xFF)
      {
      newval <<= 1; 
      fuses [fusenumber] &= ~0x07;   // also program (clear) "boot into bootloader" bit 
      fuses [fusenumber] |= newval;  
      }  // if valid
      
    }  // if not address 0
  
  if (writeIt)
    {
    writeFuse (fuses [fusenumber], fuseCommands [fusenumber]);      
    }
    
  return false;
  }  // end of updateFuses


//------------------------------------------------------------------------------
//      TFT MENU
//------------------------------------------------------------------------------
char* menu[20] = {"CYCLE ALPHA"};

int numMenu = 0;

#define menu_top 14

char menu_select;    
char keydown=0;     


//------------------------------------------------------------------------------
//      TFT MENU INIT
//------------------------------------------------------------------------------
void tftMenuInit()
{
  // Clear screen and display the menu
  int i;
  
  tft.setTextWrap(false);
  tft.fillScreen(ST7735_BLACK);
  tft.setTextSize(1);
  
  tft.setCursor(0, 0);
  tft.setTextColor(ST7735_GREEN, ST7735_BLACK); 
  tft.println(menu[0]);

  tft.drawLine(0, 9, tft.width()-1, 9, ST7735_GREEN);
  
  tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);  
  
  for(i=1;i<sizeof(menu);i++)
  {  
     tft.setCursor(0, ((i-1)*10)+menu_top); 
     delay(10);   
     tft.println(menu[i]);
     delay(10);
  }  
} // end of tftMenuInit

//------------------------------------------------------------------------------
//      TFT MENU SELECT
//------------------------------------------------------------------------------
void tftMenuSelect(char menuitem) 
{
  // Highlight a selected menu item
  char i;
  // Remove highlight of current item
  tft.setCursor(0, ((menu_select-1)*10)+menu_top);
  tft.setTextColor(ST7735_YELLOW, ST7735_BLACK); 
  tft.println(menu[menu_select]); 
  // Highlight new menu item
  tft.setCursor(0, ((menuitem-1)*10)+menu_top);
  tft.setTextColor(ST7735_YELLOW, ST7735_BLUE); 
  tft.println(menu[menuitem]);
  // change menu_select to new item  
  menu_select=menuitem;

} // end of tftMenuSelect

//------------------------------------------------------------------------------
//      DISPLAY TEXT
//------------------------------------------------------------------------------

void displayText(__FlashStringHelper *text,uint16_t color) {
    tft.setTextWrap(false);
    tft.fillScreen(ST7735_BLACK);
    tft.setTextSize(1);
    
    tft.setCursor(0, 0);
    tft.setTextColor(ST7735_GREEN, ST7735_BLACK); 
    tft.println(menu[0]);
  
    tft.drawLine(0, 9, tft.width()-1, 9, ST7735_GREEN);
    
    tft.setTextColor(color, ST7735_BLACK);  
    tft.setCursor(0, menu_top);    
    tft.setTextWrap(true);
    tft.println(text);
} // end of displayText

//------------------------------------------------------------------------------
//      DISPLAY ERROR
//------------------------------------------------------------------------------
void displayError(__FlashStringHelper *text) {
    displayText(text, ST7735_RED);
    
    delay(1500); // give some time for message to be read
} // end of displayError

//------------------------------------------------------------------------------
//      WRITE EEPROM
//------------------------------------------------------------------------------
void writeEEPROM(int deviceaddress, unsigned int eeaddress, char* data) {   
  // Uses Page Write for 24LC256   
  // Allows for 64 byte page boundary   
  // Splits string into max 16 byte writes   
  unsigned char i=0, counter=0;   
  unsigned int address;   
  unsigned int page_space;  
  unsigned int page=0;   
  unsigned int num_writes;   
  unsigned int data_len=0;   
  unsigned char first_write_size;   
  unsigned char last_write_size;   
  unsigned char write_size; 
  
  // Calculate length of data   
  do{ data_len++; } while(data[data_len]);  
  
  // Calculate space available in first page   
  page_space = int(((eeaddress/64) + 1)*64)-eeaddress;
  
  // Calculate first write size   
  if (page_space>16){      
    first_write_size=page_space-((page_space/16)*16);      
    if (first_write_size==0){ 
      first_write_size=16;   
    }
  }    
  else {     
    first_write_size=page_space; 
  }  
  
  // calculate size of last write   
  if (data_len>first_write_size){      
    last_write_size = (data_len-first_write_size)%16;
  }
  
  // Calculate how many writes we need   
    if (data_len>first_write_size){      
      num_writes = ((data_len-first_write_size)/16)+2;
    }   
    else {     
      num_writes = 1; 
    }    
    
  i=0;      
  address=eeaddress;  
  for(page=0;page<num_writes;page++)   {      
    if(page==0) {write_size=first_write_size;}     
    else if(page==(num_writes-1)){ 
      write_size=last_write_size;
    }      
    else{
      write_size=16; 
    } 
    
    Wire.beginTransmission(deviceaddress);      
    Wire.write((int)((address) >> 8)); // MSB      
    Wire.write((int)((address) & 0xFF)); // LSB      
    counter=0;      
    do{         
      Wire.write((byte) data[i]);         
      i++;         
      counter++;      
    } 
    while((data[i]) && (counter<write_size));     
    Wire.endTransmission();     
    address+=write_size;   // Increment address for next write            
    delay(6); // needs 5ms for page write   
  } 
} // end of writeEEPROM

//------------------------------------------------------------------------------
//      READ EEPROM
//------------------------------------------------------------------------------
void readEEPROM(int deviceaddress, unsigned int eeaddress, unsigned char* data, unsigned int num_chars) {   
  unsigned char i=0;   
  Wire.beginTransmission(deviceaddress);   
  Wire.write((int)(eeaddress >> 8)); // MSB   
  Wire.write((int)(eeaddress & 0xFF)); // LSB   
  Wire.endTransmission();     
  Wire.requestFrom(deviceaddress,num_chars);     
  while(Wire.available()) data[i++] = Wire.read();
} // end of readEEPROM


//------------------------------------------------------------------------------
//      FREE MEMORY - DEBUG ONLY
//------------------------------------------------------------------------------
#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
} // end of freeMemory

//------------------------------------------------------------------------------
//      SETUP
//------------------------------------------------------------------------------
void setup () 
  {
   
  pinMode(switchA, INPUT_PULLUP);
  pinMode(switchB, INPUT_PULLUP);
    
  pinMode (TFT_LED, OUTPUT);
  digitalWrite (TFT_LED, LOW);
  
  Serial.begin(9600);

   while (!Serial) {
    SysCall::yield();
  }

  //>>> DISPLAY
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  
  tft.fillScreen(ST7735_BLACK);
  
  delay(100);
  tft.setRotation(3);
  
  delay(100);
  displayText(F("What I cannot create,\nI do not understand!"), ST7735_WHITE);

  digitalWrite (TFT_LED, HIGH);
  
  delay(2500);

  Serial.print(F("Initializing SD card..."));
  

  if (!SD.begin(SD_CHIP_SELECT_PIN, SD_SCK_MHZ(12))) {
    Serial.println(F("Initialization failed!"));
    displayError(F("\nERROR\n\nSD Card\nInitialization Failed!\n\nInsert SD Card\nAnd Reboot Board."));
    SD.initErrorHalt();
    return;
  }

  Serial.println(F("Initialization done."));

  delay(100);
  
  numMenu = 0;   
     
  SdFile file;
   while (file.openNext(SD.vwd(), O_READ)) {
     // Skip directories (e.g. System Volume Information)
     if (!file.isHidden() && file.isFile()) {
        char* fileName;
        
        fileName = new char [25]; 
        
        numMenu++;
        file.getName(fileName, 25);

        menu[numMenu] = fileName;
     }
     file.close();
  }
  
  delay(100);
   
  tftMenuInit();                    // Draw menu
  menu_select=1;                    // Select 1st menu item 
  tftMenuSelect(menu_select);       // Highlight selected menu item
  //<<< DISPLAY

  delay(200);
}  // end of setup

void getline (char * buf, size_t bufsize)
{
byte i;

  // discard any old junk
  while (Serial.available ())
    Serial.read ();

  for (i = 0; i < bufsize - 1; )
    {
    if (Serial.available ())
      {
      int c = Serial.read ();

      if (c == '\n')  // newline terminates
        break;

      if (!isspace (c))  // ignore spaces, carriage-return etc.
        buf [i++] = toupper (c);
      } // end if available
    }  // end of for
  buf [i] = 0;  // terminator
  Serial.println (buf);  // echo what they typed
  }     // end of getline


void readFlashContents ()
  {
  Serial.println (F("CREATING BACKUP"));
  
  pagesize = currentSignature.pageSize;
  pagemask = ~(pagesize - 1);
  oldPage = NO_PAGE;
  byte lastMSBwritten = 0;

  char name[13] = {"Backup.hex"};

  // ensure back in programming mode
  if (!startProgramming ()){
    return;
  }

  SdFile backupFile;

  // open the file for writing
  if (!backupFile.open(name, O_WRITE | O_CREAT | O_TRUNC))
    {
    Serial.print (F("Could not open file for backup."));
    return;
    }

  byte memBuf [16];
  unsigned int i;
  char linebuf [50];
  byte sumCheck;
  
  for (unsigned long address = 0; address < 32768; address += sizeof memBuf)
    {
    bool allFF;

    unsigned long thisPage = address & pagemask;
    // page changed? show progress
    if (thisPage != oldPage && oldPage != NO_PAGE)
      showProgress ();
    // now this is the current page
    oldPage = thisPage;

    // don't write lines that are all 0xFF
    allFF = true;

    for (i = 0; i < sizeof memBuf; i++)
      {
      memBuf [i] = readFlash (address + i);
      if (memBuf [i] != 0xFF)
        allFF = false;
      }  // end of reading 16 bytes
    if (allFF)
      continue;

    byte MSB = address >> 16;
    if (MSB != lastMSBwritten)
      {
      sumCheck = 2 + 2 + (MSB << 4);
      sumCheck = ~sumCheck + 1;
      // hexExtendedSegmentAddressRecord (02)
      sprintf (linebuf, ":02000002%02X00%02X\r\n", MSB << 4, sumCheck);
      backupFile.print (linebuf);
      lastMSBwritten = MSB;
      }  // end if different MSB

    sumCheck = 16 + lowByte (address) + highByte (address);
    sprintf (linebuf, ":10%04X00", (unsigned int) address & 0xFFFF);
    for (i = 0; i < sizeof memBuf; i++)
      {
      sprintf (&linebuf [(i * 2) + 9] , "%02X",  memBuf [i]);
      sumCheck += memBuf [i];
      }  // end of reading 16 bytes

    // 2's complement
    sumCheck = ~sumCheck + 1;
    // append sumcheck
    sprintf (&linebuf [(sizeof memBuf * 2) + 9] , "%02X\r\n",  sumCheck);

    backupFile.clearWriteError ();
    backupFile.print (linebuf);
    if (backupFile.getWriteError ())
       {
       Serial.println (F("Error writing file."));
       backupFile.close ();
       return;
       }   // end of an error

    }  // end of reading flash

  backupFile.print (":00000001FF\r\n");    // end of file record
  
  backupFile.close ();
  // ensure written to disk
  
  SD.vwd()->sync ();
  Serial.println (F("BACKUP SAVED"));
  }  // end of readFlashContents

// returns true if error, false if OK
bool chooseInputFile ()
  {
 
  if (readHexFile(checkFile))
    {
    return true;  // error, don't attempt to write
    }
  
  // check file would fit into device memory
  if (highestAddress > currentSignature.flashSize)
    {
    ShowMessage (MSG_FILE_TOO_LARGE_FOR_FLASH);
    return true; 
    }
  
  // check start address makes sense
  if (updateFuses (false))
    {
    return true;
    }
  
   return false;   
  }  // end of chooseInputFile

// returns true if OK, false on error
bool writeFlashContents ()
  { 
    errors = 0;
    
    if (chooseInputFile ()){
      return false;  
    }
    
    // ensure back in programming mode  
    if (!startProgramming ()){
      return false;
    }
  
    // now commit to flash
    if (readHexFile(writeToFlash)){
      return false;
    }
  
    // verify
    if (readHexFile(verifyFlash)){
      return false;
    }
  
    // now fix up fuses so we can boot    
    if (errors == 0){
      updateFuses (true);
    }
    
    return errors == 0;
  }  // end of writeFlashContents


//------------------------------------------------------------------------------
//      DO WORK
//------------------------------------------------------------------------------
void doWork (int menuIndex) {
  
  selectedHexFile = menu[menuIndex];

  Serial.println(selectedHexFile);

  Serial.println(F("FLASHING"));

  //--------
  tft.fillScreen(ST7735_BLACK);
  
  delay(50);
  displayText(F("PROGRAMMING IN PROGRESS,\nPLEASE WAIT!"), ST7735_YELLOW);
  //--------
  if (!startProgramming ())
    {
      ShowMessage (MSG_CANNOT_ENTER_PROGRAMMING_MODE);
      displayError(F("\nERROR\n\nCAN NOT ENTER\nPROGRAMMING MODE!"));
      return;
    }  // end of could not enter programming mode
    
  getSignature ();
  getFuseBytes ();
  
  // don't have signature? don't proceed
  if (foundSig == -1)
    {
      ShowMessage (MSG_CANNOT_FIND_SIGNATURE);
      return;
    }  // end of no signature
  
  bool ok = writeFlashContents ();
  
  stopProgramming ();
  delay (500);
  
  if (ok){
    ShowMessage (MSG_FLASHED_OK); 
  }
  
} // end of doWork


//------------------------------------------------------------------------------
//      LOOP
//------------------------------------------------------------------------------
void loop () 
{
  int x;
  x = analogRead (JOYSTICK_KEY);

  // DIRTY HACK, because pull-up was missing and first\imidiate read is "0" and triggeres the action with no good reason (probably very low SRAM)
  delay(10);
  x = analogRead (JOYSTICK_KEY); 
  
  Serial.println(x);
  
  if ((x < 100) & (keydown==0)) {
    // Select
    keydown=1;
                                    // Note the syntax for doing this
    doWork(menu_select);

    delay(100);
    tftMenuInit();                  // Redraw the Menu
    tftMenuSelect(menu_select);     // Highlight the current menu item
  }
  else if ((x < 200) & (keydown==0)){
    // Down
    // move down one menu item, if at bottom wrap to top
    keydown=1;    
    if (menu_select<numMenu) tftMenuSelect(menu_select+1);
    else tftMenuSelect(1);   
  }
  else if ((x < 400) & (keydown==0)){
    // Right
    keydown=1;    
  }  
   else if ((x < 550) & (keydown==0)) {
    // Up
    // move up one menu item, if at top wrap to bottom
    keydown=1;    
    if (menu_select>1) tftMenuSelect(menu_select-1);
    else tftMenuSelect(numMenu);
  }
  else if ((x < 800) & (keydown==0)){
    // Left
    keydown=1;    
  }
  else if (x >= 800)
  {
    keydown=0; // key released
  }

  // A and B buttons 

  // "A" switch not in use at the moment
  
  int switchAState = 0;
  switchAState = digitalRead(switchA);

  if (switchAState == LOW) {
    // Burn bootloader
  }
  /**/

  int switchBState = 0;
  switchBState = digitalRead(switchB);
  
  if (switchBState == LOW) {
    // Backup current target MCU firmware  
    //--------
    tft.fillScreen(ST7735_BLACK);  
    delay(50);
    displayText(F("CREATING BACKUP,\nPLEASE WAIT!"), ST7735_YELLOW);
    
    readFlashContents();
    
    delay(100);
    tftMenuInit();                  // Redraw the Menu
    tftMenuSelect(menu_select); 
    /*
    Serial.print(F( "debug: Free ram = " ) );
    Serial.println( freeMemory() ) ;
    Serial.flush() ;
    */
  }
}  // end of loop