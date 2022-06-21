 /**
 * > Open Source Laboratory Instrument Project:
 *   Control Software for Teensy 4.1 board
 * 
 * > Author
 *   Michele Sorelli 2022
 * 
 */
#include <Arduino.h>


/** 
 * Teensy 4.1 Custom Serial I/O : GPIO6_DR pin mapping (from msb to lsb) 
 * 
 * 27 26 39 38   21 20 23 22   16 17 41 40   15 14 18 19   X X 25 24   X X X X   X X X X   0 1 X X
                                                                          SCLK  |MOSI PINS|
 *                                                                        
 */


// Global variables

// PC serial communication
bool g_clientCXN = false;               // USB serial connection established
bool g_startCOM  = true;                // start serial communication flag
unsigned long g_currenTime;
unsigned long g_starTime;
unsigned long g_byteCount = 0;


// digital pins (to AD9959 DDS)            Connection         Function
const byte c_PwnDwn    = 2;             // output to DUC   /  DUC power-down control
const byte c_HardReset = 3;             // output to DUC   /  DUC master reset
const byte c_IOUpdate  = 5;             // output to DUC   /  data I/O update 
const byte c_ChipSel   = 10;            // output to DUC   /  DUC chip select
const byte c_HardResetInterrupt = 28;   // input           /  trigger DUC hard reset routine
const byte c_SoftResetInterrupt = 30;   // input           /  trigger MCU soft reset routine
const byte c_UpdateInterrupt    = 32;   // input           /  trigger DUC update routine

// quad-SPI (data pins ordered from msb to lsb, + SCLK)
const byte c_quadSPIPins[5] = {15, 14, 18, 19, 40};
const byte c_QuadSclk = 40;
const byte c_QuadSclkDiv = 2;

// safe-clear mask for GPIO6_DR port pins
// 1111 1111 1110 0000 1111 1111 1111 1111                                           
#define c_SafeClearGPIO6Bit5(n)  (n & 0xffe0ffff) 

// serial clock pin masks
#define c_SclkHigh 0b00000000000100000000000000000000

// nibble masks
#define c_Mask8Nibble04 0b11110000
#define c_Mask8Nibble14 0b00001111
#define c_Mask8Nibble02 0b11000000
#define c_Mask8Nibble12 0b00110000
#define c_Mask8Nibble22 0b00001100
#define c_Mask8Nibble32 0b00000011
#define c_Mask8Nibble01 0b10000000
#define c_Mask8Nibble11 0b01000000
#define c_Mask8Nibble21 0b00100000
#define c_Mask8Nibble31 0b00010000
#define c_Mask8Nibble41 0b00001000
#define c_Mask8Nibble51 0b00000100
#define c_Mask8Nibble61 0b00000010
#define c_Mask8Nibble71 0b00000001
#define c_ReadMask 0x80


// AD9959 DDS (Device Under Control)
// clock variables
const byte  c_PLLDiv  = 20;
const float c_RefClk  = 25; 
const float c_SysClk  = c_PLLDiv * c_RefClk;  //  [MHz]   DDS SYS_CLK  = PLL factor * REF_CLK = 20 * 25 MHz = 500 MHz
const float c_SyncClk = 0.25 * c_SysClk;      //  [MHz]   DDS SYNC_CLK = 0.25 * SYS_CLK = 125 MHz
const byte  c_Tassert = 1;                    //  [μs]    DDS master reset and I/O update assert time 
                                              //          (must be higher than 1/SYNC_CLK μs, i.e. 8 ns)

// AD9959 bit precision
const byte c_DDSbits = 32; 

// AD9959 DAC channels
const byte c_Nch = 4;
const byte c_chSel0 = 0b00010110; // selection bytes
const byte c_chSel1 = 0b00100110;
const byte c_chSel2 = 0b01000110;
const byte c_chSel3 = 0b10000110;
const byte c_Ch0 = 0;             // custom IDs
const byte c_Ch1 = 1;
const byte c_Ch2 = 2;
const byte c_Ch3 = 3;
const byte c_Ch0Ch1 = 10;
const byte c_Ch0Ch2 = 20;
const byte c_Ch0Ch3 = 30;
const byte c_Ch1Ch2 = 12;
const byte c_Ch1Ch3 = 13;
const byte c_Ch2Ch3 = 23;
const byte c_Ch1Ch2Ch3 = 123;
const byte c_Ch0Ch1Ch3 = 130;
const byte c_Ch0Ch1Ch2 = 210;
const byte c_Ch0Ch2Ch3 = 230;
const byte c_ChAll = 4;


// AD9959 register addresses
const byte c_CSR     = 0x00;  // Channel Selection Register
const byte c_FR1     = 0x01;  // Function Register 1
const byte c_CFR     = 0x03;  // Channel Function Register
const byte c_CFTW0   = 0x04;  // Channel Frequency Tuning Word 0 (chirp minimum frequency)
const byte c_CPOW0   = 0x05;  // Channel Phase Offset Word 0
const byte c_LSRR    = 0x07;  // Linear Sweep Ramp Rate
const byte c_RDW     = 0x08;  // LSR Rising Delta Word
const byte c_FDW     = 0x09;  // LSR Falling Delta Word
const byte c_CFTW1   = 0x0A;  // Channel Frequency Tuning Word 1 (chirp maximum frequency)

// AD9959 register sizes
const byte c_SelSize = 2;
const byte c_FR1Size = 4;
const byte c_CFRSize = 4;


// input to MCU board
String g_inString  = "";      // input instruction string
float g_parsedValues[24];     // array of parsed values
unsigned int g_numIn = 0;     // overall input setting counter
unsigned int g_inCh0 = 0;     // channel 0 memory index (in)
unsigned int g_inCh1 = 0;     // channel 1 memory index (in)
unsigned int g_inCh2 = 0;     // channel 2 memory index (in)
unsigned int g_inCh3 = 0;     // channel 3 memory index (in)


// output from MCU board
volatile unsigned int g_numOut = 0;   // overall output setting counter
volatile unsigned int g_outCh0 = 0;   // channel 0 memory index (out)
volatile unsigned int g_outCh1 = 0;   // channel 1 memory index (out)
volatile unsigned int g_outCh2 = 0;   // channel 2 memory index (out)
volatile unsigned int g_outCh3 = 0;   // channel 3 memory index (out)

// frequency tuning word buffers
const unsigned int c_maxSize = 5500;     // max buffer size
unsigned int g_bufferFTW0Ch0[c_maxSize]; // channel 0 buffer: EITHER frequency tuning word (Single Tone Mode) OR chirp start frequency tuning word (Linear Sweep Mode)
unsigned int g_bufferFTW1Ch0[c_maxSize]; // channel 0 buffer: chirp end frequency tuning word (Linear Sweep Mode only)             
unsigned int g_bufferFTW0Ch1[c_maxSize]; // channel 1 buffer: EITHER frequency tuning word (Single Tone Mode) OR chirp start frequency tuning word (Linear Sweep Mode)      
unsigned int g_bufferFTW1Ch1[c_maxSize]; // channel 1 buffer: chirp end frequency tuning word (Linear Sweep Mode only)      
unsigned int g_bufferFTW0Ch2[c_maxSize]; // channel 2 buffer: EITHER frequency tuning word (Single Tone Mode) OR chirp start frequency tuning word (Linear Sweep Mode)
unsigned int g_bufferFTW1Ch2[c_maxSize]; // channel 2 buffer: chirp end frequency tuning word (Linear Sweep Mode only)
unsigned int g_bufferFTW0Ch3[c_maxSize]; // channel 3 buffer: EITHER frequency tuning word (Single Tone Mode) OR chirp start frequency tuning word (Linear Sweep Mode)
unsigned int g_bufferFTW1Ch3[c_maxSize]; // channel 3 buffer: chirp end frequency tuning word (Linear Sweep Mode only)

// linear frequency sweep time/frequency step tuning words
unsigned int g_bufferRDWCh0[c_maxSize];  // channel 0 buffer: Rising  (frequency) Delta Word buffer
unsigned int g_bufferRDWCh1[c_maxSize];  // channel 1 buffer: Rising  (frequency) Delta Word buffer
unsigned int g_bufferRDWCh2[c_maxSize];  // channel 2 buffer: Rising  (frequency) Delta Word buffer
unsigned int g_bufferRDWCh3[c_maxSize];  // channel 3 buffer: Rising  (frequency) Delta Word buffer
unsigned int g_bufferFDWCh0[c_maxSize];  // channel 0 buffer: Falling (frequency) Delta Word buffer
unsigned int g_bufferFDWCh1[c_maxSize];  // channel 1 buffer: Falling (frequency) Delta Word buffer
unsigned int g_bufferFDWCh2[c_maxSize];  // channel 2 buffer: Falling (frequency) Delta Word buffer
unsigned int g_bufferFDWCh3[c_maxSize];  // channel 3 buffer: Falling (frequency) Delta Word buffer
byte         g_bufferRSRRCh0[c_maxSize]; // channel 0 buffer: Rising Step Ramp Rate
byte         g_bufferRSRRCh1[c_maxSize]; // channel 1 buffer: Rising Step Ramp Rate
byte         g_bufferRSRRCh2[c_maxSize]; // channel 2 buffer: Rising Step Ramp Rate
byte         g_bufferRSRRCh3[c_maxSize]; // channel 3 buffer: Rising Step Ramp Rate
byte         g_bufferFSRRCh0[c_maxSize]; // channel 0 buffer: Falling Step Ramp Rate
byte         g_bufferFSRRCh1[c_maxSize]; // channel 1 buffer: Falling Step Ramp Rate
byte         g_bufferFSRRCh2[c_maxSize]; // channel 2 buffer: Falling Step Ramp Rate
byte         g_bufferFSRRCh3[c_maxSize]; // channel 3 buffer: Falling Step Ramp Rate

// channel selection mask buffer
byte g_bufferSelMask[c_maxSize * 4];

// channel operation mode mask buffer
byte g_bufferModeMask[c_maxSize * 4];




// Board functions
/**
 * Board setup function (executed once)
 * . digital pins initialization
 * . frequency tuning word buffers initialization
 * . PC - board serial port initialization
 */
void setup(){

  // initialize digital pins
  initDigitalPins();

  // reset DUC
  // (master reset signal and DUC initialization)
  hardResetDUC();

  // initialize frequency tuning word buffers
  initFTWBuffers();

  // initialize serial monitor (max baud rate)
  Serial.begin(115200);

  // initialize PC >>> board serial communication
  if (!g_clientCXN){
    Serial.println("\n\n\n* PC >>> board serial communication established!\n");
    g_clientCXN = true;
  }

  // activate interrupt service routines
  activateISR();

}


/**
 * Board loop function (executed continuously)
 * . listen to USB serial communication
 */
void loop(){ 

  // listen to USB serial port
  readSerialPort();

}




// PC - board communication functions
/**
 * Listen to the board USB serial port
 */
void readSerialPort(){

  if (Serial.available() > 0){

    // get input instruction string
    char inChar = (char)Serial.read();
    g_byteCount++;

    // get communication start string: start timer
    if (g_startCOM){

      // deactivate interrupts
      deactivateISR();

      // serial communication has started
      g_startCOM = false;
      g_starTime = micros();
    }

    // convert incoming bytes to char,
    // add them to string as long as they are not a newline
    if (inChar != '\n') g_inString += inChar;
    else{   
        
      // decode instruction strings
      if (g_inString != "END") decodeInstructionString();
      // end of communication
      else{       

        // print serial communication summary
        printSerialCOM();

        // activate interrupts
        activateISR();

        // a new serial communication will start
        g_startCOM = true;
              
      }

      // delete strings
      g_inString  = "";

    }

  }

}


/**
 * Decode input instruction strings and store data into memory buffers
 */
void decodeInstructionString(){
  
  // decode configuration header
  byte cfgHdr;
  cfgHdr = (byte)g_inString.charAt(0);
  g_inString.remove(0, 1);

  // decode channel selection nibble
  decodeChannelSelectionNibble(cfgHdr);

  // decode operation mode nibble
  decodeChannelModeNibble(cfgHdr);

  // parse incoming data
  parseInstructionString();

  // encode and store parsed data
  storeParsedValues(cfgHdr);

  // increase overall input counter
  g_numIn++;

}


/**
 * Decode channel selection nibble
 * @param[in] cfgHdr configuration char header
 */
void decodeChannelSelectionNibble(byte cfgHdr){

  // programmed channels selection: first nibble
  byte chSelByte = (cfgHdr & c_Mask8Nibble04) | 0b00000110;

  // fill channel selection buffer
  switch (chSelByte){
    case c_chSel0:
      g_bufferSelMask[g_numIn] = c_Ch0;
      break;
    case c_chSel1:
      g_bufferSelMask[g_numIn] = c_Ch1;
      break;
    case c_chSel2:
      g_bufferSelMask[g_numIn] = c_Ch2;
      break;
    case c_chSel3:
      g_bufferSelMask[g_numIn] = c_Ch3;
      break;
    case c_chSel0 | c_chSel1 | c_chSel2 | c_chSel3:
      g_bufferSelMask[g_numIn] = c_ChAll;
      break;
    case c_chSel0 | c_chSel1:
      g_bufferSelMask[g_numIn] = c_Ch0Ch1;
      break;
    case c_chSel0 | c_chSel2:
      g_bufferSelMask[g_numIn] = c_Ch0Ch2;
      break;
    case c_chSel0 | c_chSel3:
      g_bufferSelMask[g_numIn] = c_Ch0Ch3;
      break;
    case c_chSel1 | c_chSel2:
      g_bufferSelMask[g_numIn] = c_Ch1Ch2;
      break;
    case c_chSel1 | c_chSel3:
      g_bufferSelMask[g_numIn] = c_Ch1Ch3;
      break;
    case c_chSel2 | c_chSel3:
      g_bufferSelMask[g_numIn] = c_Ch2Ch3;
      break;
    case c_chSel1 | c_chSel2 | c_chSel3:
      g_bufferSelMask[g_numIn] = c_Ch1Ch2Ch3;
      break;
    case c_chSel0 | c_chSel1 | c_chSel3:
      g_bufferSelMask[g_numIn] = c_Ch0Ch1Ch3;
      break;
    case c_chSel0 | c_chSel1 | c_chSel2:
      g_bufferSelMask[g_numIn] = c_Ch0Ch1Ch2;
      break;
    case c_chSel0 | c_chSel2 | c_chSel3:
      g_bufferSelMask[g_numIn] = c_Ch0Ch2Ch3;
      break;
    default:
      // no DAC channel selected
      g_bufferSelMask[g_numIn] = 0b00000110;
      break;
  }

}


/**
 * Decode channel operation mode nibble
 * (0: Single-Tone mode; 1: Linear Sweep mode)
 * @param[in] cfgHdr configuration char header
 */
void decodeChannelModeNibble(byte cfgHdr){

  // channels operation mode: second nibble
  byte chModByte = ((cfgHdr & c_Mask8Nibble14) << 4) | 0b00000110;

  // fill channel operation mode buffer
  g_bufferModeMask[g_numIn] = chModByte;

}


/**
 * Parse comma-separated instruction string
 * 
 */
void parseInstructionString(){

byte v = 0;
int start_idx = -1;
int sep_idx = g_inString.indexOf(',', start_idx + 1);
while (sep_idx != -1){  
  g_parsedValues[v] = (g_inString.substring(start_idx, sep_idx)).toFloat();
  sep_idx = g_inString.indexOf(',', start_idx + 1);
  v++;  
}

}


/**
 * Encode DUC tuning words and store them into memory buffers.
 * 
 */
void storeParsedValues(byte cfgHdr){

  // declare DUC tuning words
  unsigned int FTW0;
  unsigned int FTW1;
  unsigned int RDW;
  unsigned int FDW;
  byte RSRR;
  byte FSRR;

  // backward loop over channels (from 3 to 0)
  byte offset = 0;
  for (byte i = c_Nch - 1; i >= 1; i--){

    // check channel selection bit in cfgHdr
    if (isBitSet(cfgHdr, i + c_Nch)){

      // check channel mode bit in cfgHdr (0: STM; 1: LSM)
      if (isBitSet(cfgHdr, i)){

        // linear sweep mode: 6 values
        float freqStart = g_parsedValues[offset];
        float freqEnd   = g_parsedValues[offset + 1];

        // check sweeping direction
        // up
        if (freqStart <= freqEnd){
          FTW0 = encodeFTW(freqStart);
          FTW1 = encodeFTW(freqEnd);
          RDW  = encodeFTW(g_parsedValues[offset + 2]);
          FDW  = encodeFTW(g_parsedValues[offset + 3]);
          RSRR = (byte)(g_parsedValues[offset + 4] * c_SyncClk);
          FSRR = (byte)(g_parsedValues[offset + 5] * c_SyncClk);
        }
        // down
        else{
          FTW0 = encodeFTW(freqEnd);
          FTW1 = encodeFTW(freqStart);
          RDW  = encodeFTW(g_parsedValues[offset + 3]);
          FDW  = encodeFTW(g_parsedValues[offset + 2]);
          RSRR = (byte)(g_parsedValues[offset + 5] * c_SyncClk);
          FSRR = (byte)(g_parsedValues[offset + 4] * c_SyncClk);
        }
        offset = offset + 6;

        // fill memory
        storeLinearSweep(i, FTW0, FTW1, RDW, FDW, RSRR, FSRR);

      }
      else{
        
        // single-tone mode: 1 value
        FTW0 = encodeFTW(g_parsedValues[offset]);
        offset++;

        // fill memory
        storeSingleTone(i, FTW0);

      }

    }

  }

}


/**
 * Store single-tone frequency tuning word in the respective channel buffer.
 * 
 */
void storeSingleTone(byte ch, unsigned int FTW){

  switch (ch){
    case c_Ch0:
      g_bufferFTW0Ch0[g_inCh0] = FTW;
      g_inCh0++;
      break;
    case c_Ch1:
      g_bufferFTW0Ch1[g_inCh1] = FTW;
      g_inCh1++;
      break;
    case c_Ch2:
      g_bufferFTW0Ch2[g_inCh2] = FTW;
      g_inCh2++;
      break;
    case c_Ch3:
      g_bufferFTW0Ch3[g_inCh3] = FTW;
      g_inCh3++;
      break;
  }

}


/**
 * Store linear sweep tuning words in the respective channel buffers.
 * 
 */
void storeLinearSweep(byte ch, unsigned int FTW0, unsigned int FTW1,
                      unsigned int RDW, unsigned int FDW, byte RSRR, byte FSRR){

  switch (ch){
    case c_Ch0:
      g_bufferFTW0Ch0[g_inCh0] = FTW0;
      g_bufferFTW1Ch0[g_inCh0] = FTW1;
      g_bufferRDWCh0[g_inCh0]  = RDW;
      g_bufferFDWCh0[g_inCh0]  = FDW;
      g_bufferRSRRCh0[g_inCh0] = RSRR;
      g_bufferFSRRCh0[g_inCh0] = FSRR;
      g_inCh0++;
      break;
    case c_Ch1:
      g_bufferFTW0Ch1[g_inCh1] = FTW0;
      g_bufferFTW1Ch1[g_inCh1] = FTW1;
      g_bufferRDWCh1[g_inCh1]  = RDW;
      g_bufferFDWCh1[g_inCh1]  = FDW;
      g_bufferRSRRCh1[g_inCh1] = RSRR;
      g_bufferFSRRCh1[g_inCh1] = FSRR;
      g_inCh1++;
      break;
    case c_Ch2:
      g_bufferFTW0Ch2[g_inCh2] = FTW0;
      g_bufferFTW1Ch2[g_inCh2] = FTW1;
      g_bufferRDWCh2[g_inCh2]  = RDW;
      g_bufferFDWCh2[g_inCh2]  = FDW;
      g_bufferRSRRCh2[g_inCh2] = RSRR;
      g_bufferFSRRCh2[g_inCh2] = FSRR;
      g_inCh2++;
      break;
    case c_Ch3:
      g_bufferFTW0Ch3[g_inCh3] = FTW0;
      g_bufferFTW1Ch3[g_inCh3] = FTW1;
      g_bufferRDWCh3[g_inCh3]  = RDW;
      g_bufferFDWCh3[g_inCh3]  = FDW;
      g_bufferRSRRCh3[g_inCh3] = RSRR;
      g_bufferFSRRCh3[g_inCh3] = FSRR;
      g_inCh3++;
      break;
  }

}




// DUC initialization functions
/**
 * AD9959 initialization function
 * . quad-SPI initialization
 * . FR1 initialization (REF_CLK multiplier)
 */
void initDUC(){

  // set Function Register 1 (PLL x20, Vco gain HIGH)
  setPLLDivider(); 

  // initialize quad-SPI
  initQuadSPI();
 
}


/**
 * Program Function Register 1: set PLL factor to x20 (SYS_CLK = 20 * REF_CLK)
 */
void setPLLDivider(){

  // Function Register 1 bytes
  byte bufferFR1[c_FR1Size];
  bufferFR1[0] = c_FR1;         // 0b00000001
  bufferFR1[1] = 0b11010000;    // FR1[23]:    VCO gain control    (= 1 if the system clock is higher than 255MHz);
                                // FR1[22:18]: Clock Multiplier    (= 10100, i.e. x20);
                                // FR1[17:16]: Charge pump control (= 00, 75μA: best phase noise characteristics)
  bufferFR1[2] = 0b00000000;    // FR1[15]:    open;
                                // FR1[14:12]: profile pin config;
                                // FR1[11:10]: RU/RD bits;
                                // FR1[9:8]:   modulation level    (=00 is required in linear sweep mode)
  bufferFR1[3] = 0b00100000;    // FR1[5] = 1  OUT SYNC_CLK pin disabled

  // select all DDS channels
  selectDDSChannels(c_ChAll);

  // transfer word to DDS via quad-SPI
  digitalWriteFast(c_ChipSel, LOW);
  quadSPIBufferTransfer(bufferFR1, c_FR1Size);
  digitalWriteFast(c_ChipSel, HIGH);

  // issue an I/O update pulse: FR1 register is set
  ioUpdate();

  // whenever the clock multiplier is enabled,
  // time should be allowed to lock the PLL (typically 1 ms)
  delay(1);

}




// Board initialization functions
/**
 * Initialize digital pins logic state
 */
void initDigitalPins(){ 

  // set master reset pin as output, set it LOW
  pinMode(c_HardReset, OUTPUT);
  digitalWriteFast(c_HardReset, LOW);

  // set power down pin as output, set it LOW
  // (DUC external power-down control is unused)
  pinMode(c_PwnDwn, OUTPUT);
  digitalWriteFast(c_PwnDwn, LOW);

  // set I/O update pin as output, set it LOW
  pinMode(c_IOUpdate, OUTPUT);

  // set chip select pin as output, set it LOW
  pinMode(c_ChipSel, OUTPUT);
  digitalWriteFast(c_ChipSel, LOW);

  // set DUC update interrupt pins as input
  pinMode(c_UpdateInterrupt, INPUT_PULLUP);
  pinMode(c_SoftResetInterrupt, INPUT_PULLUP);
  pinMode(c_HardResetInterrupt, INPUT_PULLUP);

}


/**
 * Initialize Frequency Tuning Word buffers (to 0)
 */
void initFTWBuffers(){

  // zero buffer content
  for (unsigned int i = 0; i < c_maxSize; i++){
    g_bufferFTW0Ch0[i] = 0;
    g_bufferFTW1Ch0[i] = 0;
    g_bufferFTW0Ch1[i] = 0;
    g_bufferFTW1Ch1[i] = 0;
    g_bufferFTW0Ch2[i] = 0;
    g_bufferFTW1Ch2[i] = 0;
    g_bufferFTW0Ch3[i] = 0;
    g_bufferFTW1Ch3[i] = 0;
    g_bufferRDWCh0[i]  = 0;
    g_bufferRDWCh1[i]  = 0;
    g_bufferRDWCh2[i]  = 0;
    g_bufferRDWCh3[i]  = 0; 
    g_bufferFDWCh0[i]  = 0;
    g_bufferFDWCh1[i]  = 0;
    g_bufferFDWCh2[i]  = 0;
    g_bufferFDWCh3[i]  = 0;
    g_bufferRSRRCh0[i] = 0;
    g_bufferRSRRCh1[i] = 0;
    g_bufferRSRRCh2[i] = 0;
    g_bufferRSRRCh3[i] = 0;
    g_bufferFSRRCh0[i] = 0;
    g_bufferFSRRCh1[i] = 0;
    g_bufferFSRRCh2[i] = 0;
    g_bufferFSRRCh3[i] = 0;
  }

}




// Interrupt service routines
/**
 * Board re-initialization: triggered when the c_SoftResetInterrupt pin goes from low to high
 */
void softResetBoard(){
  
  // deactivate interrupts
  deactivateISR();

  // print to serial monitor
  Serial.println("\n* Board ""soft"" reset!\n");
  
  // reset state variables
  g_startCOM  = true;
  g_byteCount = 0;
    
  // reset indices
  g_numIn  = 0;   // input
  g_inCh0  = 0;
  g_inCh1  = 0;
  g_inCh2  = 0;
  g_inCh3  = 0;
  g_numOut = 0;   // output
  g_outCh0 = 0;
  g_outCh1 = 0;
  g_outCh2 = 0;
  g_outCh3 = 0;

  // reset FTW buffers
  initFTWBuffers();

  // activate interrupts
  activateISR();

}


/**
 * Issue a master reset signal (DUC "hard" reset): triggered when the c_HardResetInterrupt pin goes from low to high
 */
void hardResetDUC(){

  // deactivate interrupts
  deactivateISR();

  // print to serial monitor
  Serial.println("\n* DUC ""hard"" reset!\n");

  // issue master reset pulse
  digitalWriteFast(c_HardReset, HIGH);
  delayMicroseconds(c_Tassert);
  digitalWriteFast(c_HardReset, LOW);

  // initialize DUC (PLL and quad-SPI com)
  initDUC();

  // activate interrupts
  activateISR();

}


/**
 * Update DDS channel programming: triggered when the c_UpdateInterrupt pin goes from low to high
 */
void ddsUpdate(){

  // deactivate interrupts
  deactivateISR();

  // code here
  // ...
  // ...

  // activate interrupts
  activateISR();

}


/**
 * Activate interrupt service routines
 */
void activateISR(){
  
  attachInterrupt(digitalPinToInterrupt(c_UpdateInterrupt), ddsUpdate, RISING);
  attachInterrupt(digitalPinToInterrupt(c_HardResetInterrupt), hardResetDUC, RISING);
  attachInterrupt(digitalPinToInterrupt(c_SoftResetInterrupt), softResetBoard, RISING);

}


/**
 * De-activate interrupt service routines
 */
void deactivateISR(){

  detachInterrupt(digitalPinToInterrupt(c_UpdateInterrupt));
  detachInterrupt(digitalPinToInterrupt(c_HardResetInterrupt));
  detachInterrupt(digitalPinToInterrupt(c_SoftResetInterrupt));
  
}




// DDS update functions
/**
 * (Single Tone Mode) Transfer channel frequency tuning word to AD9959 DDS via quad-SPI
 * @param[in] ch programmed channel(s)
 * @param[in] FTW frequency tuning word
 */
void spiTransferChST(byte ch, unsigned int FTW){

  // words buffering
  const byte sizeST = 5;
  byte bufferFTWST[sizeST];
  bufferFTWST[0] = c_CFTW0;                           // access Channel Frequency Tuning Word 0 register 0x04
  bufferFTWST[1] = (byte)((FTW & 0xFF000000) >> 24);  // upper byte
  bufferFTWST[2] = (byte)((FTW & 0x00FF0000) >> 16);  // second-high byte
  bufferFTWST[3] = (byte)((FTW & 0x0000FF00) >> 8);   // second-low byte
  bufferFTWST[4] = (byte)( FTW & 0x000000FF);         // lower byte

  // select DDS channel
  selectDDSChannels(ch);

  // transfer data to DDS (custom quad-SPI)
  digitalWriteFast(c_ChipSel, LOW);
  quadSPIBufferTransfer(bufferFTWST, sizeST);
  digitalWriteFast(c_ChipSel, HIGH);

}


/**
 * (Linear Sweep Mode) Transfer chirp words to AD9959 DDS via quad-SPI
 * @param[in] ch programmed channel(s)
 * @param[in] FTW0 start frequency tuning word
 * @param[in] FTW1 end frequency tuning word
 * @param[in] RDW rising delta frequency word
 * @param[in] FDW falling delta frequency word
 * @param[in] RSRR rising sweep ramp rate
 * @param[in] FSRR falling sweep ramp rate
 */
void spiTransferChLS(byte ch, unsigned int FTW0, unsigned int FTW1,
                     unsigned int RDW, unsigned int FDW, byte RSRR, byte FSRR){
  
  // words buffering
  const byte sizeLS = 23;
  byte bufferFTWLS[sizeLS];

  // lower chirp frequency
  bufferFTWLS[0] = c_CFTW0;                               // access Channel Frequency Tuning Word 0 register 0x04
  bufferFTWLS[1] = (byte)((FTW0 & 0xFF000000) >> 24);     // upper byte
  bufferFTWLS[2] = (byte)((FTW0 & 0x00FF0000) >> 16);     // second-high byte
  bufferFTWLS[3] = (byte)((FTW0 & 0x0000FF00) >> 8);      // second-low byte
  bufferFTWLS[4] = (byte)( FTW0 & 0x000000FF);            // lower byte

  // higher chirp frequency
  bufferFTWLS[5] = c_CFTW1;                               // access Channel Frequency Tuning Word 1 register 0x0A
  bufferFTWLS[6] = (byte)((FTW1 & 0xFF000000) >> 24);     // upper byte
  bufferFTWLS[7] = (byte)((FTW1 & 0x00FF0000) >> 16);     // second-high byte
  bufferFTWLS[8] = (byte)((FTW1 & 0x0000FF00) >> 8);      // second-low byte
  bufferFTWLS[9] = (byte)( FTW1 & 0x000000FF);            // lower byte

  // time step tuning bytes
  bufferFTWLS[10] = c_LSRR;                               // access Linear Sweep Ramp Rate register
  bufferFTWLS[11] = FSRR;
  bufferFTWLS[12] = RSRR;

  // rising frequency delta word
  bufferFTWLS[13] = c_RDW;                            // access RDW register 0x08
  bufferFTWLS[14] = (byte)((RDW & 0xFF000000) >> 24);     // upper byte
  bufferFTWLS[15] = (byte)((RDW & 0x00FF0000) >> 16);     // second-high byte
  bufferFTWLS[16] = (byte)((RDW & 0x0000FF00) >> 8);      // second-low byte
  bufferFTWLS[17] = (byte)( RDW & 0x000000FF);            // lower byte

  // falling frequency delta word
  bufferFTWLS[18] = c_FDW;                            // access FDW register 0x09
  bufferFTWLS[19] = (byte)((FDW & 0xFF000000) >> 24);     // upper byte
  bufferFTWLS[20] = (byte)((FDW & 0x00FF0000) >> 16);     // second-high byte
  bufferFTWLS[21] = (byte)((FDW & 0x0000FF00) >> 8);      // second-low byte
  bufferFTWLS[22] = (byte)( FDW & 0x000000FF);            // lower byte

  // select DDS channel
  selectDDSChannels(ch);

  // transfer data to DDS (custom quad-SPI)
  digitalWriteFast(c_ChipSel, LOW);
  quadSPIBufferTransfer(bufferFTWLS, sizeLS);
  digitalWriteFast(c_ChipSel, HIGH);  

}




// DUC configuration functions
/**
 * Activate the Single-Tone operation mode of the AD9959 DDS
 */
void activateChSTM(byte ch){

  // Channel Function Register bytes
  byte bufferCFR[c_CFRSize];      
  bufferCFR[0] = c_CFR;
  bufferCFR[1] = 0x02;
  bufferCFR[2] = 0x03;
  bufferCFR[3] = 0x00;

  // select DDS channel
  selectDDSChannels(ch);

  // transfer word to DDS via quad-SPI
  digitalWriteFast(c_ChipSel, LOW);
  quadSPIBufferTransfer(bufferCFR, c_CFRSize);
  digitalWriteFast(c_ChipSel, HIGH);

  // issue an I/O update pulse: CFR is set
  ioUpdate();

}


/**
 * Activate the Linear Sweep operation mode of the AD9959 DDS
 */
void activateChLSM(byte ch){

  // Channel Function Register bytes
  byte bufferCFR[c_CFRSize];      
  bufferCFR[0] = c_CFR;
  bufferCFR[1] = 0b10000000;    // CFR[23:22]: Select Frequency Sweep (= 10);
                                // CFR[21:16]: Open 
  bufferCFR[2] = 0b01000011;    // CFR[15]:    Dwell mode             (=0 no-dwell disabled)
                                // CFR[14]:    Linear Sweep Enable    (=1)
                                // CFR[13]:    Load SRR at I/O Update (=0 the linear sweep ramp rate timer is loaded only upon timeout (timer=1) and is not loaded because of an I/O update pulse)
                                // CFR[12:11]: Open
                                // CFR[10]:    0
                                // CFR[9:8]:   DAC full-scale         (=11 full-scale enabled)
  bufferCFR[3] = 0b00000000;    // CFR[7:0]    (DEFAULT VALUE: 0x02)

  // select DDS channel
  selectDDSChannels(ch);

  // transfer word to DDS via quad-SPI
  digitalWriteFast(c_ChipSel, LOW);
  quadSPIBufferTransfer(bufferCFR, c_CFRSize);
  digitalWriteFast(c_ChipSel, HIGH);

  // issue an I/O update pulse: CFR is set
  ioUpdate();

}


/**
 * Select AD9959 DDS channels
 */
void selectDDSChannels(byte ch){

  // selection
  byte bufferChSel[c_SelSize];
  bufferChSel[0] = c_CSR;
  switch (ch){
    case c_Ch0:
      bufferChSel[1] = c_chSel0;
      break;
    case c_Ch1:
      bufferChSel[1] = c_chSel1;
      break;
    case 2:
      bufferChSel[1] = c_chSel2;
      break;
    case 3:
      bufferChSel[1] = c_chSel3;
      break;
    case c_ChAll:
      bufferChSel[1] = c_chSel0 | c_chSel1 | c_chSel2 | c_chSel3;
      break;
    case c_Ch0Ch1:
      bufferChSel[1] = c_chSel0 | c_chSel1;
      break;
    case c_Ch0Ch2:
      bufferChSel[1] = c_chSel0 | c_chSel2;
      break;
    case c_Ch0Ch3:
      bufferChSel[1] = c_chSel0 | c_chSel3;
      break;
    case c_Ch1Ch2:
      bufferChSel[1] = c_chSel1 | c_chSel2;
      break;
    case c_Ch1Ch3:
      bufferChSel[1] = c_chSel1 | c_chSel3;
      break;
    case c_Ch2Ch3:
      bufferChSel[1] = c_chSel2 | c_chSel3;
      break;
    case c_Ch1Ch2Ch3:
      bufferChSel[1] = c_chSel1 | c_chSel2 | c_chSel3;
      break;
    case c_Ch0Ch1Ch3:
      bufferChSel[1] = c_chSel0 | c_chSel1 | c_chSel3;
      break;
    case c_Ch0Ch1Ch2:
      bufferChSel[1] = c_chSel0 | c_chSel1 | c_chSel2;
      break;
    case c_Ch0Ch2Ch3:
      bufferChSel[1] = c_chSel0 | c_chSel2 | c_chSel3;
      break;
    default:
      bufferChSel[1] = 0b00000110;
      Serial.println("\n* No DDS channel selected: corrupted channel programming routine!\n");
      break;
  }

  // transfer buffer content to DUC (custom quad-SPI)
  digitalWriteFast(c_ChipSel, LOW);
  quadSPIBufferTransfer(bufferChSel, c_SelSize);
  digitalWriteFast(c_ChipSel, HIGH);

}




// Custom quad-SPI functions
/**
 * Initialize quad-SPI
 */
void initQuadSPI(){ 

  // select DUC
  digitalWriteFast(c_ChipSel, HIGH);

  // set quad-SPI pins
  for (byte i = 0; i < 5; i++){
    pinMode(c_quadSPIPins[i], OUTPUT);
    digitalWriteFast(c_quadSPIPins[i], LOW);
  }

  // transfer 00000000 00000110 via "custom" single-bit SPI
  byte bufferInitSPI[2];
  bufferInitSPI[0] = c_CSR;
  bufferInitSPI[1] = 0b11110110;
  
  // select slave device
  digitalWriteFast(c_ChipSel, LOW);

  // send data  
  singleSPIBufferTransfer(bufferInitSPI, 2);

  // de-select DUC
  digitalWriteFast(c_ChipSel, HIGH);

  // issue an I/O update for changing the serial mode bits
  ioUpdate();

}


/**
 * Custom SPI 8-bit data transfer (with serial clock divider)
 * 
 * @param[in] data input byte to be transferred
 */
void singleSPIByteTransfer(byte data){
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble01) << 9);
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble01) << 9)  | c_SclkHigh; 
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble11) << 10);
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble11) << 10) | c_SclkHigh; 
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble21) << 11);
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble21) << 11) | c_SclkHigh;
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble31) << 12);
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble31) << 12) | c_SclkHigh;
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble41) << 13);
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble41) << 13) | c_SclkHigh; 
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble51) << 14);
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble51) << 14) | c_SclkHigh; 
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble61) << 15);
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble61) << 15) | c_SclkHigh;
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble71) << 16);
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble71) << 16) | c_SclkHigh;
}


/**
 * Custom quad-SPI 8-bit data transfer (with serial clock divider)
 * 
 * @param[in] data input byte to be transferred
 */
void quadSPIByteTransfer(byte data){
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble04) << 12);                        
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble04) << 12) | c_SclkHigh;
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble14) << 16);
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble14) << 16) | c_SclkHigh;
}


/**
 * Custom SPI buffer transfer (with serial clock divider)
 * 
 * @param[in] buffer input buffer to be transferred
 * @param[in] buffer_size buffer size in bytes
 */
void singleSPIBufferTransfer(byte buffer[], unsigned int buffer_size){
  // loop over buffer byte elements
  for (unsigned int b = 0; b < buffer_size; b++) singleSPIByteTransfer(buffer[b]);
  GPIO6_DR = c_SafeClearGPIO6Bit5(GPIO6_DR);
}


/**
 * Custom quad-SPI buffer transfer (with serial clock divider)
 * 
 * @param[in] buffer input buffer to be transferred
 * @param[in] buffer_size buffer size in bytes
 */
void quadSPIBufferTransfer(byte buffer[], unsigned int buffer_size){
  // loop over buffer byte elements
  for (unsigned int b = 0; b < buffer_size; b++) quadSPIByteTransfer(buffer[b]);
  GPIO6_DR = 0;
}




// Utilities
/**
 * Encode DDS Frequency Tuning Word
 * 
 * @param[in] f_AOD AOD frequency in MHz
 * 
 * @returns 32-bit Frequency Tuning Word
 */
unsigned int encodeFTW(float f_AOD){

  unsigned int FTW;
  FTW = (unsigned int)round(pow(2, c_DDSbits) * f_AOD / c_SysClk);

  return FTW;

}


/**
 * Decode DDS Frequency Tuning Word
 * 
 * @param[in] FTW 32-bit Frequency Tuning Word
 * 
 * @returns AOD frequency in MHz
 */
float decodeFrequency(unsigned int FTW){

  float f_AOD;
  f_AOD = (float)(FTW * c_SysClk / pow(2, c_DDSbits));

  return f_AOD;

}


/**
 * Issue an I/O update pulse via MCU board
 * 
 * note: the I/O update pulse is oversampled by the SYNC_CLK,
 * therefore its width must be greater than one SYNC_CLK period, i.e. 8 ns
 */
void ioUpdate(){

  digitalWriteFast(c_IOUpdate, HIGH);
  delayMicroseconds(c_Tassert);
  digitalWriteFast(c_IOUpdate, LOW);
        
}


/**
 * Check if k-th bit in byte n is set.
 * 
 * @param[in] b input byte to check
 * @param[in] p checked position
 * 
 * @returns true if p-th bit is set, false otherwise
 */
bool isBitSet(byte b, byte p){

  bool isSet = (b >> (p - 1)) & 1;

  return isSet;

}




// Print functions
/**
 * Print instruction string communication summary
 */
void printSerialCOM(){

  // get current time
  g_currenTime = micros();

  // info  
  float elapsed = (g_currenTime - g_starTime) * 0.001;
  Serial.print("* PC >>> board serial communication complete!\n");
  Serial.print("  Received: ");
  Serial.print(g_byteCount);
  Serial.print(" B, ");
  Serial.print(elapsed);
  Serial.print(" ms");
  float rate = (float) g_byteCount / elapsed;
  Serial.print(", rate = ");
  Serial.print(rate);
  Serial.println(" kB/s\n");

}


/**
 * Print complete byte to serial monitor
 */
void printByte(byte B){
  for(int i = 7; i >= 0; i--){
    Serial.print(bitRead(B,i));
    if(i % 4 == 0) Serial.print(" ");
  } 
  Serial.println();
}


/**
 * Print complete 32-bit uint word to serial monitor
 */
void printUint(unsigned int UI){
  for(int i = 31; i >= 0; i--){
    Serial.print(bitRead(UI, i));
    if(i % 4 == 0) Serial.print(" ");
  } 
  Serial.println();
}
