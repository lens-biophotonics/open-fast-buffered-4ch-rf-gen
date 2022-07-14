 /**
 * > Open Source Laboratory Instrument Project:
 *   Control Software for AD9959 DDS
 * 
 * > Author
 *   Michele Sorelli (2022)
 * 
 */
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <CircularBuffer.h>


/** 
 * Teensy 4.1 Custom Serial I/O : GPIO6_DR pin mapping (from msb to lsb) 
 * 
 * 27 26 39 38   21 20 23 22   16 17 41 40   15 14 18 19   X X 25 24   X X X X   X X X X   0 1 X X
                                       SCLK  |MOSI PINS|
 *                                                                        
 */


// Global variables

// debugging mode
bool g_debug = false;

// input timing mode
bool g_timeit = false;
unsigned long g_strTime = 0;
unsigned int g_strIn = 1;

// PC serial communication
bool g_clientCXN = false;       // USB serial connection established
bool g_endCOM    = true;        // start serial communication flag
unsigned long g_currenTime;
unsigned long g_starTime;
volatile unsigned long g_byteCount = 0;


// digital pins (to AD9959 DDS)            Connection         Function
const byte c_PwnDwn    = 2;             // output to DUC   /  DUC power-down control
const byte c_HardReset = 3;             // output to DUC   /  DUC master reset
const byte c_IOUpdate  = 5;             // output to DUC   /  data I/O update 
const byte c_ChipSel   = 10;            // output to DUC   /  DUC chip select
const byte c_HardResetInterrupt = 31;   // input           /  trigger DUC hard reset routine
const byte c_SoftResetInterrupt = 33;   // input           /  trigger MCU soft reset routine
const byte c_singleSPInterrupt  = 35;   // input           /  trigger single-SPI activation routines
const byte c_quadSPInterrupt    = 36;   // input           /  trigger quad-SPI activation routines
const byte c_UpdateInterrupt    = 37;   // input           /  trigger DUC update routine

// quad-SPI (data pins ordered from msb to lsb, + SCLK)
const byte c_QuadSPIPins[5] = {15, 14, 18, 19, 40};
const byte c_QuadSclk = 40;
const byte c_QuadSclkDiv = 2;
bool g_QuadSPIActive = false;

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
const unsigned int c_Nch = 4;
const byte c_chSel0 = 0b00010000; // selection bytes
const byte c_chSel1 = 0b00100000;
const byte c_chSel2 = 0b01000000;
const byte c_chSel3 = 0b10000000;
const unsigned int c_Ch0   = 0;          // IDs
const unsigned int c_Ch1   = 1;
const unsigned int c_Ch2   = 2;
const unsigned int c_Ch3   = 3;
const unsigned int c_ChAll = 4;
byte g_sweepCh = 0b00000000;


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
const byte c_CFRSize = 6;


// input to MCU board
const unsigned int c_maxSize = 4000;  // max buffer size
using byteBuffer   = CircularBuffer<byte, c_maxSize>;
using uintBuffer   = CircularBuffer<unsigned int, c_maxSize>;
using stringBuffer = CircularBuffer<String, c_maxSize>;
String g_inString  = "";              // input instruction string
stringBuffer g_inStringBuffer;        // input string FIFO buffer
volatile unsigned int g_numIn = 0;    // overall input setting counter
String g_outString = "";              // decoded instruction string
float g_parsedValues[24];             // array of parsed values


// output from MCU board
volatile unsigned int g_numOut = 0;   // overall output setting counter
uintBuffer g_bufferFTW0Ch0;           // channel 0 buffer: EITHER frequency tuning word (Single Tone Mode) OR chirp start frequency tuning word (Linear Sweep Mode)
uintBuffer g_bufferFTW1Ch0;           // channel 0 buffer: chirp end frequency tuning word (Linear Sweep Mode only)             
uintBuffer g_bufferFTW0Ch1;           // channel 1 buffer: EITHER frequency tuning word (Single Tone Mode) OR chirp start frequency tuning word (Linear Sweep Mode)      
uintBuffer g_bufferFTW1Ch1;           // channel 1 buffer: chirp end frequency tuning word (Linear Sweep Mode only)      
uintBuffer g_bufferFTW0Ch2;           // channel 2 buffer: EITHER frequency tuning word (Single Tone Mode) OR chirp start frequency tuning word (Linear Sweep Mode)
uintBuffer g_bufferFTW1Ch2;           // channel 2 buffer: chirp end frequency tuning word (Linear Sweep Mode only)
uintBuffer g_bufferFTW0Ch3;           // channel 3 buffer: EITHER frequency tuning word (Single Tone Mode) OR chirp start frequency tuning word (Linear Sweep Mode)
uintBuffer g_bufferFTW1Ch3;           // channel 3 buffer: chirp end frequency tuning word (Linear Sweep Mode only)

// linear frequency sweep time/frequency step tuning words
uintBuffer g_bufferRDWCh0;            // channel 0 buffer: Rising  (frequency) Delta Word buffer
uintBuffer g_bufferRDWCh1;            // channel 1 buffer: Rising  (frequency) Delta Word buffer
uintBuffer g_bufferRDWCh2;            // channel 2 buffer: Rising  (frequency) Delta Word buffer
uintBuffer g_bufferRDWCh3;            // channel 3 buffer: Rising  (frequency) Delta Word buffer
uintBuffer g_bufferFDWCh0;            // channel 0 buffer: Falling (frequency) Delta Word buffer
uintBuffer g_bufferFDWCh1;            // channel 1 buffer: Falling (frequency) Delta Word buffer
uintBuffer g_bufferFDWCh2;            // channel 2 buffer: Falling (frequency) Delta Word buffer
uintBuffer g_bufferFDWCh3;            // channel 3 buffer: Falling (frequency) Delta Word buffer
byteBuffer g_bufferRSRRCh0;           // channel 0 buffer: Rising Step Ramp Rate
byteBuffer g_bufferRSRRCh1;           // channel 1 buffer: Rising Step Ramp Rate
byteBuffer g_bufferRSRRCh2;           // channel 2 buffer: Rising Step Ramp Rate
byteBuffer g_bufferRSRRCh3;           // channel 3 buffer: Rising Step Ramp Rate
byteBuffer g_bufferFSRRCh0;           // channel 0 buffer: Falling Step Ramp Rate
byteBuffer g_bufferFSRRCh1;           // channel 1 buffer: Falling Step Ramp Rate
byteBuffer g_bufferFSRRCh2;           // channel 2 buffer: Falling Step Ramp Rate
byteBuffer g_bufferFSRRCh3;           // channel 3 buffer: Falling Step Ramp Rate

// channel operation mode mask buffer
CircularBuffer<byte, c_maxSize * c_Nch> g_bufferSingleToneMode;
CircularBuffer<byte, c_maxSize * c_Nch> g_bufferLinearSweepMode;


// message strings
char c_pcUSBMsg[] PROGMEM = "\n * PC >>> MCU serial communication established!";
char c_softRMsg[] PROGMEM = "\n * MCU ""soft"" reset!";
char c_hardRMsg[] PROGMEM = "\n * MCU ""hard"" reset!";
char c_quSPIMsg[] PROGMEM = "\n * MCU >>> DUC quad-SPI communication activated!";
char c_sgSPIMsg[] PROGMEM = "\n * MCU >>> DUC single-SPI communication activated!";
char c_chErrMsg[] PROGMEM = "\n * No DDS channel selected: corrupted channel programming routine!";
char c_inMemMsg[] PROGMEM = "\n * Max memory reached! Ignoring input instruction strings...";
char c_inStrMsg[] PROGMEM = "\n * String buffer full! Ignoring input instruction strings...";
char c_STModMsg[] PROGMEM = "     Single-Tone mode";
char c_LSModMsg[] PROGMEM = "     Linear Sweep mode";


// Board functions
/**
 * MCU board setup function (executed once)
 * . digital pins initialization
 * . frequency tuning word buffers initialization
 * . PC >>> board serial port initialization
 */
void setup(){

  // disable debug serial prints for input strings timing
  if (g_timeit) g_debug = false;

  // initialize digital pins
  initDigitalPins();

  // reset DUC
  // (master reset signal and DUC initialization)
  hardResetDUC();

  // initialize serial monitor (max baud rate)
  Serial.begin(115200);

  // initialize PC >>> board serial communication
  if (!g_clientCXN){
    Serial.println(c_pcUSBMsg);
    g_clientCXN = true;
  }

  // activate interrupt service routines
  activateISR();

}


/**
 * MCU board loop function (executed continuously)
 * . listen to USB serial communication
 */
void loop(){ 

  // listen to USB serial port
  readSerialPort();

  // decode input data strings
  decodeDataString();

}




// PC >>> board communication functions
/**
 * Listen to the board USB serial port.
 */
void readSerialPort(){

  if (Serial.available()){

    // get incoming char(s)
    char inChar = (char)Serial.read();
    g_byteCount++;

    // start communication timer
    if (g_endCOM){

      // input timing mode
      if (g_timeit) delay(1000);

      // deactivate interrupts
      deactivateISR();

      // serial communication has started
      g_endCOM = false;
      g_starTime = micros();
      
      // input timing mode
      if (g_timeit) g_strTime = g_starTime;
    }

    // add char to string as long as it is not a newline
    if (inChar != '\n') g_inString += inChar;
    else{

      // push incoming string into FIFO string buffer
      if (!g_inString.equals("END")){
        if (!g_inStringBuffer.isFull()){
          g_inStringBuffer.push(g_inString);

          // input timing mode
          if (g_timeit) printReadTime();

        }
        else Serial.println(c_inStrMsg);
      }

      // end of communication
      else{

        // print serial communication summary
        printSerialCOM();

        // activate interrupts
        activateISR();

        // a new serial communication will start
        g_endCOM = true;

      }

      // delete string
      g_inString = "";

    }

  }

}


/**
 * Decode input instruction strings and store data into memory buffers.
 */
void decodeDataString(){
  
  // if not communicating
  if (g_endCOM == true){

    // deactivate interrupts
    deactivateISR();

    // strings available for decoding
    if (!g_inStringBuffer.isEmpty()){
      
      // wait for serial prints when debugging
      if (g_debug) delay(1000);

      // input timing mode
      if (g_timeit) g_strTime = micros();

      // "pop" data string from input buffer
      g_outString = g_inStringBuffer.shift();
  
      // get configuration header
      byte cfgHdr = decodeChannelHeader();

      // parse incoming data
      parseDataString();

      // encode and store parsed data
      storeParsedValues(cfgHdr);

      // increase overall input counter
      g_numIn++;

      // input timing mode
      if (g_timeit) printDecodeTime();

    }

    // activate interrupts
    activateISR();

  }

}


/**
 * Decode channel activation byte header.
 * MSN: updated channels (0: do not update; 1: update);
 * LSN: channels operation mode (0: Single-Tone mode; 1: Linear Sweep mode).
 * @returns DUC channels configuration header
 */
byte decodeChannelHeader(){

  // get configuration header
  byte sep_idx = g_outString.indexOf(',', 0);
  byte cfgHdr = (byte)(g_outString.substring(0, sep_idx)).toInt();

  // remove it from input data string
  g_outString.remove(0, sep_idx + 1);

  // channels operation mode byte
  byte chLSModByte = ( ((cfgHdr & c_Mask8Nibble14) << 4) & (cfgHdr & c_Mask8Nibble04));
  byte chSTModByte = (~((cfgHdr & c_Mask8Nibble14) << 4) & (cfgHdr & c_Mask8Nibble04));

  // push channel operation mode into FIFO buffers
  g_bufferSingleToneMode.push(chSTModByte);
  g_bufferLinearSweepMode.push(chLSModByte);

  return cfgHdr;

}


/**
 * Parse comma-separated data string.
 * 
 */
void parseDataString(){

byte v = 0;
int start_idx = 0;
int sep_idx = g_outString.indexOf(',', start_idx + 1);
while (sep_idx != -1){  
  g_parsedValues[v] = (g_outString.substring(start_idx, sep_idx)).toFloat();
  start_idx = sep_idx + 1;
  sep_idx = g_outString.indexOf(',', start_idx);
  v++;  
}

}


/**
 * Encode DUC tuning words and store them into memory buffers.
 * @param[in] cfgHdr DUC channels configuration header
 */
void storeParsedValues(byte cfgHdr){

  // DUC tuning words
  unsigned int FTW0;
  unsigned int FTW1;
  unsigned int RDW;
  unsigned int FDW;
  byte RSRR;
  byte FSRR;

  // debug mode
  if (g_debug){
    char str[20];
    sprintf(str, "\n * %d >>> MCU\n", g_numIn);
    Serial.println(str);
  }

  // backward loop over channels (from 3 to 0)
  unsigned int offset = 0;
  for (unsigned int ch = 0; ch < c_Nch; ch++){
    
    // check channel selection bit in cfgHdr
    if (bitRead(cfgHdr, ch + c_Nch)){
     
      // debug mode
      if (g_debug){
        char strCh[20];
        sprintf(strCh, " . Channel %d: ", ch);
        Serial.print(strCh);
      } 

      // check channel mode bit in cfgHdr (0: STM; 1: LSM)
      if (bitRead(cfgHdr, ch)){

        // check sweeping direction
        // up
        if (g_parsedValues[offset] <= g_parsedValues[offset + 1]){
          FTW0 = encodeFTW(g_parsedValues[offset]);
          FTW1 = encodeFTW(g_parsedValues[offset + 1]);
          RDW  = encodeFTW(g_parsedValues[offset + 2]);
          FDW  = encodeFTW(g_parsedValues[offset + 3]);
          RSRR = (byte)(g_parsedValues[offset + 4] * c_SyncClk);
          FSRR = (byte)(g_parsedValues[offset + 5] * c_SyncClk);
        }
        // down
        else{
          FTW0 = encodeFTW(g_parsedValues[offset + 1]);
          FTW1 = encodeFTW(g_parsedValues[offset]);
          RDW  = encodeFTW(g_parsedValues[offset + 3]);
          FDW  = encodeFTW(g_parsedValues[offset + 2]);
          RSRR = (byte)(g_parsedValues[offset + 5] * c_SyncClk);
          FSRR = (byte)(g_parsedValues[offset + 4] * c_SyncClk);
        }
        offset = offset + 6;

        // fill memory
        storeLinearSweep(ch, FTW0, FTW1, RDW, FDW, RSRR, FSRR);

        // debug mode
        if (g_debug){
          Serial.println(c_LSModMsg);
          printLinearSweep(FTW0, FTW1, RDW, FDW, RSRR, FSRR);
        } 

      }
      else{        
        
        // single-tone mode: 1 value
        FTW0 = encodeFTW(g_parsedValues[offset]);
        offset++;

        // fill memory
        storeSingleTone(ch, FTW0);

        // debug mode
        if (g_debug){
          Serial.println(c_STModMsg);
          printSingleTone(FTW0);
        }
        
      }

    }

  }

}


/**
 * Store single-tone frequency tuning word in the respective channel buffer.
 * @param[in] ch  DUC channel
 * @param[in] FTW single-tone frequency tuning word
 */
void storeSingleTone(unsigned int ch, unsigned int FTW){

  switch (ch){
    case c_Ch0:
      g_bufferFTW0Ch0.push(FTW);
      break;
    case c_Ch1:
      g_bufferFTW0Ch1.push(FTW);
      break;
    case c_Ch2:
      g_bufferFTW0Ch2.push(FTW);
      break;
    case c_Ch3:
      g_bufferFTW0Ch3.push(FTW);
      break;
  }

}


/**
 * Store linear sweep tuning words in the respective channel buffers.
 * @param[in] ch   DUC channel
 * @param[in] FTW0 chirp start frequency tuning word
 * @param[in] FTW1 chirp end frequency tuning word
 * @param[in] RDW  rising frequency delta word
 * @param[in] FDW  falling frequency delta word
 * @param[in] RSRR rising step ramp rate
 * @param[in] FSRR falling step ramp rate
 * 
 */
void storeLinearSweep(unsigned int ch, unsigned int FTW0, unsigned int FTW1,
                      unsigned int RDW, unsigned int FDW, byte RSRR, byte FSRR){

  switch (ch){
    case c_Ch0:
      g_bufferFTW0Ch0.push(FTW0);
      g_bufferFTW1Ch0.push(FTW1);
      g_bufferRDWCh0.push(RDW);
      g_bufferFDWCh0.push(FDW);
      g_bufferRSRRCh0.push(RSRR);
      g_bufferFSRRCh0.push(FSRR);
      break;
    case c_Ch1:
      g_bufferFTW0Ch1.push(FTW0);
      g_bufferFTW1Ch1.push(FTW1);
      g_bufferRDWCh1.push(RDW);
      g_bufferFDWCh1.push(FDW);
      g_bufferRSRRCh1.push(RSRR);
      g_bufferFSRRCh1.push(FSRR);
      break;
    case c_Ch2:
      g_bufferFTW0Ch2.push(FTW0);
      g_bufferFTW1Ch2.push(FTW1);
      g_bufferRDWCh2.push(RDW);
      g_bufferFDWCh2.push(FDW);
      g_bufferRSRRCh2.push(RSRR);
      g_bufferFSRRCh2.push(FSRR);
      break;
    case c_Ch3:
      g_bufferFTW0Ch3.push(FTW0);
      g_bufferFTW1Ch3.push(FTW1);
      g_bufferRDWCh3.push(RDW);
      g_bufferFDWCh3.push(FDW);
      g_bufferRSRRCh3.push(RSRR);
      g_bufferFSRRCh3.push(FSRR);
      break;
  }

}




// DUC initialization functions
/**
 * AD9959 initialization function
 * . PLL initialization (FR1 register)
 * . quad-SPI initialization
 */
void initDUC(){

  // set Function Register 1 (PLL x20, Vco gain HIGH)
  setPLLDivider(); 

  // initialize quad-SPI
  initSingleSPI();
 
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
  if (g_QuadSPIActive) quadSPIBufferTransfer(bufferFR1, c_FR1Size);
  else singleSPIBufferTransfer(bufferFR1, c_FR1Size);
  digitalWriteFast(c_ChipSel, HIGH);

  // issue an I/O update pulse: FR1 register is set
  ioUpdate();

  // whenever the clock multiplier is enabled,
  // time should be allowed to lock the PLL (typically 1 ms)
  delay(1);

}




// Board initialization functions
/**
 * Initialize digital pins logic state.
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

  // set SPI communication pins
  for (byte i = 0; i < 5; i++){
    pinMode(c_QuadSPIPins[i], OUTPUT);
    digitalWriteFast(c_QuadSPIPins[i], LOW);
  }

  // set DUC update interrupt pins as input
  pinMode(c_UpdateInterrupt, INPUT_PULLUP);
  pinMode(c_SoftResetInterrupt, INPUT_PULLUP);
  pinMode(c_HardResetInterrupt, INPUT_PULLUP);
  pinMode(c_singleSPInterrupt, INPUT_PULLUP);
  pinMode(c_quadSPInterrupt, INPUT_PULLUP);

}


/**
 * Initialize DUC tuning word buffers (to 0).
 */
void resetFTWBuffers(){

  // reset buffers to their initial state
  g_bufferFTW0Ch0.clear();
  g_bufferFTW1Ch0.clear();
  g_bufferFTW0Ch1.clear();
  g_bufferFTW1Ch1.clear();
  g_bufferFTW0Ch2.clear();
  g_bufferFTW1Ch2.clear();
  g_bufferFTW0Ch3.clear();
  g_bufferFTW1Ch3.clear();
  g_bufferRDWCh0.clear();
  g_bufferRDWCh1.clear();
  g_bufferRDWCh2.clear();
  g_bufferRDWCh3.clear();
  g_bufferFDWCh0.clear();
  g_bufferFDWCh1.clear();
  g_bufferFDWCh2.clear();
  g_bufferFDWCh3.clear();
  g_bufferRSRRCh0.clear();
  g_bufferRSRRCh1.clear();
  g_bufferRSRRCh2.clear();
  g_bufferRSRRCh3.clear();
  g_bufferFSRRCh0.clear();
  g_bufferFSRRCh1.clear();
  g_bufferFSRRCh2.clear();
  g_bufferFSRRCh3.clear();

}




// Interrupt service routines
/**
 * MCU board re-initialization ("soft" reset): triggered when the c_SoftResetInterrupt pin goes from low to high.
 */
void softResetBoard(){
  
  // deactivate interrupts
  deactivateISR();

  // print to serial monitor
  Serial.println(c_softRMsg);
  
  // reset state variables
  g_endCOM  = true;
  g_byteCount = 0;
  g_numIn  = 0;
  g_numOut = 0;

  // reset FTW buffers
  resetFTWBuffers();

  // activate interrupts
  activateISR();

}


/**
 * DUC reset via master reset signal ("hard" reset): triggered when the c_HardResetInterrupt pin goes from low to high.
 */
void hardResetDUC(){

  // deactivate interrupts
  deactivateISR();

  // print to serial monitor
  Serial.println(c_hardRMsg);

  // issue master reset pulse
  digitalWriteFast(c_HardReset, HIGH);
  delayMicroseconds(c_Tassert);
  digitalWriteFast(c_HardReset, LOW);

  // quad-SPI is not active
  g_QuadSPIActive = false;

  // all channels in single-tone mode
  g_sweepCh = 0b00000000;

  // set Function Register 1 (PLL x20, Vco gain HIGH)
  setPLLDivider();

  // activate interrupts
  activateISR();

}


/**
 * Update DUC channels programming: triggered when the c_UpdateInterrupt pin goes from low to high.
 */
void updateDUC(){

  // deactivate interrupts
  deactivateISR();

  // debug mode
  if (g_debug){
    char str[20];
    sprintf(str, "\n * %d >>> DUC\n", g_numOut);
    Serial.println(str);
  }

  // activate channel operation modes (updated channels only)
  byte stMode = g_bufferSingleToneMode.shift();
  byte lsMode = g_bufferLinearSweepMode.shift();
  byte stMask = (~g_sweepCh ^ stMode) & stMode;
  byte lsMask = (g_sweepCh ^ lsMode) & lsMode;
  if (stMask) activateChSTM(stMask);
  if (lsMask) activateChLSM(lsMask);

  // transfer new configurations to DUC channels
  updateCh0(stMode, lsMode);
  updateCh1(stMode, lsMode);
  updateCh2(stMode, lsMode);
  updateCh3(stMode, lsMode);

  // update linear sweep channels
  if (stMask || lsMask) g_sweepCh = (g_sweepCh ^ stMode) | lsMode;

  // increase overall output counter
  g_numOut++;

  // activate interrupts
  activateISR();

}


/**
 * Initialize quad-SPI communication.
 */
void initQuadSPI(){ 

  // only if quad-SPI is active
  if (g_QuadSPIActive == false){

    // deactivate interrupts
    deactivateISR();

    // print to serial monitor
    Serial.println(c_quSPIMsg);

    // transfer 00000000 00000110 via "custom" single-bit SPI
    byte bufferInitSPI[2];
    bufferInitSPI[0] = c_CSR;
    bufferInitSPI[1] = 0b11110110;

    // select slave device
    digitalWriteFast(c_ChipSel, LOW);

    // send data via single-SPI
    singleSPIBufferTransfer(bufferInitSPI, 2);

    // de-select DUC
    digitalWriteFast(c_ChipSel, HIGH);

    // issue an I/O update for changing the serial mode bits
    ioUpdate();

    // quad-SPI is active
    g_QuadSPIActive = true;

    // activate interrupts
    activateISR();

  }

}


/**
 * Initialize single-SPI communication.
 */
void initSingleSPI(){ 

  // only if quad-SPI is active
  if (g_QuadSPIActive){

    // deactivate interrupts
    deactivateISR();

    // print to serial monitor
    Serial.println(c_sgSPIMsg);

    // transfer 00000000 11110000 via "custom" quad-bit SPI
    byte bufferInitSPI[2];
    bufferInitSPI[0] = c_CSR;
    bufferInitSPI[1] = 0b11110000;
    
    // select slave device
    digitalWriteFast(c_ChipSel, LOW);

    // send data via quad-SPI
    quadSPIBufferTransfer(bufferInitSPI, 2);

    // de-select DUC
    digitalWriteFast(c_ChipSel, HIGH);

    // issue an I/O update for changing the serial mode bits
    ioUpdate();

    // quad-SPI is not active
    g_QuadSPIActive = false;

    // activate interrupts
    activateISR();

  }

}


/**
 * Activate external interrupt service routines.
 */
void activateISR(){
  
  attachInterrupt(digitalPinToInterrupt(c_UpdateInterrupt), updateDUC, RISING);
  attachInterrupt(digitalPinToInterrupt(c_HardResetInterrupt), hardResetDUC, RISING);
  attachInterrupt(digitalPinToInterrupt(c_SoftResetInterrupt), softResetBoard, RISING);  
  attachInterrupt(digitalPinToInterrupt(c_singleSPInterrupt), initSingleSPI, RISING);
  attachInterrupt(digitalPinToInterrupt(c_quadSPInterrupt), initQuadSPI, RISING);

}


/**
 * De-activate external interrupt service routines.
 */
void deactivateISR(){

  detachInterrupt(digitalPinToInterrupt(c_UpdateInterrupt));
  detachInterrupt(digitalPinToInterrupt(c_HardResetInterrupt));
  detachInterrupt(digitalPinToInterrupt(c_SoftResetInterrupt));
  detachInterrupt(digitalPinToInterrupt(c_singleSPInterrupt));
  detachInterrupt(digitalPinToInterrupt(c_quadSPInterrupt));

}




// DDS update functions
/**
 * DUC channel 0 update function.
 */
void updateCh0(byte singleToneByte, byte linearSweepByte){

  if (bitRead(singleToneByte, 4) == 1){

    unsigned int FTW0 = g_bufferFTW0Ch0.shift();
    spiTransferChST(0, FTW0);

    if (g_debug){
      Serial.print(F(" . Channel 0: "));
      Serial.println(c_STModMsg);
      printSingleTone(FTW0);
    }

  }
  else if(bitRead(linearSweepByte, 4) == 1){

    unsigned int FTW0 = g_bufferFTW0Ch0.shift();
    unsigned int FTW1 = g_bufferFTW1Ch0.shift();
    unsigned int RDW  = g_bufferRDWCh0.shift();
    unsigned int FDW  = g_bufferFDWCh0.shift();
    byte RSRR = g_bufferRSRRCh0.shift();
    byte FSRR = g_bufferFSRRCh0.shift();
    spiTransferChLS(0, FTW0, FTW1, RDW, FDW, RSRR, FSRR);

    if (g_debug){
      Serial.print(F(" . Channel 0: "));
      Serial.println(c_LSModMsg);
      printLinearSweep(FTW0, FTW1, RDW, FDW, RSRR, FSRR);
    }

  }

}


/**
 * DUC channel 1 update function.
 */
void updateCh1(byte singleToneByte, byte linearSweepByte){

  if (bitRead(singleToneByte, 5) == 1){

    unsigned int FTW0 = g_bufferFTW0Ch1.shift();
    spiTransferChST(1, FTW0);

    if (g_debug){
      Serial.print(F(" . Channel 1: "));
      Serial.println(c_STModMsg);
      printSingleTone(FTW0);
    }

  }
  else if(bitRead(linearSweepByte, 5) == 1){

    unsigned int FTW0 = g_bufferFTW0Ch1.shift();
    unsigned int FTW1 = g_bufferFTW1Ch1.shift();
    unsigned int RDW  = g_bufferRDWCh1.shift();
    unsigned int FDW  = g_bufferFDWCh1.shift();
    byte RSRR = g_bufferRSRRCh1.shift();
    byte FSRR = g_bufferFSRRCh1.shift();
    spiTransferChLS(1, FTW0, FTW1, RDW, FDW, RSRR, FSRR);

    if (g_debug){
      Serial.print(F(" . Channel 1: "));
      Serial.println(c_LSModMsg);
      printLinearSweep(FTW0, FTW1, RDW, FDW, RSRR, FSRR);
    }

  }

}


/**
 * DUC channel 2 update function.
 */
void updateCh2(byte singleToneByte, byte linearSweepByte){

  if (bitRead(singleToneByte, 6) == 1){

    unsigned int FTW0 = g_bufferFTW0Ch2.shift();
    spiTransferChST(2, FTW0);

    if (g_debug){
      Serial.print(F(" . Channel 2: "));
      Serial.println(c_STModMsg);
      printSingleTone(FTW0);
    }

  }
  else if(bitRead(linearSweepByte, 6) == 1){

    unsigned int FTW0 = g_bufferFTW0Ch2.shift();
    unsigned int FTW1 = g_bufferFTW1Ch2.shift();
    unsigned int RDW  = g_bufferRDWCh2.shift();
    unsigned int FDW  = g_bufferFDWCh2.shift();
    byte RSRR = g_bufferRSRRCh2.shift();
    byte FSRR = g_bufferFSRRCh2.shift();
    spiTransferChLS(2, FTW0, FTW1, RDW, FDW, RSRR, FSRR);

    if (g_debug){
      Serial.print(F(" . Channel 2: "));
      Serial.println(c_LSModMsg);
      printLinearSweep(FTW0, FTW1, RDW, FDW, RSRR, FSRR);
    }

  }

}


/**
 * DUC channel 3 update function.
 */
void updateCh3(byte singleToneByte, byte linearSweepByte){

  if (bitRead(singleToneByte, 7) == 1){

    unsigned int FTW0 = g_bufferFTW0Ch3.shift();
    spiTransferChST(3, FTW0);

    if (g_debug){
      Serial.print(F(" . Channel 3: "));
      Serial.println(c_STModMsg);
      printSingleTone(FTW0);
    }

  }
  else if(bitRead(linearSweepByte, 7) == 1){

    unsigned int FTW0 = g_bufferFTW0Ch3.shift();
    unsigned int FTW1 = g_bufferFTW1Ch3.shift();
    unsigned int RDW  = g_bufferRDWCh3.shift();
    unsigned int FDW  = g_bufferFDWCh3.shift();
    byte RSRR = g_bufferRSRRCh3.shift();
    byte FSRR = g_bufferFSRRCh3.shift();
    spiTransferChLS(3, FTW0, FTW1, RDW, FDW, RSRR, FSRR);

    if (g_debug){
      Serial.print(F(" . Channel 3: "));
      Serial.println(c_LSModMsg);
      printLinearSweep(FTW0, FTW1, RDW, FDW, RSRR, FSRR);
    }

  }

}


/**
 * (Single Tone Mode) Transfer channel frequency tuning word to AD9959 DDS via quad-SPI.
 * @param[in] ch  programmed channel(s)
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
  if (g_QuadSPIActive) quadSPIBufferTransfer(bufferFTWST, sizeST);
  else singleSPIBufferTransfer(bufferFTWST, sizeST);
  digitalWriteFast(c_ChipSel, HIGH);

}


/**
 * (Linear Sweep Mode) Transfer chirp words to AD9959 DDS via quad-SPI.
 * @param[in] ch   programmed channel(s)
 * @param[in] FTW0 start frequency tuning word
 * @param[in] FTW1 end frequency tuning word
 * @param[in] RDW  rising delta frequency word
 * @param[in] FDW  falling delta frequency word
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
  if (g_QuadSPIActive) quadSPIBufferTransfer(bufferFTWLS, sizeLS);
  else singleSPIBufferTransfer(bufferFTWLS, sizeLS);
  digitalWriteFast(c_ChipSel, HIGH);

}




// DUC configuration functions
/**
 * Activate the Single-Tone operation mode of the AD9959 DDS on desired input channels.
 * @param[in] chSelMask DUC channel selection mask
 */
void activateChSTM(byte chSelMask){

  // Channel Function Register bytes
  byte bufferCFR[c_CFRSize];      
  bufferCFR[0] = c_CSR;
  bufferCFR[1] = chSelMask;
  bufferCFR[2] = c_CFR;
  bufferCFR[3] = 0x02;
  bufferCFR[4] = 0x03;
  bufferCFR[5] = 0x00;

  // transfer word to DDS via quad-SPI
  digitalWriteFast(c_ChipSel, LOW);  
  if (g_QuadSPIActive){
    bufferCFR[1] = bufferCFR[1] | 0b00000110;
    quadSPIBufferTransfer(bufferCFR, c_CFRSize);
  }
  else singleSPIBufferTransfer(bufferCFR, c_CFRSize);
  digitalWriteFast(c_ChipSel, HIGH);

  // issue an I/O update pulse: CFR is set
  ioUpdate();

}


/**
 * Activate the Linear Sweep operation mode of the AD9959 DDS on desired input channels.
 * @param[in] chSelMask DUC channel selection mask
 */
void activateChLSM(byte chSelMask){

  // Channel Function Register bytes
  byte bufferCFR[c_CFRSize];
  bufferCFR[0] = c_CSR;
  bufferCFR[1] = chSelMask;
  bufferCFR[2] = c_CFR;
  bufferCFR[3] = 0b10000000;    // CFR[23:22]: Select Frequency Sweep (= 10);
                                // CFR[21:16]: Open 
  bufferCFR[4] = 0b01000011;    // CFR[15]:    Dwell mode             (=0 no-dwell disabled)
                                // CFR[14]:    Linear Sweep Enable    (=1)
                                // CFR[13]:    Load SRR at I/O Update (=0 the linear sweep ramp rate timer is loaded only upon timeout (timer=1) and is not loaded because of an I/O update pulse)
                                // CFR[12:11]: Open
                                // CFR[10]:    0
                                // CFR[9:8]:   DAC full-scale         (=11 full-scale enabled)
  bufferCFR[5] = 0b00000000;    // CFR[7:0]    (DEFAULT VALUE: 0x02)

  // transfer word to DDS via quad-SPI
  digitalWriteFast(c_ChipSel, LOW);
  if (g_QuadSPIActive){
    bufferCFR[1] = bufferCFR[1] | 0b00000110;
    quadSPIBufferTransfer(bufferCFR, c_CFRSize);
  }
  else singleSPIBufferTransfer(bufferCFR, c_CFRSize);
  digitalWriteFast(c_ChipSel, HIGH);

  // issue an I/O update pulse: CFR is set
  ioUpdate();

}


/**
 * Select AD9959 DDS channels.
 * @param[in] ch channel(s) to be selected
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
    case c_Ch2:
      bufferChSel[1] = c_chSel2;
      break;
    case c_Ch3:
      bufferChSel[1] = c_chSel3;
      break;
    case c_ChAll:
      bufferChSel[1] = c_chSel0 | c_chSel1 | c_chSel2 | c_chSel3;
      break;
    default:
      bufferChSel[1] = 0b00000000;
      Serial.println(c_chErrMsg);
      break;
  }

  // adjust serial mode bits
  if (g_QuadSPIActive) bufferChSel[1] = bufferChSel[1] | 0b00000110;

  // transfer buffer content to DUC (custom quad-SPI)
  digitalWriteFast(c_ChipSel, LOW);
  if (g_QuadSPIActive) quadSPIBufferTransfer(bufferChSel, c_SelSize);
  else singleSPIBufferTransfer(bufferChSel, c_SelSize);
  digitalWriteFast(c_ChipSel, HIGH);

}




// Custom quad-SPI functions
/**
 * Custom SPI 8-bit data transfer (with serial clock divider).
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
 * Custom quad-SPI 8-bit data transfer (with serial clock divider).
 * @param[in] data input byte to be transferred
 */
void quadSPIByteTransfer(byte data){
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble04) << 12);                        
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble04) << 12) | c_SclkHigh;
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble14) << 16);
  for (byte i = 0; i < c_QuadSclkDiv; i++) GPIO6_DR = ((data & c_Mask8Nibble14) << 16) | c_SclkHigh;
}


/**
 * Custom SPI buffer transfer (with serial clock divider).
 * @param[in] buffer input buffer to be transferred
 * @param[in] buffer_size buffer size in bytes
 */
void singleSPIBufferTransfer(byte buffer[], unsigned int buffer_size){
  // loop over buffer byte elements
  for (unsigned int b = 0; b < buffer_size; b++) singleSPIByteTransfer(buffer[b]);
  GPIO6_DR = c_SafeClearGPIO6Bit5(GPIO6_DR);
}


/**
 * Custom quad-SPI buffer transfer (with serial clock divider).
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
 * Encode DDS Frequency Tuning Word.
 * @param[in] freq frequency in MHz
 * @returns 32-bit Frequency Tuning Word
 */
unsigned int encodeFTW(float freq){

  unsigned int FTW;
  FTW = (unsigned int)round(pow(2, c_DDSbits) * freq / c_SysClk);

  return FTW;

}


/**
 * Decode DDS Frequency Tuning Word.
 * @param[in] FTW 32-bit Frequency Tuning Word
 * @returns frequency in MHz
 */
float decodeFrequency(unsigned int FTW){

  float freq;
  freq = (float)(FTW * c_SysClk / pow(2, c_DDSbits));

  return freq;

}


/**
 * Decode DDS frequency sweep ramp rate.
 * @param[in] SRR sweep ramp rate
 * @returns time step in [μs]
 */
float decodeSweepRampRate(byte SRR){

  float dt;
  dt = (float)(SRR / c_SyncClk);

  return dt;

}


/**
 * Issue an I/O update pulse via MCU board.
 * NOTE: I/O update pulses are oversampled by the SYNC_CLK,
 * therefore their width must be greater than one SYNC_CLK period, i.e. 8 ns.
 */
void ioUpdate(){

  digitalWriteFast(c_IOUpdate, HIGH);
  delayMicroseconds(c_Tassert);
  digitalWriteFast(c_IOUpdate, LOW);
        
}




// Print functions
/**
 * Print instruction string communication summary.
 */
void printSerialCOM(){

  // get current time
  g_currenTime = micros();

  // wait for serial monitor to be opened
  if (g_debug) delay(2000);

  // info  
  float elapsed = (g_currenTime - g_starTime) * 0.001;
  Serial.print(F("\n * PC >>> MCU serial communication complete!\n"));
  Serial.print(F("   Received: "));
  Serial.print(g_byteCount);
  Serial.print(F(" B, "));
  Serial.print(elapsed);
  Serial.print(F(" ms"));
  float rate = (float) g_byteCount / elapsed;
  Serial.print(F(", rate = "));
  Serial.print(rate);
  Serial.println(F(" kB/s\n"));  

}


/**
 * Print complete byte to serial monitor.
 */
void printByte(byte B){
  for(int i = 7; i >= 0; i--){
    Serial.print(bitRead(B,i));
    if(i % 4 == 0) Serial.print(" ");
  } 
  Serial.println();
}


/**
 * Print data string read time to serial monitor.
 */
void printReadTime(){
  char str[50];
  sprintf(str, "\n * String %d serial time: %luus", g_strIn, micros() - g_strTime);
  Serial.println(str);
  g_strIn++;
  g_strTime = micros();
}


/**
 * Print data string decoding time to serial monitor.
 */
void printDecodeTime(){
  char str[50];
  sprintf(str, " * String %d decode time: %luus\n", g_numIn, micros() - g_strTime);
  Serial.println(str);
}


/**
 * Print complete 32-bit uint word to serial monitor.
 */
void printUint(unsigned int UI){
  for(int i = 31; i >= 0; i--){
    Serial.print(bitRead(UI, i));
    if(i % 4 == 0) Serial.print(" ");
  } 
  Serial.println();
}


/**
 * Print single-tone channel configuration.
 */
void printSingleTone(unsigned int FTW){

  char str[60];

  Serial.print(F("   FTW:            "));
  printUint(FTW);

  sprintf(str, "   f  [MHz]:       %.6f\n", decodeFrequency(FTW));
  Serial.println(str);

}


/**
 * Print linear sweep channel configuration.
 */
void printLinearSweep(unsigned int FTW0, unsigned int FTW1, unsigned int RDW, unsigned int FDW, byte RSRR, byte FSRR){

  char  str[60];
  float df;
  float dt;

  // start frequency
  Serial.println(F("   Start"));
  Serial.print(F("   FTW0:           "));
  printUint(FTW0);  
  sprintf(str,   "   f  [MHz]:       %.6f\n", decodeFrequency(FTW0));
  Serial.println(str);

  // end frequency
  Serial.println(F("   End"));
  Serial.print(F("   FTW1:           "));
  printUint(FTW0);
  sprintf(str,   "   f  [MHz]:       %.6f\n", decodeFrequency(FTW1));
  Serial.println(str);

  // rising ramp phase
  df = decodeFrequency(RDW);
  dt = decodeSweepRampRate(RSRR);
  Serial.println(F("   Rise"));
  Serial.print(F("   RDW:            "));
  printUint(RDW);  
  sprintf(str,   "   df [MHz]:       %.9f\n", df);
  Serial.print(str);
  Serial.print(F("   RSRR:           "));
  printByte(RSRR);
  sprintf(str,   "   dt  [us]:       %.3f\n", dt);
  Serial.print(str);
  sprintf(str,   "   chirp [MHz/us]: %.6f\n", df/dt);
  Serial.println(str);

  // falling ramp phase
  df = decodeFrequency(FDW);
  dt = decodeSweepRampRate(FSRR);
  Serial.println(F("   Fall"));
  Serial.print(F("   FDW:            "));
  printUint(FDW);
  sprintf(str,   "   df [MHz]:       %.9f\n", df);
  Serial.print(str);
  Serial.print(F("   FSRR:           "));
  printByte(FSRR);
  sprintf(str,   "   dt  [us]:       %.3f\n", dt);
  Serial.print(str);
  sprintf(str,   "   chirp [MHz/us]: %.6f\n", df/dt);
  Serial.println(str);           

}
