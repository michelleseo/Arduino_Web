/*
 ADAM-4017 Power Line Web Logger Source Code.
 Copyright (C) 2014 Michelle Seo <mseo0318@gmail.com>
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <SPI.h>
#include <Ethernet.h>
#include <WebSocket.h>
#include <SoftwareSerial.h>
#include "Sd2Card.h"
//#include <Time.h>
#include <minWire.h>
#include <minRTClib.h>


//-------------------------------------We cannot use RAM on Display Message. Move it to code.
#include <avr/pgmspace.h>
                                            //0123456789012345 
prog_char NEW_SD_MSG[] PROGMEM  =           {"New SD"};
prog_char SD_FAIL_MSG[] PROGMEM  =          {"SD Fail"};
prog_char BLOCK_UPDATE_FAIL_MSG[] PROGMEM = {"Update Fail"};
prog_char CURR_BLOCK_MSG[] PROGMEM  =       {"*Block="};
prog_char DASH_MSG[] PROGMEM  =             {"-"};
prog_char COMMA_MSG[] PROGMEM  =            {":"};
prog_char SPACE_MSG[] PROGMEM  =            {" "};

byte mac[] = {0x90, 0xA2, 0xDA, 0x0F, 0x25, 0xC4};
byte ip[] = {192,168,219,16};

WebSocket wsServer;

//-------------------------------------SD Card

#define USED_MARK       0xaa55a500
#define SD_BASE_BLOCK   1024    //1024 is nothing special.
#define LOG_BLOCK_SIZE  120

byte sdBuffer[512];     //SD card only writes in 512 bytes block. Thus we have to have 512 bytes.
byte sdStatus = 0;
byte LogBlockSize = 0;
uint32_t currBlock;
long lastPollTime = 0;

Sd2Card sdCard;

void SDbegin(void);
void makeLogBlock(void);
void update_currBlock(void); 

//-------------------------------------RS-485
int DE485 = 2;
char rx485Ln[16];
int rx485ix;
SoftwareSerial Soft485(3, 5);

//-------------------------------------ADAM-4017
void AdamPoll(char chn);
void AdamReceiver(void);
void ADMA4017_config(void);

//-------------------------------------DS-1307
RTC_DS1307 rtc;

//-------------------------------------Data Uploading Function
void upload_lastBlockNo(void);
void upload_DataBlock(uint32_t reqBlock);
byte *setTerminator(byte *pD);

//-------------------------------------Debug
#define printLn  rx485Ln
void printCurrBlock(void);
void printTOD(void);
void printProg(prog_char *p);
void printProgln(prog_char *p);

void WSsend(char *p, int sz);

//----------------------------------------------------------------------------- setup()
void setup(){
  Serial.begin(9600);

  Wire.begin();
  rtc.begin();

  Soft485.begin(9600);
  pinMode(DE485, OUTPUT);
  ADMA4017_config();

  Ethernet.begin(mac, ip);  
  wsServer.registerConnectCallback(&onConnect);
  wsServer.registerDataCallback(&onData);
  wsServer.registerDisconnectCallback(&onDisconnect);
  wsServer.begin();
  delay(100);
 
  rx485ix = -1;
  
  SDbegin();
  if(!sdStatus) printProgln(SD_FAIL_MSG);
}

void loop(){
  wsServer.listen();

  if(sdStatus){
    if( (millis() - lastPollTime) >= 500 ){
      lastPollTime = millis();
      AdamPoll('0');
    }
  }
  AdamReceiver();
}


/*
  -----------------------------------------------------------------------------
   D A T A  B L O C K  M A K I N G
  -----------------------------------------------------------------------------
*/

//-----------------------------------------------
// SD Card Initialize
//-----------------------------------------------
void SDbegin(void){
  byte *pB;

  sdStatus = 0;
  pinMode(10, OUTPUT);    //SS

  if(!(sdCard.init(SPI_HALF_SPEED, 4))) return;     //SD Card Initialize Fail

  if(!(sdCard.readBlock(SD_BASE_BLOCK, sdBuffer))) return;  //SD Card Read Fail
  pB = sdBuffer;

  if(*((uint32_t*)pB) != USED_MARK){       //No Used Flag. New SD Card!!
    printProgln(NEW_SD_MSG);
    *((uint32_t*)pB) = USED_MARK;
    currBlock = SD_BASE_BLOCK + 1;
    *((uint32_t*)(pB+4)) = currBlock;

    if(!(sdCard.writeBlock(SD_BASE_BLOCK, sdBuffer))) return;   //SD Card Write Fail
  }
  else currBlock = *((uint32_t*)(pB+4));    //First Free Block

  LogBlockSize = 0;
  sdStatus++;  
  printCurrBlock();   //debug purpose
}

//-----------------------------------------------
// Create DATA Logging Block
//-----------------------------------------------
void makeLogBlock(void){
  byte i;
  byte *pB;
  float fdata;
  DateTime now;

  //------------------------- Reset buffer if it is the first time.
  if(!LogBlockSize){
    pB = sdBuffer + 8;    //insert Used Flag & time-stamp when finished.
    fdata = -99.999 * 10.;//-99.999 means that there is no data. When we read them we do /10 thus we have to do *10.
    i = 120;              //fill it in 120 times.
    while(i--){
      *((float*)pB) = fdata;
      pB += 4;
    }
    i = 24;             //24 bytes padding 
    while(i--){*pB = 0x00;}
  }  
  /*
    ADAM-4017 returns +dd.ddd
    It uses fixed 3 decimal points.

    To convert decimal string to float:
    double atof	(	const char * 	nptr	)	
    http://www.nongnu.org/avr-libc/user-manual/group__avr__stdlib.html#ga689c9d3c4c04463aa31d329937789d06
    
    But there is a problem. It only works with 2 decimal points.
    It rounds up third decimal point.
    Thus atof("12.345") will become 12.35.

    As a solution we do *10 then convert it to float.
    0123456   0123456
    +98.765   +987.65
  */

 //------------------------- Insert Data into buffer
                        //            0123456
                        //rx485Ln[] ">+dd.ddd"<cr>
  rx485Ln[3] = rx485Ln[4];
  rx485Ln[4] = '.';         // *10

  pB = sdBuffer + 8 + (LogBlockSize * 4);
  *((float*)pB) = atof(rx485Ln);

  LogBlockSize++;
 
  //------------------------- buffer is full
   if(LogBlockSize >= LOG_BLOCK_SIZE){
    pB = sdBuffer;
    *((uint32_t*)pB) = USED_MARK;  //Used Flag
    pB += 4;
    now = rtc.now();
    *((uint32_t*)pB) = now.unixtime();         //Time-stamp

    sdCard.writeBlock(currBlock, sdBuffer); //write to SD Card
    currBlock++;  
    LogBlockSize = 0;    
    update_currBlock();             //Fix the next free block location
  }
}

void update_currBlock(void){ 
  byte *pB;
  byte status = 0;

  if(sdCard.readBlock(SD_BASE_BLOCK, sdBuffer)){
    pB = sdBuffer;

    if(*((uint32_t*)pB) == USED_MARK){
      *((uint32_t*)(pB+4)) = currBlock;
      if(sdCard.writeBlock(SD_BASE_BLOCK, sdBuffer)){
        status++;
        printTOD();         //display time stamp
        printCurrBlock();   //display next free block
      }
    }
  }
  if(!status) printProgln(BLOCK_UPDATE_FAIL_MSG); //something is going terribly wrong
}

/*
  -----------------------------------------------------------------------------
   D A T A  U P  L O A D I N G   F u n c t i o n
  -----------------------------------------------------------------------------
*/

/*
  0         1 
  012345678901  last data block number 12bytes send
  N=4294967295

  0         1 
  012345678901  time stamp 12bytes send
  T=4294967295
  
  012345678  Data 9bytes send
  D=+99.999
  D=         empty
  D=-99.999  terminator  
*/

void xsfBlockNo(uint32_t block){
  ltoa(block - SD_BASE_BLOCK, printLn+2, 10);
  printLn[0] = 'N';
  printLn[1] = '=';
  WSsend(printLn, strlen(printLn));
}

void upload_lastBlockNo(void){
  xsfBlockNo(currBlock - 1);
}

void upload_DataBlock(uint32_t reqBlock){

  byte *pS;
  int cnt;
  float data, *pF;

  // Store the currently logging Block into SD.
  // We might lose packets that ADAM-4017 is receiving.
  sdCard.writeBlock(currBlock, sdBuffer);

  printLn[1] = '=';

  if(sdCard.readBlock(reqBlock, sdBuffer)){
    pS = sdBuffer;
    if(*((uint32_t*)pS) == USED_MARK){
      pS += 4;

      // block number
      xsfBlockNo(reqBlock);

      // time stamp (seconds)
      ltoa(*((uint32_t*)pS), printLn+2, 10);
      printLn[0] = 'T';
      WSsend(printLn, strlen(printLn));
      pS += 4;

      // data
      printLn[0] = 'D';
      pF = (float*)pS;
      cnt = 120;
      while(cnt--){
        data = *pF++ / 10.0;
        
        if(data != -99.999) dtostrf(data, 7, 3, printLn+2); 
        else printLn[2] = 0x00;

        WSsend(printLn, strlen(printLn));
      }    
    }        
    printLn[0] = 'D';
    setTerminator((byte*)(printLn+2));
    WSsend(printLn, strlen(printLn));
  }
  // Reload the block that we have stored.
  sdCard.readBlock(currBlock, sdBuffer);
}

//-----------------------------------------------
byte *setTerminator(byte *pD){
  *pD++ = '-'; 
  *pD++ = '9'; 
  *pD++ = '9'; 
  *pD++ = '.'; 
  *pD++ = '9'; 
  *pD++ = '9'; 
  *pD++ = '9'; 
  *pD++ = 0x00; 
  return pD;
}


/*
  -----------------------------------------------------------------------------
   D E B U G G I N G   D i s p l a y   F u n c t i o n
  -----------------------------------------------------------------------------
*/

void printTOD(void){
  DateTime now = rtc.now();
  Serial.print(now.year(), DEC);
  printProg(DASH_MSG);
  Serial.print(now.month(), DEC);
  printProg(DASH_MSG);
  Serial.print(now.day(), DEC);
  printProg(SPACE_MSG);
  Serial.print(now.hour(), DEC);
  printProg(COMMA_MSG);
  Serial.print(now.minute(), DEC);
  printProg(COMMA_MSG);
  Serial.println(now.second(), DEC);
}

void printCurrBlock(void){
  strcpy_P(printLn, CURR_BLOCK_MSG);
  Serial.print(printLn);
  Serial.println(currBlock);
}

void printProg(prog_char *p){
  strcpy_P(printLn, p);
  Serial.print(printLn);
}

void printProgln(prog_char *p){
  strcpy_P(printLn, p);
  Serial.println(printLn);
}

/*
  -----------------------------------------------------------------------------
   A D A M - 4 0 1 7   h a n d l e r
  -----------------------------------------------------------------------------

  Analog In
    "AIn" --->
    <--- "+dd.ddd"

    n = 0 ~ 7
    +dd.ddd : -10.000 ~ +10.000

  ADAM-4017
    "$AAn"<cr> --->
    <--- ">+dd.ddd"<cr>
*/

void AdamPoll(char chn){
  digitalWrite(DE485, HIGH);
  Soft485.write("#00");
  Soft485.write(chn);
  Soft485.write(0x0D);
  digitalWrite(DE485, LOW);
} 

void AdamReceiver(void){
  char rxChr;

  if(Soft485.available()){
    rxChr = Soft485.read();
    if(rxChr == '>') rx485ix = 0;
    else if(rx485ix >= 0){
      if(rxChr == 0x0d){      //  0123456
        if(rx485ix == 7){     // >+dd.ddd<cr>
          if(sdStatus) makeLogBlock();
        }
        rx485ix = -1;
      }
      else{
        rx485Ln[rx485ix++] = rxChr;
        if(rx485ix > 7) rx485ix = -1;
      }
    }
  }
}

void ADMA4017_config(void){
  digitalWrite(DE485, HIGH);
  Soft485.write("%0002080600");
  Soft485.write(0x0D);
  digitalWrite(DE485, LOW);
}

/*
  -----------------------------------------------------------------------------
   W E B S O C K E T   h a n d l e r
  -----------------------------------------------------------------------------
*/

void onData(WebSocket &socket, char* rxLn, byte rxSz){
  uint32_t para;
  DateTime now;

  //-----------------------------------
  // GET
  //-----------------------------------
  if( (rxLn[0] == 'G') && (rxLn[1] == 'E') && (rxLn[2] == 'T') ){
    *(rxLn + rxSz) = 0x00;
    para = atol(rxLn + 3);
    if(para) upload_DataBlock(para + SD_BASE_BLOCK);
    else upload_lastBlockNo();
  }
  //-----------------------------------
  // RTC
  //-----------------------------------
  else if( (rxLn[0] == 'R') && (rxLn[1] == 'T') && (rxLn[2] == 'C') ){
    *(rxLn + rxSz) = 0x00;
    para = atol(rxLn + 3);

    // set RTC
    if(para){
      rtc.adjust(DateTime(para));
      delay(100);
    }  
    // send current RTC
    printLn[0] = 'T';
    printLn[1] = '=';
    now = rtc.now();
    ltoa((long int)(now.unixtime()), printLn+2, 10);
    WSsend(printLn, strlen(printLn));
  }
}


void WSsend(char *p, int sz){

// Serial Monit + WebSocket
#if 1
  *(p + sz) = 0x00;
  Serial.println(p);
  wsServer.send(p, sz);
#endif

// Serial Monit only
#if 0
  *(p + sz) = 0x00;
  Serial.println(p);
#endif

// WebSocket only
#if 0
  wsServer.send(p, sz);
#endif

}
      

void onConnect(WebSocket &socket){
}

void onDisconnect( WebSocket &socket ){
}


