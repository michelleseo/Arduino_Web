/*
 Arduino Temperature & Humidity Web Logger source code.
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
#include "Sd2Card.h"
#include <minWire.h>
#include <minRTClib.h>
#include <dht.h>

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
#define USED_MARK       0xaa55a503
#define SD_BASE_BLOCK   1024    //1024 is nothing special.
#define LOG_BLOCK_SIZE  96

byte sdBuffer[512];     //SD card only writes in 512 bytes block. Thus we have to have 512 bytes.
byte sdStatus = 0;
byte LogBlockSize = 0;
uint32_t currBlock;
uint32_t lastPollTime = 0;

Sd2Card sdCard;

void SDbegin(void);
void makeLogBlock(void);
void update_currBlock(void); 

//-------------------------------------DS-1307
RTC_DS1307 rtc;

//-------------------------------------DHT-11
#define DHT11_PIN A0
//#define SAMPLING_INTERVAL (3*1000)
#define SAMPLING_INTERVAL (uint32_t)60*1000     //Read sensor every 1 minute. Arduino default is 16 bit int. If you don't do (uint32_t) it becomes -5536.
#define MAX_REC_SIZE  42                // 512/12 = 42.666...

dht DHT;

float humidity;
float temperature;

void DHTPoll();

//-------------------------------------Data Uploading Function
void upload_lastBlockNo(void);
void upload_DataBlock(uint32_t reqBlock);

//-------------------------------------Debug
char printLn[26];
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

  Ethernet.begin(mac, ip);  
  wsServer.registerConnectCallback(&onConnect);
  wsServer.registerDataCallback(&onData);
  wsServer.registerDisconnectCallback(&onDisconnect);
  wsServer.begin();
  delay(100);
  
  SDbegin();
  if(!sdStatus) printProgln(SD_FAIL_MSG);
}

void loop(){
  wsServer.listen();

  if(sdStatus){
    if((millis() - lastPollTime) >= SAMPLING_INTERVAL){
      lastPollTime = millis();
      DHTPoll();
    }
  }
}

/*
  -----------------------------------------------------------------------------
   D H T - 1 1   h a n d l e r
  -----------------------------------------------------------------------------
*/
void DHTPoll() {
  if(DHT.read11(DHT11_PIN) == DHTLIB_OK) {
    humidity = DHT.humidity;
    temperature = DHT.temperature;
    if(sdStatus) makeLogBlock();
  }
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

  //load last block  
  sdCard.readBlock(currBlock, sdBuffer);
  pB = sdBuffer;
  if(*((uint32_t*)pB) != USED_MARK){  //New data block.
    *((uint32_t*)pB) = USED_MARK;     //Used Flag
    *((uint32_t*)(pB + 4)) = 0;       //Set record number to 0 for this Logging Block and add 3 bytes padding.
  }  
  LogBlockSize = sdBuffer[4];         //Used data block.

  sdStatus++;  
  printCurrBlock();   //debug purpose
}

//-----------------------------------------------
// Create DATA Logging Block
//-----------------------------------------------
void makeLogBlock(void){
  byte *pB;
  DateTime now;

 //------------------------- push Data into the buffer
  pB = sdBuffer + 8 + (LogBlockSize * 12);
  now = rtc.now();
  *((uint32_t*)pB) = now.unixtime();        //Time-stamp 
  *((float*)(pB + 4)) = temperature;         //Temperature
  *((float*)(pB + 8)) = humidity;            //Humidity

  printTOD();         //time stamp display
  Serial.print("\t");
  Serial.print(LogBlockSize);
  Serial.print("\t");
  Serial.print(temperature,1);
  Serial.print("C\t");
  Serial.print(humidity,1);
  Serial.println("%");

 //------------------------- Write to SD card everytime
  LogBlockSize++;
  sdBuffer[4] = LogBlockSize;

  sdCard.writeBlock(currBlock, sdBuffer);

 //------------------------- If Data block is complete, fix the next Free Block and reset Buffer
  if(LogBlockSize >= MAX_REC_SIZE){
    currBlock++;
    update_currBlock();

    pB = sdBuffer;
    *((uint32_t*)pB) = USED_MARK;     //Used Flag
    *((uint32_t*)(pB + 4)) = 0;       //Set record number to 0 for this Logging Block and add 3 bytes padding.
    LogBlockSize = 0;
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
  char *pC;
  int cnt;

  // Store the currently logging Block into SD.
  sdCard.writeBlock(currBlock, sdBuffer);

  printLn[1] = '=';

  if(sdCard.readBlock(reqBlock, sdBuffer)){
    pS = sdBuffer;
    if(*((uint32_t*)pS) == USED_MARK){
      pS += 4;

      // block number
      xsfBlockNo(reqBlock);

      cnt = *pS;
      pS += 4;
      printLn[0] = 'D';
      while(cnt--){
        // time stamp (seconds)
        ltoa(*((uint32_t*)pS), printLn+2, 10);
        pC = printLn + strlen(printLn);
        *pC++ = '=';
        pS += 4;

        // Temperature
        dtostrf(*((float*)pS), 6, 2, pC);  //-12.34C
        pC = printLn + strlen(printLn);
        *pC++ = '=';
        pS += 4;

        // Humidity
        dtostrf(*((float*)pS), 6, 2, pC);  //12.34%
        pS += 4;
        WSsend(printLn, strlen(printLn));
      }
      printLn[2] = '0';   //timestamp == 0, end-of-data
      WSsend(printLn, 3);
    }
  }
  // Reload the block that we have stored.
  sdCard.readBlock(currBlock, sdBuffer);
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
  Serial.print(now.second(), DEC);
}

void printCurrBlock(void){
  strcpy_P(printLn, CURR_BLOCK_MSG);
  Serial.print(printLn);
  Serial.print(currBlock);
  Serial.print(",");
  Serial.println(LogBlockSize);
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
    else{
     upload_lastBlockNo();
    }
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


