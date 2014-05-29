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
#include <Time.h>

//-------------------------------------Display Message ���� �� �ȳ��� RAM�� �� ���� ���. code�� �ű��.
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
#define SD_BASE_BLOCK   1024    //1024�� Ư���� ���ڴ� �ƴϴ�. 
#define LOG_BLOCK_SIZE  120

byte sdBuffer[512];     //SD card�� 512Bytes block ������ Write �ȴ�. �׷��� ��Ե� 512bytes�� �־�� �Ѵ�.
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

//-------------------------------------Debugging Dispaly
#define printLn  rx485Ln
void printCurrBlock(void);
void printTOD(void);
void printProg(prog_char *p);
void printProgln(prog_char *p);

void setup(){
  Serial.begin(9600);

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

  setTime(12,0,0,11,5,14);    //setTime(hours, minutes, seconds, days, months, years);
                              //RTC�� �ޱ�������� �ð��� ���� ���� ���.
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

  if(*((uint32_t*)pB) != 0xaa55a55a){       //Used Flag�� ���. New SD Card !!
    printProgln(NEW_SD_MSG);
    *((uint32_t*)pB) = 0xaa55a55a;
    currBlock = SD_BASE_BLOCK + 1;
    *((uint32_t*)(pB+4)) = currBlock;

    if(!(sdCard.writeBlock(SD_BASE_BLOCK, sdBuffer))) return;   //SD Card Write Fail
  }
  else currBlock = *((uint32_t*)(pB+4));    //ù��° Free block

  LogBlockSize = 0;
  sdStatus++;  
  printCurrBlock();   //debug purpose
}

//-----------------------------------------------
// DATA Logging Block�� �����.
//-----------------------------------------------
void makeLogBlock(void){
  byte i;
  byte *pB;
  float fdata;

  //------------------------- ó���̸� buffer�� �ʱ�ȭ �Ѵ�.
  if(!LogBlockSize){
    pB = sdBuffer + 8;    //Used Flag, Time-stamp�� �������� ��´�.
    fdata = -99.999 * 10.;//-99.999�� data�� ��ٴ� ��. read�� /10�� �ϴϱ� �̸� *10�� �� �д�.
    i = 120;              //120���� ä���.
    while(i--){
      *((float*)pB) = fdata;
      pB += 4;
    }
    i = 24;             //24 bytes padding 
    while(i--){*pB = 0x00;}
  }  
  /*
    ADAM-4017�� +dd.ddd 
    ���� �Ҽ����� ����ϰ� �Ҽ��� ���� 3�ڸ��� ����Ѵ�.

    �Ҽ��� �ִ� string�� float�� ��ȯ�ϱ� ���ؼ�
    double atof	(	const char * 	nptr	)	
    http://www.nongnu.org/avr-libc/user-manual/group__avr__stdlib.html#ga689c9d3c4c04463aa31d329937789d06
    
    �׷��� ������ �ִ�. �Ҽ��� ���� 2�ڸ����� �ۿ� �ȵȴ�.
    �Ҽ��� ���� 3�ڸ����� �ݿø��ϱ� ���� �Ѵ�.
    �� "12.345" �� atof()�� ��ȯ�ϸ� 12.35 �� �ȴ� 

    �ذ�å���� �Ҽ����ڸ��� �ϳ� �̵��� ����(�� *10) float�� ��ȯ�Ѵ�.
    0123456   0123456
    +98.765   +987.65
  */

 //------------------------- buffer���� Data�� �о� �ִ´�.
                        //            0123456
                        //rx485Ln[] ">+dd.ddd"<cr>
  rx485Ln[3] = rx485Ln[4];
  rx485Ln[4] = '.';         // *10

  pB = sdBuffer + 8 + (LogBlockSize * 4);
  *((float*)pB) = atof(rx485Ln);

  LogBlockSize++;
 
  //------------------------- buffer�� ��� ä���.
   if(LogBlockSize >= LOG_BLOCK_SIZE){
    pB = sdBuffer;
    *((uint32_t*)pB) = 0xaa55a55a;  //Used Flag �� 
    pB += 4;
    *((time_t*)pB) = now();         //Time-stamp�� ���� ����

    sdCard.writeBlock(currBlock, sdBuffer); //SD Card�� �� �ִ´�.
    currBlock++;  
    LogBlockSize = 0;    
    update_currBlock();             //���� Free block ��ġ�� �����Ѵ�.
  }
}

void update_currBlock(void){ 
  byte *pB;
  byte status = 0;

  if(sdCard.readBlock(SD_BASE_BLOCK, sdBuffer)){
    pB = sdBuffer;

    if(*((uint32_t*)pB) == 0xaa55a55a){
      *((uint32_t*)(pB+4)) = currBlock;
      if(sdCard.writeBlock(SD_BASE_BLOCK, sdBuffer)){
        status++;
        printTOD();         //time stamp display
        printCurrBlock();   //���� Free block ��ġ display
      }
    }
  }
  if(!status) printProgln(BLOCK_UPDATE_FAIL_MSG); //���� ũ�� �� �� �Ǿ� ����.
}


/*
  -----------------------------------------------------------------------------
   D E B U G G I N G   D i s p l a y   F u n c t i o n
  -----------------------------------------------------------------------------
*/

void printTOD(void){
  tmElements_t tm;

  breakTime(now(), tm);

  Serial.print(tm.Year-30);   //since 1970
  printProg(DASH_MSG);
  Serial.print(tm.Month);
  printProg(DASH_MSG);
  Serial.print(tm.Day);
  printProg(SPACE_MSG);
  Serial.print(tm.Hour);
  printProg(COMMA_MSG);
  Serial.print(tm.Minute);
  printProg(COMMA_MSG);
  Serial.print(tm.Second);
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
void onData(WebSocket &socket, char* rxLn, byte rxSz){
  // Analog In
  if( (rxLn[0] == 'A') && (rxLn[1] == 'I') ) AdamPoll(rxLn[2]);
}

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
          wsServer.send(rx485Ln, 7);
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

void onConnect(WebSocket &socket){
}

void onDisconnect( WebSocket &socket ){
}

