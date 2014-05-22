/*
 Arduino WebSocket + ADAM source code.
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

/*
  <2.0>
  input polling
*/
#include <SPI.h>
#include <Ethernet.h>
#include <WebSocket.h>
#include <SoftwareSerial.h>

byte mac[] = {0x90, 0xA2, 0xDA, 0x0F, 0x25, 0xC4};
byte ip[] = {192, 168, 219, 16};

WebSocket wsServer;

int DE485 = 2;
SoftwareSerial Soft485(3, 4);

char rx485Ln[8];
char preAdamPollRcv[2] = {0,0};
int rx485ix;
char bitMap[8];
byte wsConnect;
long lastPollTime;

void XsfAdamPollData(void);
void AdamPoll(void);
void AdamReceiver(void);
void hexToBit(char hex, int p);


void setup(){
  Serial.begin(9600);

  Soft485.begin(9600);
  pinMode(DE485, OUTPUT);

  wsConnect = 0;
  lastPollTime = 0;

  Ethernet.begin(mac, ip);  
  wsServer.registerConnectCallback(&onConnect);
  wsServer.registerDataCallback(&onData);
  wsServer.registerDisconnectCallback(&onDisconnect);
  wsServer.begin();
  delay(100);

  rx485ix = -1;
}

void loop(){
  wsServer.listen();
  if( (millis() - lastPollTime) >= 500 ) AdamPoll();
  AdamReceiver();
}

void onConnect(WebSocket &socket){
  Serial.println("onConnect called");
  wsConnect++;
}

void onDisconnect(WebSocket &socket){
  Serial.println("onDisconnect called");
  wsConnect = 0;
}

/*
  ----------------
  Digital Out
    "DOn=ON"
    "DOn=OFF"
    n = 0 ~ 7

  ADAM-4050
    "#001n01"<cr>
    "#001n00"<cr>
    n = 0 ~ 7

  ----------------
  Digital In
    "DI" --->
    <--- "bbbbbbbb"

  ADAM-4050
    "$AA6"<cr> --->
    <--- "!ooii00"<cr>
    ii = input data
*/
void onData(WebSocket &socket, char* rxLn, byte rxSz){

  // Digital Out
  if(rxLn[1] == 'O'){
    digitalWrite(DE485, HIGH);

    Soft485.write("#001");
    Soft485.write(rxLn[2]);
    Soft485.write('0');

    if(rxLn[5] == 'N') Soft485.write('1');
    else Soft485.write('0');

    Soft485.write(0x0D);

    digitalWrite(DE485, LOW);
  }

  // Digital In
  else if(rxLn[1] == 'I') XsfAdamPollData();
}

void XsfAdamPollData(void){
  hexToBit(preAdamPollRcv[0], 0);
  hexToBit(preAdamPollRcv[1], 4);
  wsServer.send(bitMap, 8);
}

void AdamPoll(void){
  digitalWrite(DE485, HIGH);
  Soft485.write("$006");
  Soft485.write(0x0D);
  digitalWrite(DE485, LOW);

  lastPollTime = millis();
} 

void AdamReceiver(void){
  char rxChr;

  if(Soft485.available()){
    rxChr = Soft485.read();
    if(rxChr == '!') rx485ix = 0;
    else if(rx485ix >= 0){
      if(rxChr == 0x0d){
        if(rx485ix == 6){     //!ooii00<cr>
          if( (preAdamPollRcv[0] != rx485Ln[2]) || (preAdamPollRcv[1] != rx485Ln[3]) ){
            preAdamPollRcv[0] = rx485Ln[2];
            preAdamPollRcv[1] = rx485Ln[3];
            if(wsConnect) XsfAdamPollData(); 
          }
        }
        rx485ix = -1;
      }
      else{
        rx485Ln[rx485ix++] = rxChr;
        if(rx485ix > 6) rx485ix = -1;
      }
    }
  }
}

void hexToBit(char hex, int p)
{
  int i;

  if(hex >= 'A') hex = hex - 0x41 + 10;
  else hex -= '0';
  hex ^= 0x0f;

  for(i=0;i<4;i++){
    if(hex & 0x08) bitMap[p + i] = '1';
    else bitMap[p + i] = '0';
    hex <<= 1;
  }
}

