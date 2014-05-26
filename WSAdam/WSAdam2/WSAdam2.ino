/*
 ADAM-4017 WebSocket GateWay Source Code.
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

byte mac[] = {0x90, 0xA2, 0xDA, 0x0F, 0x25, 0xC4};
byte ip[] = {192,168,219,16};

WebSocket wsServer;

int DE485 = 2;
SoftwareSerial Soft485(3, 4);
void ADMA4017_config(void);

char rx485Ln[8];
int rx485ix;
void AdamReceiver(void);

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
}

void loop(){
  wsServer.listen();
  AdamReceiver();
}

void onConnect(WebSocket &socket){
  Serial.println("onConnect called");
}

void onDisconnect( WebSocket &socket ){
  Serial.println("onDisconnect called");
}

void ADMA4017_config(void){
  digitalWrite(DE485, HIGH);
  Soft485.write("%0002080600");
  Soft485.write(0x0D);
  digitalWrite(DE485, LOW);
}

/*
  ----------------
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
  if( (rxLn[0] == 'A') && (rxLn[1] == 'I') ){
    digitalWrite(DE485, HIGH);
    Soft485.write("#00");
    Soft485.write(rxLn[2]);
    Soft485.write(0x0D);
    digitalWrite(DE485, LOW);
  }
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

