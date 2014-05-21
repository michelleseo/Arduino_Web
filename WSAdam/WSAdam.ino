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
int rx485ix;
char bitMap[8];
void AdamReceiver(void);
void hexToBit(char hex, int p);


void setup(){
  Serial.begin(9600);

  Soft485.begin(9600);
  pinMode(DE485, OUTPUT);

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
  else if(rxLn[1] == 'I'){
    digitalWrite(DE485, HIGH);
    Soft485.write("$006");
    Soft485.write(0x0D);
    digitalWrite(DE485, LOW);
  }
}


void AdamReceiver(void){
  char rxChr;

  if(Soft485.available()){
    rxChr = Soft485.read();
    if(rxChr == '!') rx485ix = 0;
    else if(rx485ix >= 0){
      if(rxChr == 0x0d){
        if(rx485ix == 6){     //!ooii00<cr>
          hexToBit(rx485Ln[2], 0);
          hexToBit(rx485Ln[3], 4);
          wsServer.send(bitMap, 8);
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

