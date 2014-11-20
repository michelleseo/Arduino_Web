/*
    originally coded by Michelle,S
*/

#include <SoftwareSerial.h>

/*----------------------------------------------- 
  A D A M 
-----------------------------------------------*/
uint8_t ADAM_DO = 0;
uint8_t ADAM_DI = 0;
uint16_t ADAM_AI = 0;
uint8_t ADAM_Fault = 0;


#define RX485_MAX   16
char rx485Ln[RX485_MAX];
int rx485ix = -1;

/*----------------------------------------------- 
  SEQUENCE
-----------------------------------------------*/
#define STANDBY               0   // Standby
#define PRESS_DOWN            1   // Lower the Press
#define ELEVATION_DOWN        2   // Lower the Elevator that holds drill
#define ELEVATION_UP          3   // Raise the Elevator
#define PRESS_UP              4   // Raise the Press
#define ELEVATION_UP_ONE_MORE 5   // Penetration failed, raise the Elevator and lower again

#define OFF       0
#define ON        1
#define UP        2
#define DOWN      3
#define BREAK     4

typedef void (*serviceFuncPtr)(); 
serviceFuncPtr serviceFunc[]={
  &Standby,                   // 0
  &PressDown,                 // 1
  &ElevationDown,             // 2
  &ElevationUp,               // 3
  &PressUp,                   // 4
  &ElevationUpOneMore         // 5
};      


uint8_t processStep;
uint8_t tryPunchingCnt;

long serviceTime;
long lastAccessTime;

/*----------------------------------------------- 
  RS-485
-----------------------------------------------*/
int DE485 = 2;
SoftwareSerial Soft485(3, 4);


/*----------------------------------------------- 
  4 Tap Filter
-----------------------------------------------*/
#define  PRE_FILT_SIZE  4           //4
#define  PRE_FILT_SIZES_SHIFT  2

uint16_t thresholdSMA;
uint16_t PreFilt[PRE_FILT_SIZE];
uint8_t ixPreFilt;
uint8_t preStableSMA;
uint8_t stageSMA;
uint16_t avrFilt;
uint16_t activeSMACnt;


/*----------------------------------------------- 
  Press Motor
-----------------------------------------------*/
#define PRESS_TAP_FAILSAFE_BRAKE    (39321)   //3V/5V * 65536 
#define PRESS_SNA_FAILSAFE_BRAKE    (120)     //120%

// Lowest shunt value that we can decide whether Press Motor is spinning or not
#define PRESS_THRESHOLD             (23592)   //1.8V/5V * 65535 
// Press Motor SMA delay value
#define PRESS_DW_PRESTABLE          33        //33*30 = 0.99sec
// Cannot press down for over 17 seconds
#define FULL_SPAN_PRESS_DOWN_TIME   (17*1000)

/*----------------------------------------------- 
  Drill Motor
-----------------------------------------------*/
// Lowest shunt value that we can decide whether the Drill Motor is spinning or not
#define DRILL_THRESHOLD             (23592) // 1.8V/5V * 65535 
// Drill Motor SMA delay value
#define DRILL_PRESTABLE             16    //16*30 = 480ms
// If it fails to penetrate within 30 seconds, put drill back to its place and retry.
#define FULL_SPAN_DRILL_TIME        (30*1000)

#define DRILL_ROLLING_THRESHOLD     14    // 1/14 = 7% threshold
                                    // 20  5%
                                    // 16  6%
                                    // 14  7%

#define MAX_TRY_PUNCHING_CNT        6     // number of retry

#define FAN_ON_KEEP_TIME            (30*1000)   // Fan spins for extra 30 seconds


/*-----------------------------------------------------------------------------
   S E T  U P
  -----------------------------------------------------------------------------*/
void setup(){
  Serial.begin(9600);
  Soft485.begin(9600);
  pinMode(DE485, OUTPUT);
//  ADMA4050_config();    // Only used once when configurating ADAM4050
//  ADMA4017_config();    // Only used once when configurating ADAM4017
  initStandby();
}

/*-----------------------------------------------------------------------------
   L O O P
  -----------------------------------------------------------------------------*/
void loop(){
  (*serviceFunc[processStep])();
}

/*-----------------------------------------------------------------------------
    A D A M - 4 0 5 0

    DO0 : Elevation Motor Enable
    DO1 :                 P
    DO2 :                 M
    DO3 : Press Motor Enable
    DO4 :             P
    DO5 :             M
    DO6 : Drill Motor Enable
    DO7 : Fan Motor Enable

    DI0 : Limit Switch Position UP
    DI1 :                       MID
    DI2 :                       BOTTOM
    DI3 : Manual Switch START
    DI4 :               STOP
    DI5 :
    DI6 :

Digital Data Out
*********************
  #AABB(data)(cr)
    #AABB(data) is all character string excluding (cr).

    AA is 485 Address

    BB is channel address
      "00" Byte, it writes 8 channel at once.
      (data) "00" ~ "FF"
      lsb = DO 0, msb = DO 7  (Eg. Turn DO 7, DO 5 ON and turn rest OFF = "A0")

      "10" ~ "17" individual write
      "10" = DO 0 ,,, "17" = DO 7
      In this case (data) = "00" or "01"

    Response: >(Cr)

Digital Data In
*********************
  $AA6(Cr)

    AA: 485 Address
    6: Digital Data In command

  Response:
    !(dataOutput)(dataInput)00(cr)

      (dataOutput) loop back from Output
      (dataInput) Result of Digital Data In
        2 character string. "A0" means to turn DO 7, DO 5 On and rest Off.

  -----------------------------------------------------------------------------*/
#define ADAM4050_ADDR   "01"

#define ELV_ENA   1
#define ELV_P     (1<<1)
#define ELV_M     (1<<2)
#define PRS_ENA   (1<<3)
#define PRS_P     (1<<4)
#define PRS_M     (1<<5)
#define DRL_ENA   (1<<6)
#define FAN_ENA   (1<<7)

#define LMT_UP      1
#define LMT_MID     (1<<1)
#define LMT_BOTTOM  (1<<2)
#define SW_START    (1<<3)
#define SW_STOP     (1<<4)

/*
  Manual p239
  %AANNTTCCFF(cr)

*/
void ADMA4050_config(void){
  digitalWrite(DE485, HIGH);
  Soft485.write('%');
  Soft485.write("00");    // First "00" --> Switch to ADAM4017_ADDR
  Soft485.write(ADAM4050_ADDR); //NN
  Soft485.write("400600");  //TTCCFF
  Soft485.write(0x0D);
  digitalWrite(DE485, LOW);
}


/*-----------------------------------------------------------------------------
    A D A M - 4 0 1 7

    VIN0 : Elevation Motor Current
    VIN1 : Press Motor Current
    VIN2 : Drill Motor Current
    VIN3 : 
    VIN4 : 
    VIN5 : 
    VIN6 : 
    VIN7 : 


Analog Data In
*********************
  #AAN(cr)

    AA: 485 Address
    N: Channel '0' ~ '7'

  Response:
    >+dd.ddd(cr)

      +dd.ddd : -5.000 ~ +5.000, 6bytes

Configuration command
*********************
%AANNTTCCFF(cr)
  
   AA : current RS-485 address
   NN : RS-485 address to be set.
   TT : "09" Input range (page 152). +-5V
   CC : "06" 9600bps (page 154).
   FF :  "00" (page 152).
        <b7> 0 : 60Hz
        <b6> 0 : Checksum Disable
        <b5 b4 b3 b2> 0000
        <b1 b0> 00 : By doing this Current value is within Input range
                           If set to +-5V the result ranges from -5.000 to +5.000.
  -----------------------------------------------------------------------------*/
#define ADAM4017_ADDR   "02"

#define ELV_CURR   0
#define PRS_CURR   1
#define DRL_CURR   2

void ADMA4017_config(void){
  digitalWrite(DE485, HIGH);
  Soft485.write('%');
  Soft485.write("00");    // First "00" --> Switches to ADAM4017_ADDR
  Soft485.write(ADAM4017_ADDR); //NN
  Soft485.write("090600");  //TTCCFF
  Soft485.write(0x0D);
  digitalWrite(DE485, LOW);
}

/*-----------------------------------------------------------------------------
    A D A M   P O L L I N G
  -----------------------------------------------------------------------------*/
void Get_ADAM_DI(){
  while((millis() - lastAccessTime) < 33){}   //33ms --> 30Hz
  get_ADAM_DI();
}

void Get_ADAM_AI(uint8_t chn){
  while((millis() - lastAccessTime) < 33){}   //33ms --> 30Hz
  get_ADAM_AI(chn);
}

//-----------------------------------------------
// on exit : 1 = OK, 0 = Fail
uint8_t xsf_ADAM_DO(){

  long currTime;
  uint8_t retry = 3;
  uint8_t result = 0;

  while(retry--){
    lastAccessTime = millis();

    digitalWrite(DE485, HIGH);    //#AABB(data)(cr)
    Soft485.write('#');
    Soft485.write(ADAM4050_ADDR);
    Soft485.write("00");
    Soft485.print(ADAM_DO, HEX);
    Soft485.write(0x0D);
    digitalWrite(DE485, LOW);

    currTime = millis();
    while((millis() - currTime) < 500){    //0.5sec waiting
      result = rcvADAM();
      if(result) break;
    }
    if(result == '>') break;
  }
  if(result == '>'){
    ADAM_Fault = 0;
    return 1;
  }
  ADAM_Fault++;
  Serial.println(F("ADAM-4050 Digital-Out No Response..."));
  return 0;
}

//-----------------------------------------------
// on exit : 1 = OK, 0 = Fail
uint8_t get_ADAM_DI(){

  long currTime;
  uint8_t retry = 3;
  uint8_t result = 0;

  while(retry--){
    lastAccessTime = millis();

    digitalWrite(DE485, HIGH);    // $AA6(Cr)
    Soft485.write('$');
    Soft485.write(ADAM4050_ADDR);
    Soft485.write('6');
    Soft485.write(0x0D);
    digitalWrite(DE485, LOW);

    currTime = millis();
    while((millis() - currTime) < 500){    //0.5sec waiting
      result = rcvADAM();
      if(result) break;
    }
    if(result == '!') break;
  }
  if(result == '!'){
    ADAM_Fault = 0;
    return 1;
  }
  ADAM_Fault++;
  Serial.println(F("ADAM-4050 Digital-In No Response..."));
  return 0;
}

//-----------------------------------------------
// on exit : 1 = OK, 0 = Fail
uint8_t get_ADAM_AI(uint8_t chn){

  long currTime;
  uint8_t retry = 3;
  uint8_t result = 0;

  while(retry--){
    lastAccessTime = millis();

    digitalWrite(DE485, HIGH);    // #AAN(cr)
    Soft485.write('#');
    Soft485.write(ADAM4017_ADDR);
    Soft485.write(chn + '0');
    Soft485.write(0x0D);
    digitalWrite(DE485, LOW);

    currTime = millis();
    while((millis() - currTime) < 500){    //0.5sec waiting
      result = rcvADAM();
      if(result) break;
    }
    if(result == '@') break;
  }
  lastAccessTime = millis();

  if(result == '@'){
    ADAM_Fault = 0;
    return 1;
  }
  ADAM_Fault++;
  Serial.println(F("ADAM-4017 Analog-In No Response..."));
  return 0;
}

//-----------------------------------------------
uint8_t rcvADAM(){
  char rxChr;
  uint8_t result = 0;

  if(!Soft485.available()) return 0;
  rxChr = Soft485.read();

  if(rxChr == '!' || rxChr == '>'){
    rx485ix = 0;
    rx485Ln[rx485ix++] = rxChr;
  }  
  else if(rx485ix > 0 && rx485ix < RX485_MAX){
    rx485Ln[rx485ix++] = rxChr;
    if(rxChr == 0x0d){
    /*
      >(Cr)
      !(dataOutput)(dataInput)00(cr)
      >+dd.ddd(cr)  ==> duplicate, change to @ 
    */
      if(rx485Ln[0] == '!'){
        ADAM_DI = ~ASC2HEX(rx485Ln + 3);
      }
      else if(rx485Ln[0] == '>'){
        if(rx485ix > 2){
          rx485Ln[rx485ix - 1] = 0;
          ADAM_AI = float2uint16(atof(rx485Ln + 1));
          rx485Ln[0] = '@';
        }  
      }
      rx485ix = -1;
      result = rx485Ln[0];
    } //if(rxChr == 0x0d)
  } //else if(rx485ix > 0 && rx485ix < RX485_MAX)
  return result;
}


void dspVolt(uint16_t v){
  double V;
  V = v * 5.0 / 65536.;
  Serial.println(V);
}

uint16_t float2uint16(double f){
  if(f < 0) return 0;
  if(f > 5.0) return 0xffff;
  return (uint16_t)(f * 65535/5);
}

uint8_t ASC2HEX(char* pB){
  ASC2NIB(*pB) << 4 | ASC2NIB(*(pB + 1));
}
uint8_t ASC2NIB(char c){
  if(c >= 'a' && c <= 'f' ) return uint8_t(c - 'a' + 10);
  if(c >= 'A' && c <= 'F' ) return uint8_t(c - 'A' + 10);
  if(c >= '0' && c <= '9' ) return uint8_t(c - '0');
}

/*-----------------------------------------------------------------------------
    M O T O R
  -----------------------------------------------------------------------------*/
void drillMotor(uint8_t mode){
  if(mode == ON){ 
    ADAM_DO |= DRL_ENA;
  }
  else if(mode == OFF){
    ADAM_DO &= ~DRL_ENA;
  }
}  

void pressMotor(uint8_t mode){
  switch(mode){
    case OFF:
      ADAM_DO &= ~(PRS_ENA | PRS_P | PRS_M);
      break;
    case UP:
      ADAM_DO &= ~(PRS_P | PRS_M);
      ADAM_DO |= (PRS_ENA | PRS_P);
      break;
    case DOWN:
      ADAM_DO &= ~(PRS_P | PRS_M);
      ADAM_DO |= (PRS_ENA | PRS_M);
      break;
  }
}  

void elevationMotor(uint8_t mode){
  switch(mode){
    case OFF:
      ADAM_DO &= ~(ELV_ENA | ELV_P | ELV_M);
      break;
    case UP:
      ADAM_DO &= ~(ELV_P | ELV_M);
      ADAM_DO |= (ELV_ENA | ELV_P);
      break;
    case DOWN:
      ADAM_DO &= ~(ELV_P | ELV_M);
      ADAM_DO |= (ELV_ENA | ELV_M);
      break;
  }
}  

/********************************************************************** 
    S  T  A  N  D      B  Y
***********************************************************************/
void initStandby(){

  // all motors stop
  drillMotor(OFF);
  pressMotor(OFF);
  elevationMotor(OFF);
  xsf_ADAM_DO();
  
  serviceTime = millis();   //for Fan Off Check
  
  // Limit Switch check
  get_ADAM_DI();
  if(ADAM_DI & (LMT_UP | LMT_MID) == (LMT_UP | LMT_MID)){     //Elevation, Press is in original position
    processStep = STANDBY;
    Serial.println(F("++++ STAND BY ++++"));
    return;
  }
  if(!(ADAM_DI & LMT_MID)){      //Elevation is not in correct position
    initElevationUp();           //When Elevation up finishes nitPressUp() starts.
    return;
  }                 //Press is not in correct position.
  initPressUp();
  return;
}

void Standby(){
  if(ADAM_DO & FAN_ENA){ 
    if((millis() - serviceTime) >= FAN_ON_KEEP_TIME){  
      ADAM_DO &= ~FAN_ENA;   //Fan stop
      xsf_ADAM_DO();
    }
  }

  Get_ADAM_DI();                            // Manual Switch check   
  if(ADAM_DI & SW_START){                   //[START] ON --> 'drilling' STARTS
    tryPunchingCnt = MAX_TRY_PUNCHING_CNT;
    initPressDown();
  }
}

/**********************************************************************
    P  R  E  S  S    D  O  W  N
***********************************************************************/
void initPressDown(){
  pressMotor(DOWN);
  xsf_ADAM_DO();

  initSMA(PRESS_THRESHOLD, PRESS_DW_PRESTABLE);         //4 tap filter & moving average line initialize
  serviceTime = millis() + FULL_SPAN_PRESS_DOWN_TIME;   //it cannot press down for longer than this time
  processStep = PRESS_DOWN;
  Serial.println(F("++++ PRESS DOWN ++++"));
}

void PressDown(){
  static uint8_t scanCh = 0;
  long currTime;
  uint32_t i32;
  uint16_t sma;
  uint8_t stopType;

  if((millis() - lastAccessTime) < 33) return;   //33ms --> 30Hz

  if(!scanCh){  // 1/16 cycle, use 1.9Hz to do ADAM_DI()
    get_ADAM_DI();
    if(ADAM_DI & SW_STOP) initStandby();  //If [stop] button is pressend while drilling, everything retunrs to original position.
    scanCh++;
    return;
  }
  scanCh++;
  scanCh &= 0x0f;     

  stopType = 0;
  /*-------------------------------------------------------------------
  Stop Condition
    (1) FULL_SPAN_PRESS_DOWN_TIME is exceeded
    (2) motor shunt value exceeds PRESS_TAP_FAILSAFE_BRAKE
    (3) motor shunt value exceeds moving average line's PRESS_SNA_FAILSAFE_BRAKE %
  ---------------------------------------------------------------------*/
  //(1) FULL_SPAN_PRESS_DOWN_TIME is exceeded
  if(millis() > serviceTime) stopType = 1;

  if(!stopType){
    get_ADAM_AI(PRS_CURR);
    sma = getSMA(1, ADAM_AI);
    if(sma){             // Motor should be spinning
      //(2) motor shunt value exceeds PRESS_TAP_FAILSAFE_BRAKE
      if(avrFilt >= PRESS_TAP_FAILSAFE_BRAKE){
        Serial.print(F("shunt over --> "));
        dspVolt(avrFilt);
        stopType = 2;
      }  
      //(3) motor shunt value exceeds moving average line's PRESS_SNA_FAILSAFE_BRAKE %
      else{ 
        i32 = sma * PRESS_SNA_FAILSAFE_BRAKE / 100;
        if(avrFilt >= (uint16_t)i32){
          Serial.print(F("SMA over --> "));
          dspVolt(avrFilt);
          stopType = 3;
        }  
      }   
    }
  }
  /*-------------------------------------------------------------------
  STOP
    (1) Press Motor Stop
    (2) Drill Motor Start & check
  ---------------------------------------------------------------------*/
  if(!stopType) return;

  //(1) Press Motor Stop
  pressMotor(OFF);
  elevationMotor(OFF);    //fail-safe

  //(2) Drill Motor Start & check
  drillMotor(ON);
  xsf_ADAM_DO();

  initSMA(DRILL_THRESHOLD, 0);
  serviceTime = millis();
  while((millis() - serviceTime) < 2000){   //Check for maximum of 2 seconds until Drill Motor starts spinning
    Get_ADAM_AI(DRL_CURR);
    sma = getSMA(0, ADAM_AI);   //Don't use sma. Use verage instead.
    if(sma >= DRILL_THRESHOLD) break;   //drill motor checked
  }
  if(sma < DRILL_THRESHOLD){    //Stop drill motor
    drillMotor(OFF);            //Stop drill motor
    xsf_ADAM_DO();
    initPressUp();              //Return it to original position
  }
  else initElevationDown();     //drill elevator down
}

/********************************************************************** 
    E  L  E  V  A  T  I  O  N    D  O  W  N
***********************************************************************/
void initElevationDown(){
  elevationMotor(DOWN);
  ADAM_DO |= FAN_ENA;   //While drilling Fan should be ON.
  if(!xsf_ADAM_DO())return;

  initSMA(DRILL_THRESHOLD, DRILL_PRESTABLE);
  serviceTime = millis() + FULL_SPAN_DRILL_TIME;   //Maximum press down time.
  processStep = ELEVATION_DOWN;
  Serial.println(F("++++ ELEVATION DOWN ++++"));
}

void ElevationDown(){
  static uint8_t scanCh = 0;
  uint32_t i32;
  uint16_t sma;
  
  /*-------------------------------------------------------------------
   Conditions
    (1) Drilling stops if [Stop] is pressed or gets stuck by Bottom Limit Sw
    (2) If exceeds FULL_SPAN_DRILL_TIME(s) raise and lower again.
    (3) If current exceeds 7% raise Drill Motor, and lower again when it falls below SMA.
  ---------------------------------------------------------------------*/
  if((millis() - lastAccessTime) < 33) return;   //33ms --> 30Hz

  if(!scanCh){  // 1/16 cycle, 1.9Hz when using ADAM_DI().
    get_ADAM_DI();
    if((ADAM_DI & SW_STOP) || (ADAM_DI & LMT_BOTTOM)) initElevationUp();  //(1) Drilling stops if [Stop] is pressed or gets stuck by Bottom Limit Sw
    scanCh++;
    return;
  }
  scanCh++;
  scanCh &= 0x0f;

  //(2) If exceeds FULL_SPAN_DRILL_TIME(s) raise and lower again.
  if(millis() > serviceTime){
    initElevationUpOneMore(); //Raise first then retry.
    return;
  }

  //(3) If current exceeds 7% raise Drill Motor, and lower again when it falls below SMA.
  get_ADAM_AI(DRL_CURR);
  sma = getSMA(1, ADAM_AI);
  if(ADAM_DO & ELV_P){      //If Elevator is moving UP, then lower
    if(avrFilt <= sma){
    //If less than or equal to SMA lower again.
      Serial.print(F("under SMA, Drill Down --> "));
      dspVolt(avrFilt);
      elevationTurnDw();
    }
  }
  else if(ADAM_DO & ELV_M){   // Elevator Down �� �̸� Up
    if(sma){
    //If current exceeds 7% of SMA raise the Drill.
      i32 = sma / DRILL_ROLLING_THRESHOLD;   //7%
      if(avrFilt >= (sma + i32)){
        Serial.print(F("over SMA, Drill Up --> "));
        dspVolt(avrFilt);
        elevationTurnUp();
      }
    }
  }
}

/**********************************************************************
    E  L  E  V  A  T  I  O  N    U  P
***********************************************************************/
void initElevationUp(){

  Get_ADAM_DI();
  if(ADAM_DI & LMT_MID){  //At Middle Limit Sw.
  }

  if(ADAM_DO & ELV_P){    //Already elevating UP
  }
  else if(ADAM_DO & ELV_M){   //Already elevating DOWN
    elevationTurnUp();
  }
  else{                   //elevation STOPPED
    elevationMotor(UP);
    xsf_ADAM_DO();
  }
  Serial.println(F("++++ ELEVATION UP ++++"));
  processStep = ELEVATION_UP;
}

void ElevationUp(){
  if((millis() - lastAccessTime) < 33) return;   //33ms --> 30Hz

  /*-------------------------------------------------------------------
  Case 1) Middle Limit Sw is reached, then it is finished.
  ---------------------------------------------------------------------*/
  /*
  Drill Motor spinns while elevating up. (if the drill is topped, the paper might get stuck between the drill.)
  */
  get_ADAM_DI();

  //Case 1) Middle Limit Sw is reached, then it is finished.
  if(ADAM_DI & LMT_MID) DoneElevationUp();    //Middle Limit Sw reached
}

void DoneElevationUp(){
  elevationMotor(OFF);    //Elevation Motor stops
  drillMotor(OFF);        //Drill Motor stops
  xsf_ADAM_DO();
  initPressUp();
}

/**********************************************************************
    P  R  E  S  S    U  P
***********************************************************************/
void initPressUp(){
  Get_ADAM_DI();

  if(ADAM_DI & LMT_UP){   //Already at Top
    pressMotor(OFF);       //press motor Off
    xsf_ADAM_DO();
    initStandby();
    return;
  }
  pressMotor(UP);       //press motor Up
  xsf_ADAM_DO();

  Serial.println(F("++++ PRESS UP ++++"));
  processStep = PRESS_UP;
}

void PressUp(void){
  if((millis() - lastAccessTime) < 33) return;   //33ms --> 30Hz

  /*-------------------------------------------------------------------
  Case 1) Top Limit Sw is reached, then it is finished
  ---------------------------------------------------------------------*/
  get_ADAM_DI();

  //Case 1) Top Limit Sw is reached, then it is finished
  if(ADAM_DI & LMT_UP){    //Top Limit Sw reached
    pressMotor(OFF);       //Press Motor Stops
    xsf_ADAM_DO();
    initPressUp();
  }
}

/**********************************************************************
    E L E V A T I O N   U P   O N E   M O R E
***********************************************************************/
void initElevationUpOneMore(){
  if(tryPunchingCnt) tryPunchingCnt--;
  elevationTurnUp();    //elevation UP!
  processStep = ELEVATION_UP_ONE_MORE;
  Serial.println(F("++++ ELEVATION UP for Retry ++++"));
}

void ElevationUpOneMore(){
  if((millis() - lastAccessTime) < 33) return;   //33ms --> 30Hz
  /*-------------------------------------------------------------------
  Case 1) Middle Limit Sw reached.

      1. number of retry remaining
        Elevation Motor stops
        2 sec delay --> 'ELEVATION_DOWN'

      2. No more retry left --> 'ELEVATION_UP'

  Case 2) [STOP] key --> 'ELEVATION_UP'
  ---------------------------------------------------------------------*/
  get_ADAM_DI();

  if(ADAM_DI & LMT_MID){    //Middle Limit Sw reached.
    elevationMotor(OFF);    //Elevation Motor stops
    xsf_ADAM_DO();

    if(!tryPunchingCnt){   //----retry out
      DoneElevationUp();   //'ELEVATION_UP' finished, 'PRESS_UP' starts
      return;
    }
    //After 2 secs delay switches to 'ELEVATION_DOWN'. For 2 secs delay, drill keeps spinning.
    delay(2000);
    initElevationDown();
    return;
  } //if(ADAM_DI & LMT_MID)

  if(ADAM_DI & SW_STOP) initElevationUp();    //finished by [STOP] key
}

//--------------------------- elevation down!
void elevationTurnDw(){
  elevationMotor(OFF);    //elevation motor stops
  xsf_ADAM_DO();
  Get_ADAM_DI();
  if(ADAM_DI & LMT_BOTTOM) return; //if stuck at bottom limit it is at bottom position. It's impossible to go down further anyways.
  delay(100);                      //motor stops after 100ms wait
  elevationMotor(DOWN);    //elevation motor Up
  xsf_ADAM_DO();
}  
//--------------------------- elevation up!
void elevationTurnUp(){
  elevationMotor(OFF);    //elevation motor stops
  xsf_ADAM_DO();
  Get_ADAM_DI();
  if(ADAM_DI & LMT_MID) return; //if stuck at mid limit it is at top position. It's impossible to go up further.
  delay(100);                   //motor stops after 100ms wait
  elevationMotor(UP);    //elevation motor Up
  xsf_ADAM_DO();
}

/*
*/

/*-----------------------------------------------------------------------------
  -----------------------------------------------------------------------------*/


/**********************************************************************
  4 Tap  F  I  L  T  E  R     &   M O V I N G A V E R A G E

  http://cafe.naver.com/smartfa/77
***********************************************************************/


//-----------------------------------------------------------------------------
void initSMA(uint16_t threshold, uint8_t preStable){
  preStableSMA = preStable;
  thresholdSMA = threshold;

  PreFilt[0] = 0;
  PreFilt[1] = 0;
  PreFilt[2] = 0;
  PreFilt[3] = 0;
  ixPreFilt = 0;
  stageSMA = 0;
}  



//-----------------------------------------------------------------------------
// mode : b0 = 1 SMA, 0 No SMA
//        b1 = 1 AC type SMA, 0 = DC type SMA
uint16_t getSMA(uint8_t mode, uint16_t shunt) {
  static uint8_t cnt;
  static uint32_t totsum;

  uint8_t i;
  uint32_t sum;

  //-----------------------------------
  // 4 Tap Filter
  // '(pre 3 tap sum + new value) / 4' becomes 4th tab.
  //-----------------------------------
  i = (ixPreFilt - (PRE_FILT_SIZE - 1)) & (PRE_FILT_SIZE - 1);
  sum = 0;
  while(i != ixPreFilt){
    sum += PreFilt[i++];
    i &= (PRE_FILT_SIZE - 1);
  }
  sum += shunt;
  sum >>= PRE_FILT_SIZES_SHIFT;
  avrFilt = (uint16_t)sum;

  PreFilt[ixPreFilt] = avrFilt;
  ixPreFilt++;
  ixPreFilt &= (PRE_FILT_SIZE - 1);

  if(!(mode & 0x01)) return avrFilt;

  //-----------------------------------
  // Simple Moving Average
  //-----------------------------------

  activeSMACnt++;   //debugging only

  switch(stageSMA){
    case 0:
      //---------- AC type 
      if(mode & 0x02){
        sum = PreFilt[ixPreFilt];   // |4th tap - 1st tap| used
        if(sum > avrFilt) sum -= avrFilt;
        else sum = avrFilt - sum;

        if(sum >= thresholdSMA){ 
          cnt = preStableSMA;  
          stageSMA = 1;
        }
      }
      //---------- DC type
      else{
        if(avrFilt >= thresholdSMA){
          cnt = preStableSMA;  
          stageSMA = 2;
          activeSMACnt = 0;   //debugging only
        }
      }
      sum = 0;   //no SMA
      break;

    case 1:     //In case of AC + after - slope avoid
      sum = PreFilt[ixPreFilt];
      if(sum > avrFilt) sum -= avrFilt;
      else sum = avrFilt - sum;

      if(sum < thresholdSMA){
        stageSMA++;
        activeSMACnt = 0;   //debugging only
      }
      sum = 0;   //no SMA
      break;

    case 2:     //initial delay
      if(!cnt){
        totsum = 0;
        cnt = 0;
        stageSMA++;
      }
      else  cnt--;
      sum = 0;   //no SMA
      break;

    case 3:  
      totsum += avrFilt; 
      cnt++;
      sum = totsum / cnt;
      if(cnt == 255){
        totsum = sum;
        stageSMA++;
      }
      break;

    case 4:  
      if(avrFilt < thresholdSMA){
        stageSMA = 0;
        sum = 0;   //no SMA
      }  
      else sum = totsum;
      break;

    default:
      stageSMA = 0; 
      break;
  }
  return (uint16_t)sum;
}
