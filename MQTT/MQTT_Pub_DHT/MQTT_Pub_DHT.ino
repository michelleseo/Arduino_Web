/*
 MQTT Publisher Arduino Example Source Code.
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
#include <dht.h>
#include <PubSubClient.h>

#define DHT11_PIN     A0
#define POLLINTERVAL  10000   // 10seconds interval

#define CLIENTID      "ArduinoTempHumSensor"
#define TOPICNAME     "DHT11/temphum"

byte mac[]    = { 0x90, 0xA2, 0xDA, 0x0F, 0x25, 0xC4 };
byte server[] = { 212, 72, 74, 21 };    // "broker.mqttdashboard.com"

dht DHT;

PubSubClient arduinoClient(server, 1883, callback);

char charSensorVal[20];
double humidity    = 0.0; // Sensed Humidity
double temperature = 0.0; // Sensed Temperature
int seq = 0;


void setup() {
  Serial.begin(9600);
  Ethernet.begin(mac);
  
  // Connect to the MQTT Server - broker.mqttdashboard.com
  beginConnection();
}

void loop() {
  if((millis() % POLLINTERVAL) == 0) {
    if(getSensorVal()){
      dtostrf(temperature, 5, 2, charSensorVal);
      charSensorVal[5] = ',';
      dtostrf(humidity, 5, 2, charSensorVal + 6);
      arduinoClient.publish(TOPICNAME, charSensorVal);
      Serial.print(seq++);
      Serial.print(":");
      Serial.println(charSensorVal);    
    }
  }
}

void beginConnection() {

  Serial.println("Try connect to MQTT server");    
  
  int connRC = arduinoClient.connect(CLIENTID);
  
  if(!connRC) {
    Serial.println(connRC);
    Serial.println("Could not connect to MQTT Server");
    
    delay(100);
    exit(-1);
  }
  else {
    Serial.println("Connected to MQTT Server...");
  }
}

void callback(char* topic, uint8_t* payload, unsigned int length) {}

byte getSensorVal() {
  if(DHT.read11(DHT11_PIN) == DHTLIB_OK) {
    humidity = DHT.humidity;
    temperature = DHT.temperature;
    return 1;
  }
  else return 0;
}
