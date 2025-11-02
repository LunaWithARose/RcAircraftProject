#include <Arduino.h>
//Servo v
#include <Servo.h>
//RF v
#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
//MPU v
#include <MPU6050.h>
//GPS v
#include <TinyGPSPlus.h>
#include <TinyGPS.h>
#include <TinyGPS++.h>
//BMP v
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

Servo esc; 
Servo sx; 
Servo sy; 
RF24 radio(7,8);

const byte address[6] = "00001";

void setup() {
  // put your setup code here, to run once:
    radio.begin();
  radio.openReadingPipe(0,address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  esc.attach(9); 
  sx.attach(4); 
  sy.attach(5);
  esc.writeMicroseconds(1000); //initialize the signal to 1000
  radio.startListening();
  Serial.begin(9600);

  
}

void loop() {
  char text[32] = "";
  
  if (radio.available()) {
    radio.read(&text, sizeof(text));
    String transData = String(text);
    //Serial.println(getValue(transData,'|',1));

    int s = getValue(transData,'|',0).toInt();
    s= map(s, 0, 1023,1000,2000); //mapping val to minimum and maximum(Change if needed) 
    Serial.println(transData);
    esc.writeMicroseconds(s); //using val as the signal to esc
    
    int sxVal = getValue(transData,'|',1).toInt();
    int syVal = getValue(transData,'|',2).toInt();

    sx.write(map(sxVal, 0, 1023,0,180));
    sy.write(map(syVal, 0, 1023,0,180));

  }
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

