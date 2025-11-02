#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7,8);

const byte address[6] = "00001";

void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.stopListening();
  Serial.begin(9600);
}

void loop() {
  
  int s = analogRead(0);
  int x = analogRead(1);
  int y = analogRead(2);
  String str = String(s);
  str += '|' + String(x) + '|' + String(y);
  Serial.println(str);
  const char text[20];
  
  str.toCharArray(text,20);
  Serial.println(text);
  radio.write(&text, sizeof(text));
  delay(10);

}