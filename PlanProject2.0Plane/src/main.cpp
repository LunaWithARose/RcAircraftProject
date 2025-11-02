#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <MPU6050_tockn.h>


/*PIN-USED-------------

RADIO-----
MOSI    11*
MISO    12*
SCK     13*
CE      3
CS      2

SERVO------
Throt1  4
Throt2  5
RUD     6
ELV     7
ALR11   8
ALR12   9
ALR21   10
ALR22   14

GPS-------
TX      21
RX      20

MPU-------
SCL     17
SDA     16

BMP-------
SCL     17
SDA     16

MISC------
LED red 15
LED blue 18
LED green 19
BAT     A8

*/


//LED-------------------
#define red 15
#define blue 18
#define green 19
 

//RADIO----------------
RF24 radio(3, 2);   // nRF24L01 (CE, CSN)
const byte address[6] = "00001";
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

int throttleValue1, throttleValue2, rudderValue, elevatorValue, aileron11Value, aileron12Value, aileron2Value, aileron2Value, travelAdjust;

//Data Package Incoming
struct Data_Package {
  byte j1PotX;
  byte j1PotY;
  byte j2PotX;
  byte j2PotY;
  byte tSwitch1;
  byte tSwitch2;
  byte mode;
  byte potent;
};
Data_Package data; //Create a variable with the above structure

//Data Package Outgoing
struct Data_Package2 {
  byte lat;
  byte lon;
  byte head;
  byte xacc;
  byte xrot;
  byte yacc;
  byte yrot;
  byte zacc;
  byte zrot;
  byte alt;
  byte bat;
};
Data_Package2 data2;

struct Data_Package3 {
  byte setup;
};
Data_Package3 data3;

struct Data_Package4 {
  byte setup2;
  byte setup3;
};
Data_Package4 data4;


//SERVO----------------
Servo throttle1;  // create servo object to control the ESC
Servo throttle2;
Servo rudderServo;
Servo elevatorServo;
Servo aileron11Servo;
Servo aileron12Servo;
Servo aileron21Servo;
Servo aileron22Servo;

int preset1 = 0;
int preset2 = 0;
int preset3 = 0;
int preset4 = 0;
int preset5 = 0;
int preset6 = 0;


//GPS------------------
float lat ,lon ; 

SoftwareSerial gpsSerial(20,21);

TinyGPS gps;


//MPU-------------------
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;


//BMP-------------------

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

long alt = 0;

//MISC------------------
int senddat =  0; 
int uhoh = 0;
int mode = 0;
int setup10 = 1;

#define BAT 14

void setup() {
  Serial.begin(9600);

  //LED
  pinMode(red, OUTPUT);  
  pinMode(green, OUTPUT);  
  pinMode(blue, OUTPUT);  

  analogWrite(green, 255);

  //RADIO
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  resetData();

  //SERVO
  throttle1.attach(4);
  throttle2.attach(5);
  rudderServo.attach(6);   
  elevatorServo.attach(7); 
  aileron11Servo.attach(8);
  aileron12Servo.attach(9);
  aileron21Servo.attach(10);
  aileron22Servo.attach(14); 

  //GPS
  gpsSerial.begin(9600);

  //MPU
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);
  

  //BMP
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
  while (1);
  }
   bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);


  analogWrite(green, 0);
}

void loop() {
  if(setup10 == 1){
    radio.openWritingPipe(address);
    
  }
  // Check whether we keep receving data, or we have a connection between the two modules
  currentTime = millis();
  if ( currentTime - lastReceiveTime > 1000 ) { // If current time is more then 1 second since we have recived the last data, that means we have lost connection
    resetData(); // If connection is lost, reset the data. It prevents unwanted behavior, for example if a drone jas a throttle up, if we lose connection it can keep flying away if we dont reset the function
  }
  // Check whether there is data to be received
  readData();

 if(mode == 1){
  rcMode();
 }
 if(mode == 2){
  flyByWire();
 }
 if(senddat == 30){
  writeData();
 }
 if(senddat < 30){
  senddat++; 
 }

 
}

void resetData() {
  // Reset the values when there is no radio connection - Set initial default values
  data.j1PotX = 127;
  data.j1PotY = 80; // Motors stops // the central point of the joystick is not starting point for the throttle, its at value of 80 instead of 127
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.tSwitch1 = 1;
  data.tSwitch2 = 1;
}

void readData() {
  
  radio.openReadingPipe(0, address);
  radio.startListening(); //  Set the module as receiver
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    lastReceiveTime = millis(); // At this moment we have received the data
  }
  throttleValue1 = constrain(data.j1PotY, 80, 255); // Joysticks stays in middle. So we only need values the upper values from 130 to 255
  throttleValue1 = map(throttleValue1, 80, 255, 1000, 2000);
  throttleValue1 = constrain(data.j1PotY, 80, 255); // Joysticks stays in middle. So we only need values the upper values from 130 to 255
  throttleValue1 = map(throttleValue1, 80, 255, 1000, 2000);

  throttleValue2 = constrain(data.j1PotY, 80, 255); // Joysticks stays in middle. So we only need values the upper values from 130 to 255
  throttleValue2 = map(throttleValue2, 80, 255, 1000, 2000);
  throttleValue2 = constrain(data.j1PotY, 80, 255); // Joysticks stays in middle. So we only need values the upper values from 130 to 255
  throttleValue2 = map(throttleValue2, 80, 255, 1000, 2000);

  mode = data.mode;

   // Controlling throttle - brushless motor with ESC
  int sensorValue = analogRead(A8);
  float voltage = sensorValue * (5.00 / 1023.00) * 3; // Convert the reading values from 5v to suitable 12V i
  // If voltage is below 11V turn on the LED

  if (voltage < 11) {
    digitalWrite(red, HIGH);
  }
  else {
    digitalWrite(red, LOW);
  }
  radio.stopListening();
}

void writeData(){
  radio.openWritingPipe(address);

int sensorValue = analogRead(A8);
float voltage = sensorValue * (5.00 / 1023.00) * 3;
if (voltage < 11) {
 uhoh = 1;
}
GPS();
MPU6050();
BMP390();

data2.lat = lat;
data2.lon = lon;
data2.xrot = roll;
data2.yrot = pitch;
data2.zrot = yaw;
data2.bat = uhoh;
data2.alt = alt;

radio.write(&data2, sizeof(Data_Package2));
}

void GPS(){
while(gpsSerial.available()){ // check for gps data
  if(gps.encode(gpsSerial.read()))// encode gps data
  {
   gps.f_get_position(&lat,&lon); // get latitude and longitude
   // display position
   String latitude = String(lat,6);
   String longitude = String(lon,6);
  }
}
}

void MPU6050(){
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX + 0.56; // GyroErrorX ~(-0.56)
  GyroY = GyroY - 2; // GyroErrorY ~(2)
  GyroZ = GyroZ + 0.79; // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  
  // Print the values on the serial monitor
}

void BMP390(){
  
  float alt = bmp.readAltitude(SEALEVELPRESSURE_HPA)*3.28084;   
}


void rcMode(){

  throttle1.writeMicroseconds(throttleValue1);

  throttle2.writeMicroseconds(throttleValue2);

  // Adjusting the servos responsiveness 
  
  // Elevator control
  elevatorValue = map(data.j2PotY, 0, 255, -35, 35);
  elevatorServo.write(elevatorValue);
  
  // Ailerons control
  aileron11Value = map(data.j1PotX, 0, 255, preset1, preset2);
  aileron12Value = map(data.j1PotX, 0, 255, preset3, preset4);
  aileron11Servo.write(aileron11Value);
  aileron12Servo.write(aileron12Value);
  aileron21Servo.write(-1 * aileron11Value);

  // Rudder control
  rudderValue = map(data.j2PotY, 0, 255, -45, 45);
  rudderServo.write(rudderValue);

   
}

void flyByWire(){
  MPU6050();
  readData();
  //FIX THIS CODE IF WRONG
  elevatorValue = map(data.j2PotY, 0, 255, -35, 35);


  

  

}
