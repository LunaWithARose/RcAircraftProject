#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <printf.h>

// Radio
RF24 radio(20, 21); // CE=20, CSN=21
const byte address[6] = "1Node";

// Control payload
struct ControlPayload {
  int16_t aileron;
  int16_t elevator;
  int16_t rudder;
  int16_t throttle;
  uint32_t seq;
};

// Servo objects
Servo aileronServo;
Servo aileronServo2;
Servo elevatorServo;
Servo rudderServo;
Servo esc;
Servo esc2;

unsigned long lastPacket = 0;
const unsigned long FAILSAFE_TIMEOUT = 1000; // 1s

void setup() {
  Serial.begin(115200);
  while (!Serial);

  printf_begin();
  if (!radio.begin()) {
    Serial.println(F("Radio hardware not responding!"));
    while (1);
  }
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(76);
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.startListening();

  // Attach outputs (adjust pins as needed)
  aileronServo.attach(5);
  aileronServo2.attach(6);
  elevatorServo.attach(2);
  rudderServo.attach(3);
  esc.attach(4);
  esc2.attach(7);

  // ESC startup/arming sequence
  Serial.println(F("Arming ESC..."));
  esc.writeMicroseconds(2000);
  esc2.writeMicroseconds(2000);
  delay(2000);
  esc.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  delay(3000);
  Serial.println(F("ESC armed. Waiting for commands..."));

}

void loop() {
  ControlPayload rx;
  bool gotPacket = false;

  if (radio.available()) {
    radio.read(&rx, sizeof(rx));
    lastPacket = millis();
    gotPacket = true;

    // Convert throttle back from -500..500 to 1000..2000 µs
    int throttleOut = map(rx.throttle, -500, 500, 1000, 2000);

    aileronServo.writeMicroseconds(rx.aileron);
    aileronServo2.writeMicroseconds(rx.aileron);
    elevatorServo.writeMicroseconds(rx.elevator);
    rudderServo.writeMicroseconds(rx.rudder);
    esc.writeMicroseconds(throttleOut);
    esc2.writeMicroseconds(throttleOut);

    Serial.print(F("RX seq: "));
    Serial.print(rx.seq);
    Serial.print(F(" | AIL=")); Serial.print(rx.aileron);
    Serial.print(F(" ELEV=")); Serial.print(rx.elevator);
    Serial.print(F(" RUD=")); Serial.print(rx.rudder);
    Serial.print(F(" THR=")); Serial.println(throttleOut);
  }

  // Failsafe – cut throttle if no packets
  if (millis() - lastPacket > FAILSAFE_TIMEOUT) {
    esc.writeMicroseconds(1000); // idle throttle
    Serial.println(F("FAILSAFE: throttle cut"));
  }
}




/*#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>

RF24 radio(20, 21); // CE=20, CSN=21

const byte address[6] = "1Node";

struct Payload {
  uint32_t seq;
  uint32_t timestamp;
  char note[16];
};

void setup() {
  Serial.begin(115200);
  while (!Serial);

  printf_begin();
  if (!radio.begin()) {
    Serial.println(F("Radio hardware not responding!"));
    while (1);
  }

  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(76);
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.startListening();

  Serial.println(F("Teensy ready, listening..."));
}

void loop() {
  if (radio.available()) {
    Payload rx;
    radio.read(&rx, sizeof(rx));

    Serial.print(F("RX: Got seq="));
    Serial.print(rx.seq);
    Serial.print(F(", note="));
    Serial.println(rx.note);

    // Build reply
    Payload tx;
    tx.seq = rx.seq;
    tx.timestamp = rx.timestamp; // echo back sender's timestamp
    strncpy(tx.note, "PONG from Teensy", sizeof(tx.note));

    radio.stopListening();
    radio.write(&tx, sizeof(tx));
    radio.startListening();

    Serial.println(F("Sent reply"));
  }
}
*/


/*#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

MPU6050 mpu;

bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorFloat gravity;
float ypr[3];

// Offsets for "zeroing"
float yawOffset = 0;
float pitchOffset = 0;
float rollOffset = 0;

void zeroOrientation() {
  // Grab one valid DMP packet and set as offset
  do {
    fifoCount = mpu.getFIFOCount();
  } while (fifoCount < packetSize);

  mpu.getFIFOBytes(fifoBuffer, packetSize);
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  yawOffset   = ypr[0];
  pitchOffset = ypr[1];
  rollOffset  = ypr[2];

  Serial.println("Orientation zeroed!");
}

void setup() {
  Wire.begin();
  Wire.setClock(100000);

  Serial.begin(115200);
  while (!Serial);

  Serial.println("Initializing MPU6050...");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  devStatus = mpu.dmpInitialize();

  // Example hard-coded offsets (replace with calibrated ones later)
  mpu.setXAccelOffset(-1683);
  mpu.setYAccelOffset(-72);
  mpu.setZAccelOffset(1091);
  mpu.setXGyroOffset(37);
  mpu.setYGyroOffset(-14);
  mpu.setZGyroOffset(24);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    Serial.println("DMP ready! Zeroing initial orientation...");
    zeroOrientation();
  } else {
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }
}

void loop() {
  if (!dmpReady) return;

  // Check for user input
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'z' || c == 'Z') {
      zeroOrientation();
    }
  }

  fifoCount = mpu.getFIFOCount();

  if (fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println("FIFO overflow!");
  } else if (fifoCount >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetAccel(&aa, fifoBuffer);

    // Apply zero offsets
    float yaw   = (ypr[0] - yawOffset)   * 180/M_PI;
    float pitch = (ypr[1] - pitchOffset) * 180/M_PI;
    float roll  = (ypr[2] - rollOffset)  * 180/M_PI;

    Serial.print("YPR (deg): ");
    Serial.print(yaw);   Serial.print(", ");
    Serial.print(pitch); Serial.print(", ");
    Serial.print(roll);

    Serial.print(" | Accel (g): ");
    Serial.print(aa.x / 16384.0); Serial.print(", ");
    Serial.print(aa.y / 16384.0); Serial.print(", ");
    Serial.println(aa.z / 16384.0);
  }
}


*/


/*
#include <Adafruit_GPS.h>

// Use Serial2 on Teensy 4.0 (RX=7, TX=8)
Adafruit_GPS GPS(&Serial2);

uint32_t timer = millis();

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("GPS minimal output test...");

  GPS.begin(9600);

  // Only ask for RMC (recommended minimum) + GGA (fix + altitude)
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update
  GPS.sendCommand(PGCMD_ANTENNA);
}

void loop() {
  // Read data from GPS
  char c = GPS.read();

  // Parse NMEA when new sentence is available
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  // Print once per second when we have a fix
  if (millis() - timer > 1000) {
    timer = millis();

    if (GPS.fix) {
      Serial.print("Lat: ");
      Serial.print(GPS.latitude, 6); Serial.print(GPS.lat);
      Serial.print("  Lon: ");
      Serial.print(GPS.longitude, 6); Serial.print(GPS.lon);

      Serial.print("  Alt: ");
      Serial.print(GPS.altitude); Serial.print(" m");

      Serial.print("  Spd: ");
      Serial.print(GPS.speed * 1.852); Serial.print(" km/h"); // convert knots → km/h

      Serial.print("  Hdg: ");
      Serial.println(GPS.angle);
    } else {
      Serial.println("No GPS fix yet...");
    }
  }
}*/