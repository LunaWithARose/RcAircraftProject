#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>

// Radio
RF24 radio(6, 7); // CE=6, CSN=7
const byte address[6] = "1Node";

// Joystick pins
const int joyLX = A9; // Left stick X (rudder)
const int joyLY = A11; // Left stick Y (throttle)
const int joyRX = A10; // Right stick X (ailerons)
const int joyRY = A8; // Right stick Y (elevator)

// Payload for controls
struct ControlPayload {
  int16_t aileron;
  int16_t elevator;
  int16_t rudder;
  int16_t throttle;
  uint32_t seq;
};

uint32_t seq = 0;

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
  radio.setChannel(76);   // same channel as Teensy
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.stopListening();

  Serial.println(F("Mega transmitter ready..."));
}

// Map analog stick values to servo range (1000–2000 µs)
int16_t mapJoystick(int raw, bool throttle = false) {
  if (throttle) {
    // Throttle centered at 0, range -500 to +500
    return map(raw, 0, 1023, -180, 180);
  } else {
    // Normal control channels, range 1000–2000
    return map(raw, 0, 1023, 65, 125);
  }
}

void loop() {
  ControlPayload tx;
  tx.seq = seq++;
  tx.aileron = mapJoystick(analogRead(joyRX));
  tx.elevator = mapJoystick(analogRead(joyRY));
  tx.rudder = mapJoystick(analogRead(joyLX));
  tx.throttle = mapJoystick(analogRead(joyLY), true);

  bool ok = radio.write(&tx, sizeof(tx));
  if (ok) {
    Serial.print(F("Sent seq: "));
    Serial.print(tx.seq);
    Serial.print(F(" | AIL=")); Serial.print(tx.aileron);
    Serial.print(F(" ELEV=")); Serial.print(tx.elevator);
    Serial.print(F(" RUD=")); Serial.print(tx.rudder);
    Serial.print(F(" THR=")); Serial.println(tx.throttle);
  } else {
    Serial.println(F("Send failed"));
  }

  delay(50); // ~20Hz updates
}




/*
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>

RF24 radio(6, 7); // CE=5, CSN=6

const byte address[6] = "1Node";

struct Payload {
  uint32_t seq;
  uint32_t timestamp;
  char note[16];
};

uint32_t seq = 0;

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
  radio.stopListening();

  Serial.println(F("Mega ready, sending..."));
}

void loop() {
  Payload tx;
  tx.seq = seq++;
  tx.timestamp = millis();
  strncpy(tx.note, "PING from Mega", sizeof(tx.note));

  Serial.print(F("TX: Sending seq="));
  Serial.print(tx.seq);

  bool ok = radio.write(&tx, sizeof(tx));
  if (ok) Serial.println(F(" ... sent OK"));
  else Serial.println(F(" ... failed"));

  // Listen briefly for reply
  radio.startListening();
  unsigned long started = millis();
  bool timeout = false;

  while (!radio.available() && !timeout) {
    if (millis() - started > 200) timeout = true;
  }

  if (timeout) {
    Serial.println(F("No reply"));
  } else {
    Payload rx;
    radio.read(&rx, sizeof(rx));
    uint32_t rtt = millis() - rx.timestamp;

    Serial.print(F("RX reply: seq echo="));
    Serial.print(rx.seq);
    Serial.print(F(", age(ms)="));
    Serial.print(rtt);
    Serial.print(F(", note="));
    Serial.println(rx.note);
  }

  radio.stopListening();
  delay(1000);
}
*/