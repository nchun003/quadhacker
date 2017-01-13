#include <SPI.h>

#define CS_PIN 10

void setup() {
  pinMode(CS_PIN, OUTPUT);
  SPI.begin();
}

void loop() {
  uint8_t inputs[4];

  for(uint8_t i = 0; i < 4; i++) {
    inputs[i] = (uint8_t)(analogRead(A0 + i) >> 2);
    digitalPotWrite(i, inputs[i]);
  }
}

void digitalPotWrite(int address, int value) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(CS_PIN, HIGH);
}

