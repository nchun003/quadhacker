#include <SPI.h>

#define CS_PIN 10

// channel minmax values
const uint8_t ch1_min = 0, ch1_max = 150;
const uint8_t ch2_min = 33, ch2_max = 110;
const uint8_t ch3_min = 33, ch3_max = 119;
const uint8_t ch4_min = 0, ch4_max = 134;


// T, A, E, R, ch5, ch6
const uint8_t channelmap[] = {3, 1, 0, 2, 4, 5};

// PLACEHOLDER - may change with battery voltage
const float vscale = (5.0f / 2.95f);

void setup() {
  pinMode(CS_PIN, OUTPUT);
  SPI.begin();
  Serial.begin(115200);
  Serial.println("Start logging values");
}

void loop() {
  uint8_t inputs[8];

  // input
  for(uint8_t i = 0; i < 8; i++) {
    inputs[i] = (uint8_t)(analogRead(A0 + i) >> 2);
    Serial.print(inputs[i]);
    Serial.print(" ");
  }
  
  Serial.print("\n");

  // output
  for(uint8_t i = 0; i < 4; i++) {
    uint8_t val = 255 - (uint8_t)(inputs[i] * vscale);
    digitalPotWrite(channelmap[i], val);
  }
}

void digitalPotWrite(int address, int value) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(CS_PIN, HIGH);
}

