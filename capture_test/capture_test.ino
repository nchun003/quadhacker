#include <SPI.h>

#define CS_PIN 10

// channel minmax values
const uint8_t ch1_min = 0, ch1_max = 150;
const uint8_t ch2_min = 33, ch2_max = 110;
const uint8_t ch3_min = 33, ch3_max = 119;
const uint8_t ch4_min = 0, ch4_max = 134;

// T, A, E, R, ch5, ch6
const uint8_t channelmap[] = {3, 1, 0, 2, 4, 5};

// types of data being sent
enum DATA_TYPES {DATA_START, DATA_MODE, DATA_POS_X, DATA_POS_Y, DATA_HDG, DATA_END};

// modes of operation
const uint8_t MODE_PASSTHRU = 0x11;
const uint8_t MODE_CAPTURE = 0x22;

float vscale = 0.0f;

void setup() {
  pinMode(CS_PIN, OUTPUT);
  SPI.begin();
  Serial.begin(115200);

  // calibrate ADCs
  // A4 is connected to transmitter reference
  // A5 is connected to 5V reference
  delay(1000);
  vscale = ((float)analogRead(A5) / (float)analogRead(A4));
}

int next_data = DATA_START;
uint8_t mode = MODE_PASSTHRU;
int pos_x = 0, pos_y = 0, heading = 0;

void loop() {
  uint8_t inputs[8];

  // input
  for(uint8_t i = 0; i < 8; i++) {
    inputs[i] = (uint8_t)(analogRead(A0 + i) >> 2);
  }

  // receive instructions
  if(Serial.available() > 0) {
    int data = (int)Serial.parseInt();
    switch(next_data) {
      case DATA_START:
        if(data != 9999)
          break;
        next_data = DATA_MODE;
        break;
      case DATA_MODE:
        mode = data;
        next_data = DATA_POS_X;
        break;
      case DATA_POS_X:
        pos_x = data;
        next_data = DATA_POS_Y;
        break;
      case DATA_POS_Y:
        pos_y = data;
        next_data = DATA_HDG;
        break;
      case DATA_HDG:
        heading = data;
        next_data = DATA_END;
        break;
      case DATA_END:
        next_data = DATA_START;
        break;
      default:
        break;
    }
  }

  // output
  if(mode == MODE_PASSTHRU) {
    for(uint8_t i = 0; i < 4; i++) {
      uint8_t val = 255 - (uint8_t)(inputs[i] * vscale);
      digitalPotWrite(channelmap[i], val);
    }
  } else if(mode == MODE_CAPTURE) {
    
  }
}

void digitalPotWrite(int address, int value) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(CS_PIN, HIGH);
}

