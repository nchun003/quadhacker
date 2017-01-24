#include <SPI.h>
#include "PID_v1.h"

#define CS_PIN 10
#define LED_MODE 12

// channel minmax values
const uint8_t ch1_min = 0, ch1_max = 150;
const uint8_t ch2_min = 33, ch2_max = 110;
const uint8_t ch3_min = 33, ch3_max = 119;
const uint8_t ch4_min = 0, ch4_max = 134;

// T, A, E, R, ch5, ch6
const uint8_t channelmap[] = {3, 1, 0, 2, 4, 5};

// types of data being sent
const int DATA_START_VAL = 7777;
const int DATA_END_VAL = 9999;
enum DATA_TYPES {DATA_START, DATA_MODE, DATA_POS_X, DATA_POS_Y, DATA_HDG, DATA_ALT, DATA_END};

// modes of operation
const uint8_t MODE_PASSTHRU = 0;
const uint8_t MODE_CAPTURE = 1;

float vscale = 0.0f;

// PID tuning values
double ch1_kp = 2, ch1_ki = 0.5, ch1_kd = 0.25;
double ch2_kp = 1, ch2_ki = 0.5, ch2_kd = 0.25;
double ch3_kp = 1, ch3_ki = 0.5, ch3_kd = 0.25;
double ch4_kp = 1, ch4_ki = 0.5, ch4_kd = 0.25;

// PID variables
double ch1_setpoint, ch1_input, ch1_output;
double ch2_setpoint, ch2_input, ch2_output;
double ch3_setpoint, ch3_input, ch3_output;
double ch4_setpoint, ch4_input, ch4_output;

// PID objects
PID ch1_pid(&ch1_input, &ch1_output, &ch1_setpoint, ch1_kp, ch1_ki, ch1_kd, DIRECT);
PID ch2_pid(&ch2_input, &ch2_output, &ch2_setpoint, ch2_kp, ch2_ki, ch2_kd, DIRECT);
PID ch3_pid(&ch3_input, &ch3_output, &ch3_setpoint, ch3_kp, ch3_ki, ch3_kd, DIRECT);
PID ch4_pid(&ch4_input, &ch4_output, &ch4_setpoint, ch4_kp, ch4_ki, ch4_kd, DIRECT);

void setup() {
  pinMode(CS_PIN, OUTPUT);
  SPI.begin();
  Serial.begin(115200);

  pinMode(LED_MODE, OUTPUT);

  // calibrate ADCs
  // A4 is connected to transmitter reference
  // A5 is connected to 5V reference
  delay(1000);
  vscale = ((float)analogRead(A5) / (float)analogRead(A4));
}

int next_data = DATA_START;
int pos_x = 0, pos_y = 0, heading = 0, altitude = 0;

uint8_t mode = MODE_PASSTHRU;
uint8_t mode_requested;
uint8_t flag_mode_change = 0;

void loop() {
  uint8_t inputs[8];
  uint8_t control_inputs[8];

  // input
  for(uint8_t i = 0; i < 8; i++) {
    inputs[i] = (uint8_t)(analogRead(A0 + i) >> 2);
  }

  // receive instructions
  if(Serial.available() > 0) {
    int data = (int)Serial.parseInt();
    //Serial.println(data);
    switch(next_data) {
      case DATA_START:
        if(data != DATA_START_VAL) break;
        next_data = DATA_MODE;
        break;
      case DATA_MODE:
        // mode changes are request-based
        if(data != mode) {
          flag_mode_change = 1;
          mode_requested = data;
        }
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
        next_data = DATA_ALT;
        break;
      case DATA_ALT:
        altitude = data;
        next_data = DATA_END;
      case DATA_END:
        if(data != DATA_END_VAL) break;
        next_data = DATA_START;
        break;
      default:
        break;
    }
  }

  // TODO: more complex mode change logic
  if(flag_mode_change) {
    mode = mode_requested;
  }

  // output
  if(mode == MODE_PASSTHRU) {

    // run this once on mode change
    if(flag_mode_change) {

      // deactivate pids
      ch1_pid.SetMode(MANUAL);
      ch2_pid.SetMode(MANUAL);
      ch3_pid.SetMode(MANUAL);
      ch4_pid.SetMode(MANUAL);

      // lower flags
      flag_mode_change = 0;
    }
    
    // direct manual mode
    for(uint8_t i = 0; i < 4; i++) {
      uint8_t val = 255 - (uint8_t)(inputs[i] * vscale);
      digitalPotWrite(channelmap[i], val);
    }

    Serial.println("No track");
    
    digitalWrite(LED_MODE, LOW);
  } else if(mode == MODE_CAPTURE) {
    
    // capture mode - switch to PID control

    // run this once on mode change
    if(flag_mode_change) {

      // activate pids
      ch1_pid.SetMode(AUTOMATIC);
      ch2_pid.SetMode(AUTOMATIC);
      ch3_pid.SetMode(AUTOMATIC);
      ch4_pid.SetMode(AUTOMATIC);

      // lower flags
      flag_mode_change = 0;
    }

    // pid sensor inputs
    ch1_input = altitude;
    ch2_input = pos_x * sin(heading) + pos_y * cos(heading);
    ch3_input = pos_y * sin(heading) + pos_x * cos(heading);
    ch4_input = heading;

    // pid setpoints (targets)
    ch1_setpoint = 50;
    ch2_setpoint = 0;
    ch3_setpoint = 0;
    ch4_setpoint = 45;

    // crunch the pids
    ch1_pid.Compute();
    ch2_pid.Compute();
    ch3_pid.Compute();
    ch4_pid.Compute();

    // read out pid outputs
    control_inputs[0] = (uint8_t)ch1_output;
    control_inputs[1] = (uint8_t)ch2_output;
    control_inputs[2] = (uint8_t)ch3_output;
    control_inputs[3] = (uint8_t)ch4_output;
    /*
    // write to RDAC
    for(uint8_t i = 0; i < 4; i++) {
      uint8_t val = 255 - (uint8_t)(control_inputs[i] * vscale);
      digitalPotWrite(channelmap[i], val);
    }*/
    digitalPotWrite(channelmap[0], 255 - (uint8_t)(control_inputs[0] * vscale));
    digitalPotWrite(channelmap[1], 255 - (uint8_t)(inputs[1] * vscale));
    digitalPotWrite(channelmap[2], 255 - (uint8_t)(inputs[2] * vscale));
    digitalPotWrite(channelmap[3], 255 - (uint8_t)(inputs[3] * vscale));

    // serial debug
    Serial.print("ALT: ");
    Serial.println(altitude);
    Serial.print("CH1: ");
    Serial.println(ch1_output);
    //Serial.print(255 - (uint8_t)(control_inputs[0] * vscale));
    
    digitalWrite(LED_MODE, HIGH);
  }
}

void digitalPotWrite(int address, int value) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(CS_PIN, HIGH);
}

