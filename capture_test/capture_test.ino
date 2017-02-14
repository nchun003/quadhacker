#include <SPI.h>
#include "PID_v1.h"

#define CS_PIN 10
#define LED_MODE 12

#define PIN_CH1 A0
#define PIN_CH2 A1
#define PIN_CH3 A2
#define PIN_CH4 A3

// channel minmax values
const uint8_t ch1_min = 0, ch1_max = 150;
const uint8_t ch2_min = 33, ch2_max = 110;
const uint8_t ch3_min = 33, ch3_max = 119;
const uint8_t ch4_min = 0, ch4_max = 134;

// T, A, E, R, ch5, ch6
const uint8_t channelmap[] = {3, 1, 0, 2, 4, 5};

// maximum value any PID can reach
const uint8_t PID_MAX = 70;

// PID tuning values
double ch1_kp = 1, ch1_ki = 0.1, ch1_kd = 0.5;
double ch2_kp = 0.8, ch2_ki = 0.4, ch2_kd = 0.3;
double ch3_kp = 1, ch3_ki = 0.5, ch3_kd = 0.2;
double ch4_kp = 2, ch4_ki = 0, ch4_kd = 0;

// PID variables
double ch1_setpoint, ch1_input, ch1_output;
double ch2_setpoint, ch2_input, ch2_output;
double ch3_setpoint, ch3_input, ch3_output;
double ch4_setpoint, ch4_input, ch4_output;

// PID objects
PID ch1_pid(&ch1_input, &ch1_output, &ch1_setpoint, ch1_kp, ch1_ki, ch1_kd, REVERSE);
PID ch2_pid(&ch2_input, &ch2_output, &ch2_setpoint, ch2_kp, ch2_ki, ch2_kd, REVERSE);
PID ch3_pid(&ch3_input, &ch3_output, &ch3_setpoint, ch3_kp, ch3_ki, ch3_kd, DIRECT);
PID ch4_pid(&ch4_input, &ch4_output, &ch4_setpoint, ch4_kp, ch4_ki, ch4_kd, REVERSE);

// types of data being sent
const int DATA_START_VAL = 7777;
const int DATA_END_VAL = 9999;
enum DATA_TYPES {DATA_START, DATA_MODE, DATA_POS_X, DATA_POS_Y, DATA_HDG, DATA_ALT, DATA_END};

// state which denotes which information is next
int next_data = DATA_START;

// modes of operation
const uint8_t MODE_PASSTHRU = 0;
const uint8_t MODE_CAPTURE = 1;

// mode logic variables
uint8_t mode = MODE_PASSTHRU;
uint8_t mode_requested;
uint8_t flag_mode_change = 0;

// voltage scale for RDAC
float vscale = 0.0f;

// pre-filter values
float pos_x_pre = 0.0f, pos_y_pre = 0.0f, heading_pre = 0.0f, altitude_pre = 0.0f;
// filtered inputs
float pos_x = 0.0f, pos_y = 0.0f, heading = 0.0f, altitude = 0.0f;

// calibration variables
uint8_t flag_calibrated = 0;
uint8_t calibration_stage = 0;

unsigned long time_capture_lost = 0;
uint8_t flag_capture_lost = 0;
unsigned long capture_timeout = 1000; // wait 1000 milliseconds to resume manual on capture loss

// input filtering
uint8_t enable_filtering = 0;
uint8_t flag_data_received = 0;
const uint8_t data_filter_max = 4;
float data_filter[4][data_filter_max] = { 0.0f };
uint8_t data_filter_index = 0;
uint8_t data_filter_size = 0;


void setup() {
  // SPI for RDAC
  pinMode(CS_PIN, OUTPUT);
  SPI.begin();

  // serial tether
  Serial.begin(115200);

  // mode indicator LED
  pinMode(LED_MODE, OUTPUT);

  // setup PID output limits
  ch1_pid.SetOutputLimits(-20, 20);
  ch2_pid.SetOutputLimits(-PID_MAX, PID_MAX);
  ch3_pid.SetOutputLimits(-PID_MAX, PID_MAX);
  ch4_pid.SetOutputLimits(-PID_MAX, PID_MAX);

  // set PID sample time
  ch1_pid.SetSampleTime(10);
  ch2_pid.SetSampleTime(10);
  ch3_pid.SetSampleTime(10);
  ch4_pid.SetSampleTime(10);

  // vscale placeholder
  vscale = 3/5;

  // blink ready
  Serial.println("Guidance module ready");
  for(int i = 0; i < 6; i++) { digitalWrite(LED_MODE, (i % 2) ? HIGH : LOW); delay(150); }
}


void loop() {
  uint8_t inputs[8];
  uint8_t control_inputs[8];

  // input
  for(uint8_t i = 0; i < 8; i++) {
    inputs[i] = (uint8_t)(analogRead(PIN_CH1 + i) >> 2);
  }

  // voltage scaling
  vscale = ((float)analogRead(A5) / (float)analogRead(A4));

  // receive instructions
  if(Serial.available() > 0) {
    //Serial.println(data);
    int data;
    switch(next_data) {
      case DATA_START:
        data = (int)Serial.parseInt();
        if(data != DATA_START_VAL) break;
        next_data = DATA_MODE;
        break;
      case DATA_MODE:
        // mode changes are request-based
        data = (int)Serial.parseInt();
        if(data != mode) {
          if(mode == MODE_CAPTURE) {
            if(time_capture_lost == 0) {
              time_capture_lost = millis();
            } else if(millis() - time_capture_lost >= capture_timeout) {
              flag_mode_change = 1;
              mode_requested = data;
              time_capture_lost = 0;
            }
          } else {
            flag_mode_change = 1;
            mode_requested = data;
          }
        } else {
          time_capture_lost = 0;
        }
        flag_capture_lost = !(time_capture_lost == 0);
        next_data = DATA_POS_X;
        break;
      case DATA_POS_X:
        digitalWrite(LED_MODE, HIGH);
        pos_x_pre = Serial.parseFloat();
        digitalWrite(LED_MODE, LOW);
        next_data = DATA_POS_Y;
        break;
      case DATA_POS_Y:
        pos_y_pre = Serial.parseFloat();
        next_data = DATA_HDG;
        break;
      case DATA_HDG:
        heading_pre = Serial.parseFloat();
        next_data = DATA_ALT;
        break;
      case DATA_ALT:
        altitude_pre = Serial.parseFloat();
        flag_data_received = 1;
        next_data = DATA_END;
        break;
      case DATA_END:
        data = (int)Serial.parseInt();
        if(data != DATA_END_VAL) break;
        next_data = DATA_START;
        break;
      default:
        data = (int)Serial.parseInt();
        Serial.print("JUNK VALUES >> ");
        Serial.println(data);
        break;
    }
  }

  // input filtering
  if(enable_filtering && flag_data_received) {
    // store raw inputs
    uint8_t j = data_filter_index;
    data_filter[0][j] = pos_x_pre;
    data_filter[1][j] = pos_y_pre;
    data_filter[2][j] = heading_pre;
    data_filter[3][j] = altitude_pre;

    if(data_filter_size < 10) data_filter_size++;
    data_filter_index = (data_filter_index + 1) % data_filter_max;

    // compute average of stored inputs
    pos_x = pos_y = heading = altitude = 0.0f;
    for(uint8_t i = 0; i < data_filter_size; i++) {
      pos_x += data_filter[0][i];
      pos_y += data_filter[1][i];
      heading += data_filter[2][i];
      altitude += data_filter[3][i];
    }

    pos_x /= data_filter_size;
    pos_y /= data_filter_size;
    heading /= data_filter_size;
    altitude /= data_filter_size;
    
    flag_data_received = 0;
  } else if(!enable_filtering) {
    pos_x = pos_x_pre;
    pos_y = pos_y_pre;
    heading = heading_pre;
    altitude = altitude_pre;
  }

  // TODO: more complex mode change logic
  if(flag_mode_change) {
    mode = mode_requested;
  }

  // output handling
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

      Serial.println("No track");
    }
    
    // direct manual mode
    for(uint8_t i = 0; i < 4; i++) {
      uint8_t val = 255 - (uint8_t)(inputs[i] * vscale);
      digitalPotWrite(channelmap[i], val);
    }
    
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

    float target_altitude = 60;

    // pid sensor inputs
    if(!flag_capture_lost) {
      float hdg_rad = (heading * 3.1415) / 180.0;
      
      ch1_input = altitude - target_altitude;
      ch2_input = pos_x;// * sin(hdg_rad) + pos_y * cos(hdg_rad);
      ch3_input = pos_y;// * sin(hdg_rad) + pos_x * cos(hdg_rad);
      ch4_input = heading;
    }

    // pid setpoints (targets)
    ch1_setpoint = 0;
    ch2_setpoint = 0;
    ch3_setpoint = 0;
    ch4_setpoint = 0;

    // crunch the pids
    ch1_pid.Compute();
    ch2_pid.Compute();
    ch3_pid.Compute();
    ch4_pid.Compute();

    // read out pid outputs
    control_inputs[0] = (uint8_t)constrain(round(ch1_output + inputs[0]), 0, 255);
    control_inputs[1] = (uint8_t)constrain(round(ch2_output + inputs[1]), 0, 255);
    control_inputs[2] = (uint8_t)constrain(round(ch3_output + inputs[2]), 0, 255);
    control_inputs[3] = (uint8_t)constrain(round(ch4_output + inputs[3]), 0, 255);

    // throttle cut override
    if(inputs[0] < 10) control_inputs[0] = inputs[0];
    
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
    Serial.println(altitude - target_altitude);
    Serial.print("CH1: ");
    Serial.println(ch1_output);

    if(flag_capture_lost)
      digitalWrite(LED_MODE, (millis() / 100 % 2) ? HIGH : LOW);
    else
      digitalWrite(LED_MODE, HIGH);
  }
}

void digitalPotWrite(int address, int value) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(address);
  SPI.transfer(value);
  digitalWrite(CS_PIN, HIGH);
}

