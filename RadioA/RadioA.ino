#include "EEPROM.h"
#include "cc1101.h"
//MEGA

CC1101 cc1101;

// The LED is wired to the Arduino Output 4 (physical panStamp pin 19)
//#define LEDOUTPUT 7
#define LEDOUTPUT 13

//D13, 10-12, 2
// counter to get increment in each loop
byte counter;
byte b;
//byte syncWord = 199;
byte syncWord[2] = {199, 0};

void blinker(){
  digitalWrite(LEDOUTPUT, HIGH);
  delay(100);
  digitalWrite(LEDOUTPUT, LOW);
  delay(100);
}

void setLED(int a) { digitalWrite(LEDOUTPUT, a ? 0 : 1); }

void setup()
{
  Serial.begin(115200);
  Serial.println("start");
  
  // setup the blinker output
  pinMode(LEDOUTPUT, OUTPUT);
  digitalWrite(LEDOUTPUT, LOW);
  
  // blink once to signal the setup
  blinker();
  
//   reset the counter
  counter=0;
  Serial.println("initializing...");
  // initialize the RF Chip
  cc1101.init();
  Serial.println("initialized");
  //cc1101.setSyncWord(&syncWord, false);
  cc1101.setSyncWord(syncWord, false);
  cc1101.setCarrierFreq(CFREQ_915);
  cc1101.disableAddressCheck();
  //cc1101.setTxPowerAmp(PA_LowPower);
  
  delay(1000);
  
  Serial.print("CC1101_PARTNUM "); //cc1101=0
  Serial.println(cc1101.readReg(CC1101_PARTNUM, CC1101_STATUS_REGISTER));
  Serial.print("CC1101_VERSION "); //cc1101=4
  Serial.println(cc1101.readReg(CC1101_VERSION, CC1101_STATUS_REGISTER));
  Serial.print("CC1101_MARCSTATE ");
  Serial.println(cc1101.readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1f);
  
  Serial.println("device initialized");
}


char data_buffer[64];
byte char_data_buff;
uint8_t data_buffer_index = 0;
uint8_t data_buffer_ready = 0;

// Read RPI values
void read_data() {
  int incomingByte = 0;
  if(data_buffer_ready) return;
  while(Serial.available() > 0){
    incomingByte = Serial.read();
    // Serial.println(incomingByte, DEC);
    // Serial read data from pi
    if(incomingByte == 'S') {
      data_buffer[0] = 'S';
      data_buffer_index = 1;
    } else {
      data_buffer[data_buffer_index] = incomingByte;
      data_buffer_index++;
    }
    
    if(incomingByte == 'E') {
      data_buffer_ready = 1;
      break;
    }
  }
}
 
void send_data() {
 if(data_buffer_ready){
    CCPACKET data;
    data.length=data_buffer_index;

    setLED(1);
  
    for(int i = 0; i<data_buffer_index; ++i){
      data.data[i] = data_buffer[i];
    }
    if(cc1101.sendData(data)){
      //Serial.println("sent ok :)");
    }else{
      //Serial.println("sent failed :(");
    }

    setLED(0);
    data_buffer_ready = 0;
    data_buffer_index = 0;
 }
}

void ReadLQI()
{
byte lqi=0;
byte value=0;
lqi=(cc1101.readReg(CC1101_LQI, CC1101_STATUS_REGISTER));
value = 0x3F - (lqi & 0x3F);
Serial.print("CC1101_LQI ");
Serial.println(value);
}

void ReadRSSI()
{
  byte rssi=0;
  byte value=0;
  
  rssi=(cc1101.readReg(CC1101_RSSI, CC1101_STATUS_REGISTER));
  
  if (rssi >= 128)
  {
    value = 255 - rssi;
    value /= 2;
    value += 74;
  }
  else
  {
    value = rssi/2;
    value += 74;
  }
  Serial.print("CC1101_RSSI ");
  Serial.println(value);
}

void loop()
{
  read_data();
  send_data();
  delayMicroseconds(100);
}
