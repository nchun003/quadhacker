#include "EEPROM.h"
#include "cc1101.h"
#include "Radio.h"

/* Radio::Radio(byte b, byte i, byte syncWord, long counter, byte chan){
	_b = b;
	_i = i;
	_syncWord = syncWord;
	_counter = counter;
	_chan = chan;
} */

Radio::Radio() {
	byte _b;
	byte i;
	_syncWord = 199;
	_counter = 0;
	_chan = 0;
	CC1101 cc1101;
}

void Radio::read_rpi_data() {
  int incomingByte = 0;
  if(Serial.available() > 0){
    incomingByte = Serial.read();
    Serial.println(incomingByte, DEC);
  }
}

long Radio::parseInt(){
	return Serial.parseInt();
}

float Radio::parseFloat(){
	return Serial.parseFloat();
}

long Radio::available(){
	return Serial.available();
}

int Radio::read(){
	return Serial.read();
}

byte Radio::get_b(){
	return _b;
}

byte Radio::get_i(){
	return _i;
}

byte Radio::get_syncWord(){
	return _syncWord;
}

long Radio::get_counter(){
	return _counter;
}

byte Radio::get_chan(){
	return _chan;
}

void Radio::ReadLQI(){
	byte lqi=0;
	byte value=0;
	lqi=(cc1101.readReg(CC1101_LQI, CC1101_STATUS_REGISTER));
	value = 0x3F - (lqi & 0x3F);
	Serial.print("CC1101_LQI ");
	Serial.println(value);
}

void Radio::ReadRSSI()
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
