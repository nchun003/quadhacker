#include "EEPROM.h"
//#include "cc1101.h"
#include "Radio.h"

Radio::Radio(byte b, byte i, byte syncWord, long counter, byte chan){
	_b = b;
	_i = i;
	_syncWord = syncWord;
	_counter = counter;
	_chan = chan;
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