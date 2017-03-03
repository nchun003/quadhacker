#ifndef Radio_h
#define Radio_h
#include "EEPROM.h"
#include "Arduino.h"
#include "cc1101.h"

class Radio
{
  public:
	//Radio(byte b, byte i, byte syncWord, long counter, byte chan);
	Radio();
	void init();
    void update();
	long available();
	uint8_t read();
	byte get_b();
	byte get_i();
	byte get_syncWord();
	long get_counter();
	byte get_chan();
	byte ReadLQI();
	byte ReadRSSI();
	CC1101 cc1101;
  private:
    byte _b;
    byte _i;
    byte _syncWord;
    long _counter;
    byte _chan;
};

#endif
