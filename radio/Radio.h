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
    void read_rpi_data();
	long parseInt();
	float parseFloat();
	long available();
	int read();
	byte get_b();
	byte get_i();
	byte get_syncWord();
	long get_counter();
	byte get_chan();
	void ReadLQI();
	void ReadRSSI();
  private:
    byte _b;
    byte _i;
    byte _syncWord;
    long _counter;
    byte _chan;
	CC1101 cc1101;
};

#endif
