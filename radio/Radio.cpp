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

// a flag that a wireless packet has been received
boolean packetAvailable = false;

/* Handle interrupt from CC1101 (INT0) gdo0 on pin2 */
void cc1101signalsInterrupt(void){
// set the flag that a package is available
packetAvailable = true;
}

#define BUFFER_SIZE 512
uint8_t buffer[BUFFER_SIZE];
int buffer_front = 0;
int buffer_back = 0;

Radio::Radio() {
	byte _b;
	byte i;
	_syncWord = 199;
	_counter = 0;
	_chan = 0;
	CC1101 cc1101;
}


void Radio::init() {
	cc1101.init();
	byte SyncWord = get_syncWord();
	cc1101.setSyncWord(&SyncWord, false);
	cc1101.setCarrierFreq(CFREQ_433);
	cc1101.disableAddressCheck();
	Serial.print("CC1101_PARTNUM "); //cc1101=0
	Serial.println(cc1101.readReg(CC1101_PARTNUM, CC1101_STATUS_REGISTER));
	Serial.print("CC1101_VERSION "); //cc1101=4
	Serial.println(cc1101.readReg(CC1101_VERSION, CC1101_STATUS_REGISTER));
	Serial.print("CC1101_MARCSTATE ");
	Serial.println(cc1101.readReg(CC1101_MARCSTATE, CC1101_STATUS_REGISTER) & 0x1f);

	attachInterrupt(0, cc1101signalsInterrupt, FALLING);

	Serial.println("device initialized");
}

void Radio::update() {
	static long interval_start;
	static int interval_packets_received = 0;
	static int interval_packets_malformed = 0;
	static int total_packets_received = 0;
	static int total_packets_malformed = 0;
	static int interval_rssi = 0;
	static int interval_lqi = 0;
	
	uint8_t debug_packet = 0;
	
	if(millis() > interval_start + 1000) {
		Serial.println("--- LINK REPORT ---");
		
		Serial.print("[");
		Serial.print(millis() / 60000);
		Serial.print(":");
		Serial.print((millis() / 1000) % 60);
		Serial.print(":");
		Serial.print(millis() % 1000);
		Serial.println("]");
		
		Serial.print("Packets received: ");
		Serial.println(interval_packets_received);
		Serial.print("\tTotal: ");
		Serial.println(total_packets_received);
		
		Serial.print("Packet loss: ");
		Serial.print(
			100.0f - ((interval_packets_received - interval_packets_malformed * 1.0f) /
			interval_packets_received) * 100.0f
			);
		Serial.println("%");
		Serial.print("\tTotal: ");
		Serial.println(total_packets_malformed);
		
		interval_rssi /= interval_packets_received;
		interval_lqi /= interval_packets_received;
		
		Serial.print("Signal quality: ");
		Serial.println(interval_lqi);
		
		Serial.print("Signal strength: ");
		Serial.println(interval_rssi);
		
		interval_rssi = 0;
		interval_lqi = 0;
		interval_packets_received = 0;
		interval_packets_malformed = 0;
		interval_start = millis();
	}
	
	if(packetAvailable){
		//Serial.println("packet received");
		// Disable wireless reception interrupt
		detachInterrupt(0);

		// clear the flag
		packetAvailable = false;

		CCPACKET packet;

		if(cc1101.receiveData(&packet) > 0){
			
			// update interval stats
			interval_packets_received++;
			total_packets_received++;
			interval_rssi += ReadRSSI();
			interval_lqi += ReadLQI();
		
		  if(!packet.crc_ok) {
			//Serial.println("crc not ok");
			interval_packets_malformed++;
			total_packets_malformed++;
		  } else if(packet.length > 0){
			//Serial.print("packet: len ");
			//Serial.print(packet.length);
			//Serial.print(" data: ");
			for(int j=0; j<packet.length; j++){
				buffer[buffer_back] = packet.data[j];
				buffer_back = (buffer_back + 1) % BUFFER_SIZE;
				if(debug_packet) Serial.print((char)packet.data[j]);
				//Serial.print(" ");
			}
			if(debug_packet) Serial.println(" .");
		  }
		}
		// Enable wireless reception interrupt
		attachInterrupt(0, cc1101signalsInterrupt, FALLING);
	}
}

long Radio::available(){
	if(buffer_front > buffer_back) 
		return (buffer_front + BUFFER_SIZE) - buffer_back;
	else 
		return buffer_back - buffer_front;
}

uint8_t Radio::read(){
	uint8_t to_return = buffer[buffer_front];
	buffer_front = (buffer_front + 1) % BUFFER_SIZE;
	return to_return;
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

byte Radio::ReadLQI(){
	byte lqi=0;
	byte value=0;
	lqi=(cc1101.readReg(CC1101_LQI, CC1101_STATUS_REGISTER));
	value = 0x3F - (lqi & 0x3F);
	//Serial.print("CC1101_LQI ");
	//Serial.println(value);
	
	return value;
}

byte Radio::ReadRSSI()
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
  //Serial.print("CC1101_RSSI ");
  //Serial.println(value);
  
  return value;
}
