#ifndef SBUS_LIBRARY_h
#define SBUS_LIBRARY_h

#include "Arduino.h"

#define SBUS_FAILSAFE_INACTIVE 		0
#define SBUS_FAILSAFE_ACTIVE   		1
#define SBUS_STARTBYTE         		0x0f
#define SBUS_ENDBYTE           		0x00

#define SBUS_MIN_OFFSET 			202	//173
#define SBUS_MID_OFFSET 			992
#define SBUS_MAX_OFFSET 			1802 //1811

#define PULSE_MIN 					1000
#define PULSE_MID 					1500
#define PULSE_MAX 					2000

//

#define SBUS_PACKET_LENGTH 			25
#define SBUS_CHANNEL_NUMBER 		18
#define SBUS_STATE_FAILSAFE 		0x08
#define SBUS_STATE_SIGNALLOSS 		0x04
#define SBUS_PERIOD_MILLIS 			15 //ms

class SbusDecoder 
{
	public:
		
		SbusDecoder(HardwareSerial & serial) : _serial (serial) {}
		
		void begin(unsigned long baudrate, uint8_t method, uint16_t period);	//period is used here only for checking arriving rate and raise a flag
		void begin() {begin(100000, SERIAL_8E2, SBUS_PERIOD_MILLIS);}	//this is the sbus default standard.
		void process();
		void close();
		bool isRateValis() {return (millis() - _lastGoodFrame <= _period);}
		unsigned long timeFromLastGoodFrame() {if (getLastTime() == 0) return 300000; else return millis() - getLastTime();}
		
		int getChannel(int channel);	//returns value between 173 to 1811
		int getChannelRc(int channel);	//returns value between 1000 to 2000
		int getNormalizedChannel(int channel);
		int getFailsafeStatus();
		int getFrameLoss();
		long getGoodFrames();
		long getLostFrames();
		long getDecoderErrorFrames();
		long long getLastTime();
		
	private:
		
		HardwareSerial & _serial;
		int _channels[18];
		int _failsafe;
		long _goodFrames = 0;
		long _lostFrames;
		long _decoderErrorFrames;
		long long _lastGoodFrame = 0;
		uint16_t _period;
};

class SbusEncoder 
{
	public:
		
		SbusEncoder(HardwareSerial & serial) : _serial (serial) {}
		
		void begin(unsigned long baudrate, uint8_t method, uint16_t period);
		void begin() {begin(100000, SERIAL_8E2, SBUS_PERIOD_MILLIS);}
		void process(int channels[], bool isSignalLoss, bool isFailsafe);
		
	private:
		
		void preparePacket(int channels[], bool isSignalLoss, bool isFailsafe);
		
		HardwareSerial & _serial;
		uint8_t _packet[SBUS_PACKET_LENGTH];
		uint32_t _nextSendTime = 0;	//last send time
		uint16_t _period;
		
};

#endif
