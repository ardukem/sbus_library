#include "sbus.h"
#include <Arduino.h>

void SbusDecoder::begin(unsigned long baudrate, uint8_t method, uint16_t period)
{
	for (byte i = 0; i<18; i++) 
	{
		_channels[i]      = 0;
	}

	_goodFrames         = 0;
	_lostFrames         = 0;
	_decoderErrorFrames = 0;
	_failsafe           = SBUS_FAILSAFE_INACTIVE;
	_period				= period;

	_serial.begin(baudrate, method);
}

void SbusDecoder::close()
{
	_serial.end();
}

void SbusDecoder::process() 
{
	static byte buffer[SBUS_PACKET_LENGTH];
	static byte buffer_index = 0;

	while (_serial.available())
	{
		byte rx = _serial.read();
		if (buffer_index == 0 && rx != SBUS_STARTBYTE) 
		{
			//incorrect start byte, out of sync
			_decoderErrorFrames++;
			continue;
			//return;
		}

		buffer[buffer_index++] = rx;

		if (buffer_index == SBUS_PACKET_LENGTH) 
		{
			buffer_index = 0;
			if (buffer[24] != SBUS_ENDBYTE) 
			{
				//incorrect end byte, out of sync
				_decoderErrorFrames++;
				continue;
				//return;
			}
			_goodFrames++;

			_channels[0]  = ((buffer[1]    |buffer[2]<<8)                 & 0x07FF);
			_channels[1]  = ((buffer[2]>>3 |buffer[3]<<5)                 & 0x07FF);
			_channels[2]  = ((buffer[3]>>6 |buffer[4]<<2 |buffer[5]<<10)  & 0x07FF);
			_channels[3]  = ((buffer[5]>>1 |buffer[6]<<7)                 & 0x07FF);
			_channels[4]  = ((buffer[6]>>4 |buffer[7]<<4)                 & 0x07FF);
			_channels[5]  = ((buffer[7]>>7 |buffer[8]<<1 |buffer[9]<<9)   & 0x07FF);
			_channels[6]  = ((buffer[9]>>2 |buffer[10]<<6)                & 0x07FF);
			_channels[7]  = ((buffer[10]>>5|buffer[11]<<3)                & 0x07FF);
			_channels[8]  = ((buffer[12]   |buffer[13]<<8)                & 0x07FF);
			_channels[9]  = ((buffer[13]>>3|buffer[14]<<5)                & 0x07FF);
			_channels[10] = ((buffer[14]>>6|buffer[15]<<2|buffer[16]<<10) & 0x07FF);
			_channels[11] = ((buffer[16]>>1|buffer[17]<<7)                & 0x07FF);
			_channels[12] = ((buffer[17]>>4|buffer[18]<<4)                & 0x07FF);
			_channels[13] = ((buffer[18]>>7|buffer[19]<<1|buffer[20]<<9)  & 0x07FF);
			_channels[14] = ((buffer[20]>>2|buffer[21]<<6)                & 0x07FF);
			_channels[15] = ((buffer[21]>>5|buffer[22]<<3)                & 0x07FF);

			((buffer[23])      & 0x0001) ? _channels[16] = 2047: _channels[16] = 0;
			((buffer[23] >> 1) & 0x0001) ? _channels[17] = 2047: _channels[17] = 0;

			if ((buffer[23] >> 3) & 0x0001) 
			{
				_failsafe = SBUS_FAILSAFE_ACTIVE;
			} 
			else 
			{
				_failsafe = SBUS_FAILSAFE_INACTIVE;
			}

			if ((buffer[23] >> 2) & 0x0001) 
			{
				_lostFrames++;
			}

			_lastGoodFrame = millis();
		}
	}
}

int SbusDecoder::getChannel(int channel) 
{
	if (channel < 1 or channel > 18) 
	{
		return 0;
	} 
	else 
	{
		return _channels[channel - 1];
	}
}

int SbusDecoder::getChannelRc(int channel)
{
	if (channel < 1 or channel > 18) 
	{
		return 0;
	} 
	else 
	{
		return map(_channels[channel - 1], SBUS_MIN_OFFSET, SBUS_MAX_OFFSET, PULSE_MIN, PULSE_MAX);
	}
}

int SbusDecoder::getNormalizedChannel(int channel) {
	if (channel < 1 or channel > 18) {
		return 0;
	} else {
		return (int) lround(_channels[channel - 1] / 9.92) - 100; //9.92 or 10.24?
	}
}

int SbusDecoder::getFailsafeStatus() {
	return _failsafe;
}

int SbusDecoder::getFrameLoss() {
	return (int) ((_lostFrames + _decoderErrorFrames) * 100 / (_goodFrames + _lostFrames + _decoderErrorFrames));
}

long SbusDecoder::getGoodFrames() {
	return _goodFrames;
}

long SbusDecoder::getLostFrames() {
	return _lostFrames;
}

long SbusDecoder::getDecoderErrorFrames() {
	return _decoderErrorFrames;
}

long long SbusDecoder::getLastTime() {
	return _lastGoodFrame;
}

//########################################################################################################################

void SbusEncoder::begin(unsigned long baudrate, uint8_t method, uint16_t period) 
{
	_period = period;
	_serial.begin(baudrate, method);
}

void SbusEncoder::preparePacket(int channels[], bool isSignalLoss, bool isFailsafe)
{
	static int output[SBUS_CHANNEL_NUMBER] = {0};

	/*
	* Map 1000-2000 with middle at 1500 chanel values to
	* 173-1811 with middle at 992 S.BUS protocol requires
	*/
	for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++)
	{
		output[i] = map(channels[i], PULSE_MIN, PULSE_MAX, SBUS_MIN_OFFSET, SBUS_MAX_OFFSET);
	}

	uint8_t stateByte = 0x00;

	if (isSignalLoss)
		stateByte |= SBUS_STATE_SIGNALLOSS;

	if (isFailsafe)
		stateByte |= SBUS_STATE_FAILSAFE;

	_packet[0] = SBUS_STARTBYTE; //Header

	_packet[1] = (uint8_t) (output[0] & 0x07FF);
	_packet[2] = (uint8_t) ((output[0] & 0x07FF)>>8 | (output[1] & 0x07FF)<<3);
	_packet[3] = (uint8_t) ((output[1] & 0x07FF)>>5 | (output[2] & 0x07FF)<<6);
	_packet[4] = (uint8_t) ((output[2] & 0x07FF)>>2);
	_packet[5] = (uint8_t) ((output[2] & 0x07FF)>>10 | (output[3] & 0x07FF)<<1);
	_packet[6] = (uint8_t) ((output[3] & 0x07FF)>>7 | (output[4] & 0x07FF)<<4);
	_packet[7] = (uint8_t) ((output[4] & 0x07FF)>>4 | (output[5] & 0x07FF)<<7);
	_packet[8] = (uint8_t) ((output[5] & 0x07FF)>>1);
	_packet[9] = (uint8_t) ((output[5] & 0x07FF)>>9 | (output[6] & 0x07FF)<<2);
	_packet[10] = (uint8_t) ((output[6] & 0x07FF)>>6 | (output[7] & 0x07FF)<<5);
	_packet[11] = (uint8_t) ((output[7] & 0x07FF)>>3);
	_packet[12] = (uint8_t) ((output[8] & 0x07FF));
	_packet[13] = (uint8_t) ((output[8] & 0x07FF)>>8 | (output[9] & 0x07FF)<<3);
	_packet[14] = (uint8_t) ((output[9] & 0x07FF)>>5 | (output[10] & 0x07FF)<<6);  
	_packet[15] = (uint8_t) ((output[10] & 0x07FF)>>2);
	_packet[16] = (uint8_t) ((output[10] & 0x07FF)>>10 | (output[11] & 0x07FF)<<1);
	_packet[17] = (uint8_t) ((output[11] & 0x07FF)>>7 | (output[12] & 0x07FF)<<4);
	_packet[18] = (uint8_t) ((output[12] & 0x07FF)>>4 | (output[13] & 0x07FF)<<7);
	_packet[19] = (uint8_t) ((output[13] & 0x07FF)>>1);
	_packet[20] = (uint8_t) ((output[13] & 0x07FF)>>9 | (output[14] & 0x07FF)<<2);
	_packet[21] = (uint8_t) ((output[14] & 0x07FF)>>6 | (output[15] & 0x07FF)<<5);
	_packet[22] = (uint8_t) ((output[15] & 0x07FF)>>3);

	_packet[23] = stateByte; //Flags byte
	_packet[24] = SBUS_ENDBYTE; //Footer
	
	
	//SbusEncoder::process();
}

void SbusEncoder::process(int channels[], bool isSignalLoss, bool isFailsafe)
{
	uint32_t now = millis();
	
	if (now >= _nextSendTime)
	{
		preparePacket(channels, isSignalLoss, isFailsafe);
		_serial.write(_packet, SBUS_PACKET_LENGTH);
		_nextSendTime = now + _period;
	}
}
