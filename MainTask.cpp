/*
 * MainTask.cpp
 *
 *  Created on: 12/03/2017
 *      Author: Vlad
 */

#include "MainTask.h"
#include "Arduino.h"

MainTask& MainTask::instance()
{
	static MainTask *inst = NULL;
	if (!inst)
		inst = new MainTask();
	return *inst;
}

void MainTask::run()
{
	initRtc();
	for (;;) {
		_sleepyPi.setTimer1(eTB_MINUTE, _rpiSleepTime);
		_sleepyPi.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

		if (_rtcInterrupt) {
			_sleepyPi.enablePiPower(true);	// Power up RPi
			// Start I2C slave
			Wire.begin(ARDUINO_I2C_SLAVE_ADDRESS);
			Wire.onReceive(i2cInterrupt);

			float piCurrent = 0.0;
			while ((piCurrent = _sleepyPi.rpiCurrent()) >= _rpiShutdownCurrent);	// wait for RPi to shutdown
			_sleepyPi.enablePiPower(false);	// Power down RPi
			initRtc();
		} else {	// WDT interrupt
			// TODO:
		}
	}
}

void MainTask::initRtc()
{
	_sleepyPi.rtcClearInterrupts();
    attachInterrupt(RTC_INTERRUPT_PIN, rtcInterrupt, FALLING);
}

void MainTask::rtcInterrupt()
{
	MainTask::instance()._rtcInterrupt = true;
}

void MainTask::i2cInterrupt(int received)
{
	while (Wire.available())
		MainTask::instance().processMessage(Wire.read());
}

void MainTask::processMessage(char data)
{
	char result;

	switch(_messageState) {
	case MSG_HEADER:
		if (data == '$')
			_messageState = MSG_LENGTH_1;
		break;
	case MSG_LENGTH_1:
		result = asciiToInt(data);
		if (result < 0)
			_messageState = MSG_HEADER;
		else {
			_messageState = MSG_LENGTH_1;
			_messageLength = result * 10;
		}
		break;
	case MSG_LENGTH_2:
		result = asciiToInt(data);
		if (result < 0)
			_messageState = MSG_HEADER;
		else {
			_messageLength += result;
			_messageIndex = 0;
			if (_messageLength > MAX_MESSAGE_LENGHT)
				_messageState = MSG_HEADER;
			else
				_messageState = MSG_BODY;
		}
		break;
	case MSG_BODY:
		if (_messageIndex < _messageLength) {
			_message[_messageIndex] = data;
			_messageIndex++;
			if (_messageIndex >= _messageLength) {
				if (data == 0)
					parseMessage();
				_messageState = MSG_HEADER;
			}
		} else
			_messageState = MSG_HEADER;
		break;
	}
}

void MainTask::parseMessage()
{
	char tag, value;
	sscanf(_message, "[%d,%d", tag, value);

	switch (tag) {
	case TAG_PDU_CONTROL:
		// TODO: use value to set PDU
		break;
	case TAG_RPI_SLEEP_TIME:
		break;
	}
}

char MainTask::asciiToInt(unsigned char data)
{
	if (data < '0' || data > '9')
		return -1;
	else
		return (data - '0');
}

