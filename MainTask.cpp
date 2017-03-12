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
	if (_messageIndex < MAX_MESSAGE_LENGHT) {
		_message[_messageIndex++] = data;
		if (data == 0) {
			char tag, value;
			if (sscanf(_message, "$%d,%d", tag, value) == 2) {
				switch (tag) {
				case TAG_PDU_CONTROL:
					_pduControl = value;
					setPdu();
					break;
				case TAG_RPI_SLEEP_TIME:
					break;
				}
			}
			_messageIndex = 0;
		}
	} else
		_messageIndex = 0;
}

void MainTask::setPdu()
{
	if (_pduControl & PDU_ROUTER_ON)
		  SleepyPi.enableExtPower(true);
	else
		  SleepyPi.enableExtPower(false);
}

char MainTask::asciiToInt(unsigned char data)
{
	if (data < '0' || data > '9')
		return -1;
	else
		return (data - '0');
}

