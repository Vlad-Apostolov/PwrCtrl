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
	for (;;) {
		while (_sleepyPi.rpiCurrent() >= _rpiShutdownCurrent);	// wait for RPi to shutdown
		powerDownPi(true);
		initRtc();

		_sleepyPi.setTimer1(eTB_MINUTE, _rpiSleepTime);
		_sleepyPi.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

		if (_rtcInterrupt) {
			_sleepyPi.enablePiPower(true);	// Power up RPi
			// Start I2C slave
			Wire.begin(ARDUINO_I2C_SLAVE_ADDRESS);
			Wire.onReceive(i2cTask);
		} else {	// WDT interrupt
			// TODO:
		}
	}
}

void MainTask::initRtc()
{
	// Start I2C master
	_sleepyPi.rtcInit(true);
    attachInterrupt(RTC_INTERRUPT_PIN, rtcInterrupt, FALLING);
}

void MainTask::rtcInterrupt()
{
	MainTask::instance()._rtcInterrupt = true;
}

void MainTask::i2cTask(int received)
{
	while (Wire.available())
		MainTask::instance().parseMessage(Wire.read());
}

void MainTask::powerDownPi(bool state)
{
	if (state) {
		_pduControl &= ~RPI_ON;
	} else {
		_pduControl |= RPI_ON;
	}
	setPdu();
}

void MainTask::parseMessage(char data)
{
	if (_messageIndex < MESSAGE_LENGHT) {	// message format "$XX,XX\0"
		_message[_messageIndex++] = data;
		if (data == 0) {
			char tag, value;
			if (sscanf(_message, "$%x,%x", tag, value) == 2) {
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

	if (_pduControl & RPI_ON)
		_sleepyPi.enablePiPower(true);
	else
		_sleepyPi.enablePiPower(false);
}

char MainTask::asciiToInt(unsigned char data)
{
	if (data < '0' || data > '9')
		return -1;
	else
		return (data - '0');
}

