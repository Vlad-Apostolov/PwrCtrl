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
	if (!inst) {
		Serial.begin(19200);
		inst = new MainTask();
	}
	return *inst;
}

void MainTask::run()
{
	while (_sleepyPi.rpiCurrent() >= _rpiShutdownCurrent);	// wait for RPi to shutdown
	powerDownPi(true);
	initRtc();
	for (;;) {
		if (_rtcInterrupt) {
			initRtc();
			_sleepTime++;
			if (_sleepTime%_spiSleepTime == 0)
				readSolarCharger();
			if (_sleepTime%_rpiSleepTime == 0) {
				powerDownPi(false);
				// Start I2C slave
				Wire.begin(ARDUINO_I2C_SLAVE_ADDRESS);
				Wire.onReceive(i2cReceiver);
				Wire.onRequest(i2cTransmitter);
				while (_sleepyPi.rpiCurrent() >= _rpiShutdownCurrent);	// wait for RPi to shutdown
				powerDownPi(true);
			}
		}
		_sleepyPi.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
	}
}

void MainTask::initRtc()
{
    attachInterrupt(RTC_INTERRUPT_PIN, rtcInterrupt, FALLING);
	// Start I2C master
	_sleepyPi.rtcInit(true);
	_sleepyPi.setAlarm(60);
}

void MainTask::rtcInterrupt()
{
	MainTask::instance()._rtcInterrupt = true;
}

void MainTask::i2cReceiver(int received)
{
	while (Wire.available())
		MainTask::instance().parseMessage(Wire.read());
}

void MainTask::i2cTransmitter()
{
	SolarChargerData* solarChargerData = MainTask::instance().nextSolarChargerDataRead();
	if (solarChargerData == NULL) {
		SolarChargerData dummy;
		solarChargerData = &dummy;
		memset(solarChargerData, 0, sizeof(SolarChargerData));
	}
	Wire.write((uint8_t*)solarChargerData, sizeof(SolarChargerData));
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
					_rpiSleepTime = value;
					break;
				case TAG_SPI_SLEEP_TIME:
					_spiSleepTime = value;
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

void MainTask::readSolarCharger()
{
	SolarChargerData& solarChargerData = nextSolarChargerDataWrite();
	solarChargerData.chargerVoltage = _solarCharger.getChargerVoltage();
	solarChargerData.chargerCurrent = _solarCharger.getChargerCurrent();
	solarChargerData.chargerPowerToday = _solarCharger.getPowerYieldToday();
	solarChargerData.loadVoltage = _solarCharger.getLoadVoltage();
	solarChargerData.loadCurrent = _solarCharger.getLoadCurrent();
	solarChargerData.panelVoltage = _solarCharger.getPanelVoltage();
	solarChargerData.panelCurrent = _solarCharger.getPanelCurrent();
	solarChargerData.panelPower = _solarCharger.getPanelPower();
	solarChargerData.time = _sleepyPi.readTime().unixtime();
	solarChargerData.cpuTemperature = getCpuTemperature();
}

MainTask::SolarChargerData& MainTask::nextSolarChargerDataWrite()
{
	uint8_t current = _solarChargerDataWrite++;
	if (_solarChargerDataWrite >= MAX_SOLAR_CHARGER_DATA)
		_solarChargerDataWrite = 0;
	if (_solarChargerDataWrite == _solarChargerDataRead) {
		// Overflow! Drop the oldest data.
		_solarChargerDataRead++;
		if (_solarChargerDataRead >= MAX_SOLAR_CHARGER_DATA)
			_solarChargerDataRead = 0;
	}
	return _solarChargerData[current];
}

MainTask::SolarChargerData* MainTask::nextSolarChargerDataRead()
{
	if (_solarChargerDataRead == _solarChargerDataWrite)
		return NULL;
	uint8_t current = _solarChargerDataRead++;
	if (_solarChargerDataRead >= MAX_SOLAR_CHARGER_DATA)
		_solarChargerDataRead = 0;
	return &_solarChargerData[current];
}

// code tacken from http://playground.arduino.cc/Main/InternalTemperatureSensor
int8_t MainTask::getCpuTemperature()
{
	unsigned int wADC;
	double t;

	// The internal temperature has to be used
	// with the internal reference of 1.1V.
	// Channel 8 can not be selected with
	// the analogRead function yet.

	// Set the internal reference and mux.
	ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
	ADCSRA |= _BV(ADEN);  // enable the ADC

	delay(20);            // wait for voltages to become stable.

	ADCSRA |= _BV(ADSC);  // Start the ADC

	// Detect end-of-conversion
	while (bit_is_set(ADCSRA,ADSC));

	// Reading register "ADCW" takes care of how to read ADCL and ADCH.
	wADC = ADCW;

	// The offset of 324.31 could be wrong. It is just an indication.
	t = (wADC - 324.31 ) / 1.22;

	// The returned temperature is in degrees Celsius.
	return round(t);
}

