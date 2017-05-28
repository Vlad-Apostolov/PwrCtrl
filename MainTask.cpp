/*
 * MainTask.cpp
 *
 *  Created on: 12/03/2017
 *      Author: Vlad
 */

#include "MemoryFree.h"
#include "Arduino.h"

#include "MainTask.h"

extern unsigned char wdtInterrupt;

#define SYSMON_IS_RUNNING	7	// PD7 - Handshake to show that the SysMon is running - Active High

MainTask& MainTask::instance()
{
	static MainTask *inst = NULL;
	if (!inst) {
		pinMode(SYSMON_IS_RUNNING, INPUT);
		Serial.begin(19200);
		Serial.println();
		Serial.println(F("MainTask started"));
		Serial.print(F("Free memory bytes: "));
		Serial.println(freeMemory());
		digitalWrite(RELAY1_PIN, HIGH); pinMode(RELAY1_PIN, OUTPUT);
		digitalWrite(RELAY2_PIN, HIGH); pinMode(RELAY2_PIN, OUTPUT);
		digitalWrite(RELAY3_PIN, HIGH); pinMode(RELAY3_PIN, OUTPUT);
		digitalWrite(RELAY4_PIN, HIGH); pinMode(RELAY4_PIN, OUTPUT);
		digitalWrite(RELAY5_PIN, HIGH); pinMode(RELAY5_PIN, OUTPUT);
		digitalWrite(RELAY6_PIN, HIGH); pinMode(RELAY6_PIN, OUTPUT);
		digitalWrite(RELAY7_PIN, HIGH); pinMode(RELAY7_PIN, OUTPUT);
		digitalWrite(YELLOW_LED_PIN, LOW); pinMode(YELLOW_LED_PIN, OUTPUT);	// yellow LED off
		inst = new MainTask();
	}
	return *inst;
}

void MainTask::run()
{
	startI2cSlave();
	Serial.println(F("Wait 40 seconds for the router to boot up and connect"));
	delay(40000);
	Serial.println(F("Power up RPi to get the configuration from the web"));
	powerUpPi();
	Serial.println(F("Wait for RPi to finish its work"));
	powerDownPi();
	initRtc();
	for (;;) {
		if (_rtcInterrupt || _wdtInterrupt) {
			_wdtInterrupt = false;
			_wdSeconds = 0;
			if (_rtcInterrupt) {
				_sleepyPi.ackTimer1();
				_rtcFailed = false;
				_rtcInterrupt = false;
			}
			_systemTime += 60 * (uint16_t)_rtcPeriodInMunutes;
			_upTime++;
			Serial.println();
			_rtcFailed ? Serial.print(F("WDT")) :  Serial.print(F("RTC"));
			Serial.print(F(" interrupt. "));
			Serial.print(F("Up time: "));
			Serial.println(_upTime);
			if (_upTime%_spiSleepTime == 0) {
				Serial.println(F("Reading solar charger"));
				readSolarCharger();
			}
			if (_upTime%_rpiSleepTime == 0) {
				powerUpPi();
				powerDownPi();
			}
		} else if (wdtInterrupt) {
			// watchdog timer interrupt
			wdtInterrupt = 0;
			_wdSeconds += 4;
			if (_wdSeconds >= 60) {
				if (_rtcFailed) {
					_wdtInterrupt = true;
					continue;
				}
				uint16_t rtcPeriodInSeconds = (60 + 6) * (uint16_t)_rtcPeriodInMunutes; // allow for +10% WD inaccuracy
				if (_wdSeconds > rtcPeriodInSeconds) {
					Serial.println(F("RTC didn't fire interrupt!"));
					_rtcFailed = true;
					_wdtInterrupt = true;
					continue;
				}
			}
		}
		Serial.flush();
		_sleepyPi.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
		delay(3);	// this is needed for the I2C slave to work in sleep mode
	}
}

void MainTask::startI2cSlave()
{
	Wire.begin(ARDUINO_I2C_SLAVE_ADDRESS);
	Wire.onReceive(i2cReceiver);
	Wire.onRequest(i2cTransmitter);
}

void MainTask::initRtc()
{
	// Start I2C master
	if (!_sleepyPi.rtcInit(true)) {
		Serial.println(F("rtcInit() failed!"));
		_rtcFailed = true;
		return;
	}
	Serial.println(F("Set RTC alarm in one minute"));
    attachInterrupt(RTC_INTERRUPT_PIN, rtcInterrupt, FALLING);
    _sleepyPi.setTimer1(eTB_MINUTE, _rtcPeriodInMunutes);
    delay(1000);
    _sleepyPi.ackTimer1();
	_rtcInterrupt = false;
	startI2cSlave();
}

void MainTask::rtcInterrupt()
{
	MainTask::instance()._rtcInterrupt = true;
	MainTask::instance()._wdtInterrupt = true;
	MainTask::instance()._rtcFailed = false;
}

void MainTask::i2cReceiver(int received)
{
	Serial.print(F("i2cReceiver "));
	Serial.println(received);
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
	Serial.print(F("i2cTransmitter "));
	Serial.println(solarChargerData->time);
	Wire.write((uint8_t*)solarChargerData, sizeof(SolarChargerData));
}

void MainTask::powerUpPi()
{
	Serial.println(F("Power up RPi"));
	_sleepyPi.enablePiPower(true);
	Serial.print(F("Waiting for RPi to power up at RPi current threshold "));
	Serial.print(RPI_POWER_UP_CURRENT);
	Serial.println(F(" mA"));
	uint16_t current;
	unsigned long powerUpTimeout = millis();
#define POWER_UP_TIMEOUT	3*60000	// 3 minutes
	do {
		current = _sleepyPi.rpiCurrent();
		Serial.print(F("RPi current: "));
		Serial.print(current);
		Serial.println(F("mA"));
		Serial.flush();
		delay(1000);
		if (digitalRead(SYSMON_IS_RUNNING))
			powerUpTimeout = millis();
		else if ((millis() - powerUpTimeout) > POWER_UP_TIMEOUT) {
			Serial.println(F("RPi didn't power up! Cycle RPi power and try again."));
			_sleepyPi.enablePiPower(false);
			delay(2000);
			_sleepyPi.enablePiPower(true);
			powerUpTimeout = millis();
		}
	} while (current < RPI_POWER_UP_CURRENT);
}

void MainTask::powerDownPi()
{
	Serial.print(F("Waiting for RPi to shutdown at RPi current threshold "));
	Serial.print(_rpiShutdownCurrent);
	Serial.println(F(" mA"));
	uint16_t current;
	unsigned long powerUpTimeout = millis();
#define POWER_DOWN_TIMEOUT	3*60000	// 3 minutes
	do {
		current = _sleepyPi.rpiCurrent();
		Serial.print(F("RPi current: "));
		Serial.print(current);
		Serial.println(F("mA"));
		Serial.flush();
		delay(1000);
		if (digitalRead(SYSMON_IS_RUNNING))
			powerUpTimeout = millis();
		else if ((millis() - powerUpTimeout) > POWER_UP_TIMEOUT) {
			Serial.print(F("RPi didn't power down! Force power down."));
			break;
		}
	} while (current >= _rpiShutdownCurrent);

	Serial.println(F("Power down RPi"));
	_sleepyPi.enablePiPower(false);
	delay(5000);		// make sure RPi loses the power before next power up
}

void MainTask::parseMessage(char data)
{
	if (_messageIndex < MESSAGE_LENGHT) {	// message format "$XX,XXXXXXXX\0"
		_message[_messageIndex++] = data;
		if (data == 0) {
			uint16_t tag;
			uint32_t value;
			if (sscanf(_message, "$%x,%lx", &tag, &value) == 2) {
				switch (tag) {
				case TAG_PDU_CONTROL:
					_pduControl = value;
					Serial.print(F("TAG_PDU_CONTROL "));
					Serial.println(_pduControl, HEX);
					setPdu();
					break;
				case TAG_RPI_SLEEP_TIME:
					_rpiSleepTime = (uint8_t)value;
					Serial.print(F("TAG_RPI_SLEEP_TIME "));
					Serial.println(_rpiSleepTime);
					break;
				case TAG_SPI_SLEEP_TIME:
					_spiSleepTime = (uint8_t)value;
					Serial.print(F("TAG_SPI_SLEEP_TIME "));
					Serial.println(_spiSleepTime);
					break;
				case TAG_SPI_SYSTEM_TIME:
					_systemTime = value;
					Serial.print(F("TAG_SPI_SYSTEM_TIME "));
					Serial.println(_systemTime);
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
	if (_pduControl & PDU_RELAY1_ON) {
		digitalWrite(RELAY1_PIN, LOW);
	} else {
		digitalWrite(RELAY1_PIN, HIGH);
	}
	if (_pduControl & PDU_RELAY2_ON) {
		digitalWrite(RELAY2_PIN, LOW);
	} else {
		digitalWrite(RELAY2_PIN, HIGH);
	}
	if (_pduControl & PDU_RELAY3_ON) {
		digitalWrite(RELAY3_PIN, LOW);
	} else {
		digitalWrite(RELAY3_PIN, HIGH);
	}
	if (_pduControl & PDU_RELAY4_ON) {
		digitalWrite(RELAY4_PIN, LOW);
	} else {
		digitalWrite(RELAY4_PIN, HIGH);
	}
	if (_pduControl & PDU_RELAY5_ON) {
		digitalWrite(RELAY5_PIN, LOW);
	} else {
		digitalWrite(RELAY5_PIN, HIGH);
	}
	if (_pduControl & PDU_RELAY6_ON) {
		digitalWrite(RELAY6_PIN, LOW);
	} else {
		digitalWrite(RELAY6_PIN, HIGH);
	}
	if (_pduControl & PDU_RELAY7_ON) {
		digitalWrite(RELAY7_PIN, LOW);
	} else {
		digitalWrite(RELAY7_PIN, HIGH);
	}
	if (_pduControl & PDU_EXTERNAL_POWER_ON) {
		SleepyPi.enableExtPower(true);
	} else {
		SleepyPi.enableExtPower(false);
	}
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
	_solarCharger.connect();
	solarChargerData.energyYieldToday = _solarCharger.getEnergyYieldToday();
	solarChargerData.panelPower = _solarCharger.getPanelPower();
	HistoryDayRecord* historyDayRecord = _solarCharger.getHistoryDayRecordy(0);
	if (historyDayRecord)
		solarChargerData.consumedToday = historyDayRecord->getConsumed();
	else
		solarChargerData.consumedToday = 0;
	historyDayRecord = _solarCharger.getHistoryDayRecordy(1);
	if (historyDayRecord)
		solarChargerData.consumedYesterday = historyDayRecord->getConsumed();
	else
		solarChargerData.consumedYesterday = 0;
	solarChargerData.chargerVoltage = _solarCharger.getChargerVoltage();
	solarChargerData.chargerCurrent = _solarCharger.getChargerCurrent();
	solarChargerData.loadCurrent = _solarCharger.getLoadCurrent();
	solarChargerData.panelVoltage = _solarCharger.getPanelVoltage();
	solarChargerData.deviceState = _solarCharger.getDeviceState();
	solarChargerData.time = _systemTime;
	solarChargerData.cpuTemperature = getCpuTemperature();
	_solarCharger.disconnect();
	Serial.println();
	Serial.print(F("energyYieldToday "));
	Serial.println(solarChargerData.energyYieldToday);
	Serial.print(F("panelPower "));
	Serial.println(solarChargerData.panelPower);
	Serial.print(F("consumedToday "));
	Serial.println(solarChargerData.consumedToday);
	Serial.print(F("consumedYesterday "));
	Serial.println(solarChargerData.consumedYesterday);
	Serial.print(F("chargerVoltage "));
	Serial.println(solarChargerData.chargerVoltage);
	Serial.print(F("chargerCurrent "));
	Serial.println(solarChargerData.chargerCurrent);
	Serial.print(F("loadCurrent "));
	Serial.println(solarChargerData.loadCurrent);
	Serial.print(F("panelVoltage "));
	Serial.println(solarChargerData.panelVoltage);
	Serial.print(F("cpuTemperature "));
	Serial.println(solarChargerData.cpuTemperature);
	Serial.print(F("Device state "));
	Serial.println(solarChargerData.deviceState);
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

