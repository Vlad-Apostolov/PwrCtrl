/*
 * SolarCharger.cpp
 *
 *  Created on: 18/03/2017
 *      Author: Vlad
 */

#include "Arduino.h"
#include "SolarCharger.h"

uint16_t SolarCharger::getChargerVoltage()
{
	_comPort.write(":7D5ED008C\n");
	_chargerVoltage = INVALID_RESULT;
	if (processReply())
		return _chargerVoltage;

	return INVALID_RESULT;
}

uint16_t SolarCharger::getChargerCurrent()
{
	_comPort.write(":7D7ED008A\n");
	_chargerCurrent = INVALID_RESULT;
	if (processReply())
		return _chargerCurrent;

	return INVALID_RESULT;
}

uint16_t SolarCharger::getPowerYieldToday()
{
	_comPort.write(":7D3ED008E\n");
	_powerYieldToday = INVALID_RESULT;
	if (processReply())
		return _powerYieldToday;

	return INVALID_RESULT;
}

int16_t SolarCharger::getChargerTemperature()
{
	_comPort.write(":7DBED0086\n");
	_chargerTemperature = INVALID_RESULT;
	if (processReply())
		return _chargerTemperature;

	return INVALID_RESULT;
}

uint16_t SolarCharger::getLoadVoltage()
{
	_comPort.write(":7ACED00B5\n");
	_loadVoltage = INVALID_RESULT;
	if (processReply())
		return _loadVoltage;

	return INVALID_RESULT;
}

uint16_t SolarCharger::getLoadCurrent()
{
	_comPort.write(":7ADED00B4\n");
	_loadCurrent = INVALID_RESULT;
	if (processReply())
		return _loadCurrent;

	return INVALID_RESULT;
}

uint16_t SolarCharger::getPanelVoltage()
{
	_comPort.write(":7ACED00B5\n");
	_panelVoltage = INVALID_RESULT;
	if (processReply())
		return _panelVoltage;

	return INVALID_RESULT;
}

uint16_t SolarCharger::getPanelCurrent()
{
	_comPort.write(":7ADED00B4\n");
	_panelCurrent = INVALID_RESULT;
	if (processReply())
		return _panelCurrent;

	return INVALID_RESULT;
}

uint32_t SolarCharger::getPanelPower()
{
	_comPort.write(":7ADED00B4\n");
	_panelPower = INVALID_RESULT;
	if (processReply())
		return _panelPower;

	return INVALID_RESULT;
}

bool SolarCharger::processReply()
{
#define READ_TIMEOUT	50
	bool result = false;
	_comPort.listen();
	_comPort.flush();
	unsigned long startMillis = millis();
	while (!result && ((millis() - startMillis) < READ_TIMEOUT)) {
		while (_comPort.available()) {
			int data = _comPort.read();
			if (data < 0)
				break;
			result = processSerialData(data);
		}
	}

	return result;
}

bool SolarCharger::processSerialData(uint8_t data)
{
	bool result = false;
	switch (_commandState) {
	case CMD_HEADER:
		if (data == ':') {
			_commandState = CMD_ID;
			_commandSize = 0;
			_firstNibble = INVALID_DATA;
		}
		break;
	case CMD_ID:
	{
		uint8_t commandId = hexToBin(data);
		if (commandId == INVALID_DATA) {
			_commandState = CMD_HEADER;
			break;
		}
		_command[_commandSize++] = commandId;
		_commandState = CMD_BODY;
		break;
	}
	case CMD_BODY:
		if (_commandSize < MAX_COMMAND_SIZE) {
			if (data == '\n') {
				if (checkSumPassed())
					result = parseCommand();
				_commandState = CMD_HEADER;
				break;
			}
			uint8_t nibble = hexToBin(data);
			if (nibble == INVALID_DATA) {
				_commandState = CMD_HEADER;
				break;
			}
			if (_firstNibble == INVALID_DATA) {
				_firstNibble = nibble;
				break;
			}
			_command[_commandSize++] = (_firstNibble<<4) | nibble;
		} else
			_commandState = CMD_HEADER;
		break;
	default:
		_commandState = CMD_HEADER;
	}

	return result;
}

bool SolarCharger::parseCommand()
{
	bool result = false;
	// expects something like this 7 F0ED 00 9600 DB
	uint8_t commandId = _command[0];
	switch (commandId) {
	case CID_GET:
	{
		uint8_t flags = _command[3];
		if (flags != 0) {
			parseGetSetFlags(flags);
			break;
		}
		uint16_t registerId = *((uint16_t*)&_command[1]);
		uint32_t registerData = 0;
		for (uint8_t i = _commandSize - 2; i >= 4; i--) {
			registerData <<= 8;
			registerData |= _command[i];
		}
		switch (registerId) {
		case CR_YIELD_TODAY:
			_powerYieldToday = registerData;
			result = true;
			break;
		case CR_CURRENT:
			_chargerCurrent = registerData;
			result = true;
			break;
		case CR_VOLTAGE:
			_chargerVoltage = registerData;
			result = true;
			break;
		case CR_INTERNAL_TEMPERATURE:
			_chargerTemperature = registerData;
			result = true;
			break;
		case LOR_CURRENT:
			_loadCurrent = registerData;
			result = true;
			break;
		case LOR_VOLTAGE:
			_loadVoltage = registerData;
			result = true;
			break;
		case SPR_CURRENT:
			_panelCurrent = registerData;
			result = true;
			break;
		case SPR_VOLTAGE:
			_panelVoltage = registerData;
			result = true;
			break;
		case SPR_POWER:
			_panelPower = registerData;
			result = true;
			break;
		default:
			break;
		}
		break;
	}
	default:
		break;
	}

	return result;
}

void SolarCharger::parseGetSetFlags(uint8_t flags)
{
	switch(flags) {
	case GSF_UNKNOWN_ID:
		Serial.println("GSF_UNKNOWN_ID");
		break;
	case GSF_NOT_SUPPORTED:
		Serial.println("GSF_NOT_SUPPORTED");
		break;
	case GSF_PARAMETER_ERROR:
		Serial.println("GSF_PARAMETER_ERROR");
		break;
	default:
		Serial.println("Unexpected Set/Get flags");
		break;
	}
}

bool SolarCharger::checkSumPassed()
{
	uint8_t checkSum = 0;
	for (uint8_t i = 0; i < _commandSize; i++)
		checkSum += _command[i];
	if (checkSum == 0x55)
		return true;
	return false;
}

uint8_t SolarCharger::hexToBin(uint8_t data)
{
	if (data >= '0' && data >= '9')
		return (data - '0');
	if (data >= 'A' && data >= 'F')
		return (data - ('A' - 10));
	return INVALID_DATA;
}

