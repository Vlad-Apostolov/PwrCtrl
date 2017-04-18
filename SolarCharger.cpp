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
	//Serial.println(__FUNCTION__);
	return readVictron(":7D5ED008C\n", _chargerVoltage);
}

uint16_t SolarCharger::getChargerCurrent()
{
	//Serial.println(__FUNCTION__);
	return readVictron(":7D7ED008A\n", _chargerCurrent);
}

uint16_t SolarCharger::getPowerYieldToday()
{
	//Serial.println(__FUNCTION__);
	return readVictron(":7D3ED008E\n", _powerYieldToday);
}

int16_t SolarCharger::getChargerTemperature()
{
	//Serial.println(__FUNCTION__);
	return readVictron(":7DBED0086\n", _chargerTemperature);
}

uint16_t SolarCharger::getLoadCurrent()
{
	//Serial.println(__FUNCTION__);
	return readVictron(":7ADED00B4\n", _loadCurrent);
}

uint16_t SolarCharger::getPanelVoltage()
{
	//Serial.println(__FUNCTION__);
	return readVictron(":7BBED00A6\n", _panelVoltage);
}

uint16_t SolarCharger::getPanelCurrent()
{
	//Serial.println(__FUNCTION__);
	return readVictron(":7BDED00A4\n", _panelCurrent);
}

uint32_t SolarCharger::getPanelPower()
{
	//Serial.println(__FUNCTION__);
	return readVictron(":7BCED00A5\n", _panelPower);
}

uint16_t SolarCharger::readVictron(const char* command, uint16_t& result)
{
	if (victronSend(command))
		return result;
	return INVALID_RESULT;
}

int16_t SolarCharger::readVictron(const char* command, int16_t& result)
{
	if (victronSend(command))
		return result;
	return INVALID_RESULT;
}

uint32_t SolarCharger::readVictron(const char* command, uint32_t& result)
{
	if (victronSend(command))
		return result;
	return INVALID_RESULT;
}

bool SolarCharger::victronSend(const char* command)
{
#define RETRY_COUNT	3
	uint8_t retry = RETRY_COUNT;
	while (retry) {
		_comPort.write(command);
		if (processReply())
			return true;
		retry--;
	}
	Serial.println("Failed...");
	return false;
}

bool SolarCharger::processReply()
{
#define READ_TIMEOUT	50
	bool result = false;
	int data = -1;
	_comPort.listen();
	_comPort.flush();
	unsigned long startMillis = millis();
	while (!result && ((millis() - startMillis) < READ_TIMEOUT)) {
		while (_comPort.available()) {
			data = _comPort.read();
			if (data < 0)
				break;
			result = processSerialData(data);
		}
	}
	Serial.flush();

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
			_firstNibble = INVALID_DATA;
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
		uint16_t registerId = _command[1];
		registerId <<= 8;
		registerId |= _command[2];
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
	if (data >= '0' && data <= '9')
		return (data - '0');
	if (data >= 'A' && data <= 'F')
		return (data - ('A' - 10));
	return INVALID_DATA;
}

