/*
 * SolarCharger.h
 *
 *  Created on: 18/03/2017
 *      Author: Vlad
 */

#ifndef SOLARCHARGER_H_
#define SOLARCHARGER_H_

#include <SoftwareSerial.h>

#include "HistoryDayRecord.h"

class SolarCharger {
public:
#define INVALID_DATA			0xFF
#define INVALID_RESULT			0
	SolarCharger(uint8_t rxPin, uint8_t txPin) :
			_comPort(rxPin, txPin),
			_commandState(CMD_HEADER),
			_commandSize(0),
			_firstNibble(INVALID_DATA),
			_energyYieldToday(INVALID_RESULT),
			_chargerCurrent(INVALID_RESULT),
			_chargerVoltage(INVALID_RESULT),
			_chargerTemperature(INVALID_RESULT),
			_loadCurrent(INVALID_RESULT),
			_panelCurrent(INVALID_RESULT),
			_panelVoltage(INVALID_RESULT),
			_panelPower(INVALID_RESULT),
			_deviceState(0)
	{
	}
	void connect() { _comPort.begin(19200); }
	void disconnect() { _comPort.end(); }
	uint16_t getChargerVoltage();
	uint16_t getChargerCurrent();
	uint16_t getEnergyYieldToday();
	int16_t getChargerTemperature();
	uint16_t getLoadVoltage();
	uint16_t getLoadCurrent();
	uint16_t getPanelVoltage();
	uint16_t getPanelCurrent();
	uint32_t getPanelPower();
	uint16_t getDeviceState();
	HistoryDayRecord* getHistoryDayRecordy(uint8_t day);

	virtual ~SolarCharger() {}

private:
#define MAX_COMMAND_SIZE 		100
	enum CommandState
	{
		CMD_HEADER,
		CMD_ID,
		CMD_BODY
	};

	enum CommandId {
		CID_DONE = 0,
		CID_UNKNOWN = 3,
		CID_ERROR = 4,
		CID_PING = 5,
		CID_GET = 7,
		CID_SET = 8
	};
	enum GetSetFlags {
		GSF_UNKNOWN_ID = 0x01,
		GSF_NOT_SUPPORTED = 0x02,
		GSF_PARAMETER_ERROR
	};

	enum ChargerRegister {
		CR_MAXIMUM_CURRENT = 0xDFED,
		CR_SYSTEM_YIELD = 0xDDED,
		CR_USER_YIELD = 0xDCED,
		CR_INTERNAL_TEMPERATURE = 0xDBED,
		CR_ERROR_CODE = 0xDAED,
		CR_CURRENT = 0xD7ED,
		CR_VOLTAGE = 0xD5ED,
		CR_STATE_INFO = 0xD4ED,
		CR_ENERGY_YIELD_TODAY = 0xD3ED,
		CR_YIELD_YESTERDAY = 0xD1ED,
		CR_MAX_POWER_YESTERDAY = 0xD0ED,
		CR_VOLTAGE_RANGE = 0xCEED,
		CR_HISTORY_VERSION = 0xCDED,
		CR_STREETLIGHT_VERSION = 0xCCED,
		CR_HISTORY_DAY_RECORD = 0x5010
	};

	enum SolarPanelRegister {
		SPR_POWER = 0xBCED,
		SPR_VOLTAGE = 0xBBED,
		SPR_CURRENT = 0xBDED,
		SPR_MAXIMUM_VOLTAGE = 0xB8ED
	};

	enum LoadOutputRegister {
		LOR_CURRENT = 0xADED,
		LOR_CONTROL = 0xABED,
		LOR_STATE = 0xA8ED,
		LOR_SWITCH_HIGH_LEVEL = 0x9DED,
		LOR = 0x9CED
	};

#define	SR_DEVICE_STATE	0x0102

	SolarCharger();
	bool processReply();
	bool processSerialData(uint8_t data);
	bool parseCommand();
	void parseGetSetFlags(uint8_t flags);
	bool checkSumPassed();
	uint8_t hexToBin(uint8_t data);
	uint16_t readVictron(const char* command, uint16_t& result);
	int16_t readVictron(const char* command, int16_t& result);
	uint32_t readVictron(const char* command, uint32_t& result);
	HistoryDayRecord* readVictron(const char* command);
	bool victronSend(const char* command);
	bool processHistoryRecord();

	SoftwareSerial _comPort;
	CommandState _commandState;
	uint8_t _command[MAX_COMMAND_SIZE];
	uint8_t _commandSize;
	uint8_t _firstNibble;
	uint16_t _energyYieldToday;
	uint16_t _maxPowerToday;
	uint16_t _chargerCurrent;
	uint16_t _chargerVoltage;
	int16_t _chargerTemperature;
	uint16_t _loadCurrent;
	uint16_t _panelCurrent;
	uint16_t _panelVoltage;
	uint32_t _panelPower;
	uint16_t _deviceState;
	HistoryDayRecord _historyDayRecord;
};

#endif /* SOLARCHARGER_H_ */
