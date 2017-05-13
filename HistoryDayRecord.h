/*
 * HistoryDayRecord.h
 *
 *  Created on: 11/05/2017
 *      Author: Vlad
 */

#ifndef HISTORYDAYRECORD_H_
#define HISTORYDAYRECORD_H_

#include <stdint.h>

class HistoryDayRecord {
public:
	HistoryDayRecord() :
		_reserved(0),
		_yield(0),
		_consumed(0),
		_batteryVoltageMaximum(0),
		_batteryVoltageMinimum(0),
		_errorDatabase(0),
		_error0(0),
		_error1(0),
		_error3(0),
		_timeBulk(0),
		_timeAbsorption(0),
		_timeFloat(0),
		_powerMaximum(0),
		_batteryCurrentMaximum(0),
		_panelVoltageMaximum(0),
		_daySequenceNumber(0)
{
}
	void set(uint8_t*& source, uint8_t size, void* destination) { uint8_t* d = (uint8_t*)destination; while (size--) *d++ = *source++; }
	void setReserved(uint8_t*& data) { set(data, sizeof(_reserved), &_reserved); }
	void setYield(uint8_t*& data) { set(data, sizeof(_yield), &_yield); }
	void setConsumed(uint8_t*& data) { set(data, sizeof(_consumed), &_consumed); }
	void setBatteryVoltageMaximum(uint8_t*& data) { set(data, sizeof(_batteryVoltageMaximum), &_batteryVoltageMaximum); }
	void setBatteryVoltageMinimum(uint8_t*& data) { set(data, sizeof(_batteryVoltageMinimum), &_batteryVoltageMinimum); }
	void setErrorDataBase(uint8_t*& data) { set(data, sizeof(_errorDatabase), &_errorDatabase); }
	void setError0(uint8_t*& data) { set(data, sizeof(_error0), &_error0); }
	void setError1(uint8_t*& data) { set(data, sizeof(_error1), &_error1); }
	void setError2(uint8_t*& data) { set(data, sizeof(_error2), &_error2); }
	void setError3(uint8_t*& data) { set(data, sizeof(_error3), &_error3); }
	void setTimeBulk(uint8_t*& data) { set(data, sizeof(_timeBulk), &_timeBulk); }
	void setTimeAbsorption(uint8_t*& data) { set(data, sizeof(_timeAbsorption), &_timeAbsorption); }
	void setTimeFloat(uint8_t*& data) { set(data, sizeof(_timeFloat), &_timeFloat); }
	void setPowerMaximum(uint8_t*& data) { set(data, sizeof(_powerMaximum), &_powerMaximum); }
	void setBatteryCurrentMaximum(uint8_t*& data) { set(data, sizeof(_batteryCurrentMaximum), &_batteryCurrentMaximum); }
	void setPanelVoltageMaximum(uint8_t*& data) { set(data, sizeof(_panelVoltageMaximum), &_panelVoltageMaximum); }
	void setDaySequenceNumber(uint8_t*& data) { set(data, sizeof(_daySequenceNumber), &_daySequenceNumber); }

	uint8_t getReserved() { return _reserved; }
	uint32_t getYield() { return _yield; }
	uint32_t getConsumed() { return _consumed; }
	uint16_t getBatteryVoltageMaximum() { return _batteryVoltageMaximum; }
	uint16_t getBatteryVoltageMinimum() { return _batteryVoltageMinimum; }
	uint8_t getErrorDatabase() { return _errorDatabase; }
	uint8_t getError0() { return _error0; }
	uint8_t getError1() { return _error1; }
	uint8_t getError2() { return _error2; }
	uint8_t getError3() { return _error3; }
	uint16_t getTimeBulk() { return _timeBulk; }
	uint16_t getTimeAbsorption() { return _timeAbsorption; }
	uint16_t getTimeFloat() { return _timeFloat; }
	uint32_t getPowerMaximum() { return _powerMaximum; }
	uint16_t getBatteryCurrentMaximum() { return _batteryCurrentMaximum; }
	uint16_t getPanelVoltageMaximum() { return _panelVoltageMaximum; }
	uint16_t getDaySequenceNumber() { return _daySequenceNumber; }
	virtual ~HistoryDayRecord() {}

private:
	uint8_t _reserved;
	uint32_t _yield;
	uint32_t _consumed;
	uint16_t _batteryVoltageMaximum;
	uint16_t _batteryVoltageMinimum;
	uint8_t _errorDatabase;
	uint8_t _error0;
	uint8_t _error1;
	uint8_t _error2;
	uint8_t _error3;
	uint16_t _timeBulk;
	uint16_t _timeAbsorption;
	uint16_t _timeFloat;
	uint32_t _powerMaximum;
	uint16_t _batteryCurrentMaximum;
	uint16_t _panelVoltageMaximum;
	uint16_t _daySequenceNumber;
};

#endif /* HISTORYDAYRECORD_H_ */
