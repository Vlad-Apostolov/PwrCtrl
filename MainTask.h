/*
 * MainTask.h
 *
 *  Created on: 12/03/2017
 *      Author: Vlad
 */

#ifndef MAINTASK_H_
#define MAINTASK_H_

#include <SleepyPi2.h>

#include "SolarCharger.h"


class MainTask {
public:
	static MainTask& instance();
	void run();

private:
#define RPI_SLEEP_TIME				30
#define SPI_SLEEP_TIME				5
#define RTC_INTERRUPT_PIN			0 /* (INT0) */
#define ARDUINO_I2C_SLAVE_ADDRESS	55
#define MESSAGE_LENGHT				7
#define RPI_SHUTDOWN_CURRENT		85

#define PDU_ROUTER_ON				0x0001
#define PDU_CAM1_ON					0x0002
#define PDU_CAM2_ON					0x0004
#define PDU_CAM3_ON					0x0008
#define PDU_CAM4_ON					0x0010
#define RPI_ON						0x0020

#define SOLAR_CHARGER_RX_PIN		10
#define SOLAR_CHARGER_TX_PIN		9

#define MAX_SOLAR_CHARGER_DATA		20

	enum MessageTag {
		TAG_PDU_CONTROL,
		TAG_RPI_SLEEP_TIME,
		TAG_SPI_SLEEP_TIME
	};
	struct SolarChargerData {
		uint32_t time;
		uint16_t chargerVoltage;
		uint16_t chargerCurrent;
		uint16_t chargerPowerToday;
		uint16_t loadVoltage;
		uint16_t loadCurrent;
		uint16_t panelVoltage;
		uint16_t panelCurrent;
		uint32_t panelPower;
		int8_t cpuTemperature;
	};
	MainTask() :
		_rtcInterrupt(false),
		_messageIndex(0),
		_rpiSleepTime(RPI_SLEEP_TIME),
		_spiSleepTime(SPI_SLEEP_TIME),
		_sleepTime(0),
		_rpiShutdownCurrent(RPI_SHUTDOWN_CURRENT),
		_pduControl(0),
		_solarChargerDataRead(0),
		_solarChargerDataWrite(0),
		_solarCharger(SOLAR_CHARGER_RX_PIN, SOLAR_CHARGER_TX_PIN)
	{
	}
	virtual ~MainTask() {}
	void initRtc();
	static void rtcInterrupt();
	static void i2cReceiver(int received);
	static void i2cTransmitter();
	void powerDownPi(bool state);
	void parseMessage(char data);
	void setPdu();
	char asciiToInt(unsigned char data);
	void readSolarCharger();
	SolarChargerData& nextSolarChargerDataWrite();
	SolarChargerData* nextSolarChargerDataRead();
	int8_t getCpuTemperature();

	SleepyPiClass _sleepyPi;
	bool _rtcInterrupt;
	uint8_t _messageIndex;
	uint8_t _rpiSleepTime;
	uint8_t _spiSleepTime;
	uint8_t _sleepTime;
	uint16_t _rpiShutdownCurrent;
	uint16_t _pduControl;
	SolarCharger _solarCharger;
	char _message[MESSAGE_LENGHT];
	SolarChargerData _solarChargerData[MAX_SOLAR_CHARGER_DATA];
	uint8_t _solarChargerDataRead;
	uint8_t _solarChargerDataWrite;
};

#endif /* MAINTASK_H_ */
