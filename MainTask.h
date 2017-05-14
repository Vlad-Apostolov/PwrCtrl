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
#define RPI_SLEEP_TIME				2
#define SPI_SLEEP_TIME				1
#define RTC_INTERRUPT_PIN			0 /* (INT0) */
#define ARDUINO_I2C_SLAVE_ADDRESS	55
#define MESSAGE_LENGHT				13
#define RPI_SHUTDOWN_CURRENT		150
#define RPI_POWER_UP_CURRENT		250

#define PDU_RELAY1_ON				0x0001
#define PDU_RELAY2_ON				0x0002
#define PDU_RELAY3_ON				0x0004
#define PDU_RELAY4_ON				0x0008
#define PDU_RELAY5_ON				0x0010
#define PDU_RELAY6_ON				0x0020
#define PDU_RELAY7_ON				0x0040
#define PDU_RELAY8_ON				0x0080
#define PDU_EXTERNAL_POWER_ON		0x0100

#define SOLAR_CHARGER_RX_PIN		10	// PB2
#define SOLAR_CHARGER_TX_PIN		9	// PB1
#define RELAY1_PIN					11	// PB3
#define RELAY2_PIN					12	// PB4
#define RELAY3_PIN					14	// PC0
#define RELAY4_PIN					15	// PC1
#define RELAY5_PIN					5	// PD5
#define RELAY6_PIN					6	// PD6
#define RELAY7_PIN					8	// PB0

#define MAX_SOLAR_CHARGER_DATA		24
#define RTC_INTERRUPT_PERIOD		1
#define YELLOW_LED_PIN				13

	enum MessageTag {
		TAG_PDU_CONTROL,
		TAG_RPI_SLEEP_TIME,
		TAG_SPI_SLEEP_TIME,
		TAG_SPI_SYSTEM_TIME
	};
	struct SolarChargerData {
		uint32_t time;
		uint32_t panelPower;
		uint32_t consumedToday;
		uint32_t consumedYesterday;
		uint16_t energyYieldToday;
		uint16_t chargerVoltage;
		uint16_t chargerCurrent;
		uint16_t loadCurrent;
		uint16_t panelVoltage;
		int16_t cpuTemperature;
		uint16_t deviceState;
		uint16_t spare;
	};
	MainTask() :
		_rtcInterrupt(false),
		_wdtInterrupt(false),
		_messageIndex(0),
		_rpiSleepTime(RPI_SLEEP_TIME),
		_spiSleepTime(SPI_SLEEP_TIME),
		_systemTime(0),
		_upTime(0),
		_rpiShutdownCurrent(RPI_SHUTDOWN_CURRENT),
		_pduControl(0),
		_solarChargerDataRead(0),
		_solarChargerDataWrite(0),
		_rtcPeriodInMunutes(RTC_INTERRUPT_PERIOD),
		_wdSeconds(0),
		_rtcFailed(false),
		_solarCharger(SOLAR_CHARGER_RX_PIN, SOLAR_CHARGER_TX_PIN)
	{
	}
	virtual ~MainTask() {}
	void initRtc();
	static void rtcInterrupt();
	static void i2cReceiver(int received);
	static void i2cTransmitter();
	void powerUpPi();
	void powerDownPi();
	void parseMessage(char data);
	void setPdu();
	char asciiToInt(unsigned char data);
	void readSolarCharger();
	SolarChargerData& nextSolarChargerDataWrite();
	SolarChargerData* nextSolarChargerDataRead();
	int8_t getCpuTemperature();
	void startI2cSlave();

	SleepyPiClass _sleepyPi;
	bool _rtcInterrupt;
	bool _wdtInterrupt;
	uint8_t _messageIndex;
	uint8_t _rpiSleepTime;
	uint8_t _spiSleepTime;
	uint32_t _systemTime;
	uint16_t _upTime;
	uint16_t _rpiShutdownCurrent;
	uint16_t _pduControl;
	uint8_t _solarChargerDataRead;
	uint8_t _solarChargerDataWrite;
	uint8_t _rtcPeriodInMunutes;
	uint16_t _wdSeconds;
	bool _rtcFailed;
	SolarCharger _solarCharger;
	char _message[MESSAGE_LENGHT];
	SolarChargerData _solarChargerData[MAX_SOLAR_CHARGER_DATA];
};

#endif /* MAINTASK_H_ */
