/*
 * MainTask.h
 *
 *  Created on: 12/03/2017
 *      Author: Vlad
 */

#ifndef MAINTASK_H_
#define MAINTASK_H_

#include <SleepyPi2.h>

class MainTask {
public:
	static MainTask& instance();
	void run();

private:
#define RPI_SLEEP_TIME				1
#define RTC_INTERRUPT_PIN			0 /* (INT0) */
#define ARDUINO_I2C_SLAVE_ADDRESS	100
#define MAX_MESSAGE_LENGHT			10
#define RPI_SHUTDOWN_CURRENT		85

#define PDU_ROUTER_ON				0x0001
#define PDU_CAM1_ON					0x0002
#define PDU_CAM2_ON					0x0004
#define PDU_CAM3_ON					0x0008
#define PDU_CAM4_ON					0x0010

	enum MessageTag {
		TAG_PDU_CONTROL,
		TAG_RPI_SLEEP_TIME,
	};
	MainTask() :
		_rtcInterrupt(false),
		_messageIndex(0),
		_rpiSleepTime(RPI_SLEEP_TIME),
		_rpiShutdownCurrent(RPI_SHUTDOWN_CURRENT),
		_pduControl(0)
	{
	}
	virtual ~MainTask() {}
	void initRtc();
	static void rtcInterrupt();
	static void i2cInterrupt(int received);
	void processMessage(char data);
	void parseMessage();
	void setPdu();
	char asciiToInt(unsigned char data);

	SleepyPiClass _sleepyPi;
	bool _rtcInterrupt;
	uint8_t _messageIndex;
	uint8_t _rpiSleepTime;
	uint16_t _rpiShutdownCurrent;
	uint16_t _pduControl;
	char _message[MAX_MESSAGE_LENGHT];
};

#endif /* MAINTASK_H_ */
