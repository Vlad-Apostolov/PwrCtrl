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
	enum MessageState {
		MSG_HEADER,
		MSG_LENGTH_1,
		MSG_LENGTH_2,
		MSG_BODY
	};
	enum MessageTag {
		TAG_PDU_CONTROL,
		TAG_RPI_SLEEP_TIME,
	};
#define RPI_SLEEP_TIME				1
#define RTC_INTERRUPT_PIN			0 /* (INT0) */
#define ARDUINO_I2C_SLAVE_ADDRESS	100
#define MAX_MESSAGE_LENGHT			10
#define RPI_SHUTDOWN_CURRENT		85
	MainTask() :
		_rtcInterrupt(false),
		_messageState(MSG_HEADER),
		_messageLength(0),
		_messageIndex(0),
		_rpiSleepTime(RPI_SLEEP_TIME),
		_rpiShutdownCurrent(RPI_SHUTDOWN_CURRENT)
	{
	}
	virtual ~MainTask() {}
	void initRtc();
	static void rtcInterrupt();
	static void i2cInterrupt(int received);
	void processMessage(char data);
	void parseMessage();
	char asciiToInt(unsigned char data);

	SleepyPiClass _sleepyPi;
	bool _rtcInterrupt;
	MessageState _messageState;
	unsigned char _messageLength;
	unsigned char _messageIndex;
	unsigned char _rpiSleepTime;
	unsigned short _rpiShutdownCurrent;
	char _message[MAX_MESSAGE_LENGHT];
};

#endif /* MAINTASK_H_ */
