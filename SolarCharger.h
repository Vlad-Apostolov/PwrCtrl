/*
 * SolarCharger.h
 *
 *  Created on: 18/03/2017
 *      Author: Vlad
 */

#ifndef SOLARCHARGER_H_
#define SOLARCHARGER_H_

#include <SoftwareSerial.h>

class SolarCharger {
public:
	SolarCharger(uint8_t rxPin, uint8_t txPin) :
		_comPort(rxPin, txPin)
	{
	}
	virtual ~SolarCharger() {}

private:
	SolarCharger();

	SoftwareSerial _comPort;
};

#endif /* SOLARCHARGER_H_ */
