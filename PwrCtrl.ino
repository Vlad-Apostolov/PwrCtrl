#include "MainTask.h"

#include <PCF8523.h>
#include <Time.h>
#include <Wire.h>
#include <LowPower.h>
#include <SleepyPi2.h>

void setup() { }

void loop() 
{
	MainTask::instance().run();
}
