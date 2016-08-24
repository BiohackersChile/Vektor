#ifndef DIYBIO_DIGITALANALOG_H
#define DIYBIO_DIGITALANALOG_H

#include <Wire.h>

/*
setups pins variables,
general, pwm settings
mappings (in, out)
data arrays
*/
void digital_analog_setup(int mappings_length, int* mapping);
void digital_analog_setup();

/*
executed by "thread" emulator, a small delay betwen executions gives
better results
*/
void analogXUpdate(unsigned long dt, bool print);
void analogXUpdate();

/*
enable, disable pin
*/
void pinXMode(uint8_t pin, bool mode);

/*
from a voltaje, with max_voltaje specified in setup
*/
void analogXWrite(uint8_t pin, uint8_t v);

#endif //DIYBIO_DIGITALANALOG_H
