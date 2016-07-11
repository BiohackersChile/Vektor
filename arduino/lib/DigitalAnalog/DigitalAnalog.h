#ifndef DIYBIO_DIGITALANALOG_H
#define DIYBIO_DIGITALANALOG_H

#include <Wire.h>

void digital_analog_setup(void);
void analogXUpdate(int dt);
void pinXMode(uint8_t pin, bool mode);
void analogXWrite(uint8_t pin, uint8_t v);

#endif //DIYBIO_DIGITALANALOG_H
