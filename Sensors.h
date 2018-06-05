#ifndef SENSORS_H
#define SENSORS_H

#include <avr/io.h>

void ADC_init();
uint8_t readSensor(int sensor_n);

#endif
