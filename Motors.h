#ifndef MOTORS_H
#define MOTORS_H //define not def

#include <avr/io.h>


void timer0_init();
void timer1_init();
void setMotorSpeeds(int8_t MotorSpeeds[2]);

#endif
