#include "Motors.h"

void timer0_init(){
//motor one
	DDRB |= (1<<7); // initilises pin b
	DDRD |= (1<<0); // initilises pin d
	TCCR0A |= (1<<7)|(1<<5)|(1<<1)|1;
	TIMSK0 |= (1<<0);
	OCR0A = 0;
	OCR0B = 0;
	TCCR0B |= (1<<2);

	return;
}

void timer1_init(){
//motor 2
	DDRB |= (1<<6)|(1<<5); //initilises pin b
	TCCR1A |= (1<<7)|(1<<5)|1;
	TCCR1B |= (1<<3);
	TIMSK1 |= (1<<0);
	OCR1A = 0;
	OCR1B = 0;
	TCCR1B |= (1<<2);

	return;
}


/*
Sets the motor to move at DC represented by a percentage input
Negative Values cause reverse movement.
*/
void setMotorSpeeds(int8_t MotorSpeeds[2]){
	double MSpeeds[2];
	MSpeeds[0] = 255*fmin(fmax(MotorSpeeds[0], -100), 100)/100;
	MSpeeds[1] = 255*fmin(fmax(MotorSpeeds[1], -100), 100)/100;


	if (MotorSpeeds[0]>0){
		OCR0A = MSpeeds[0];
		OCR0B = 0;
	}
	else{
		OCR0A = 0;
		OCR0B = -MSpeeds[0];
	}

	if (MotorSpeeds[1]>0){
		OCR1A = MSpeeds[1];
		OCR1B = 0;
	}
	else{
		OCR1A = 0;
		OCR1B = -MSpeeds[1];
	}

	return;
}
