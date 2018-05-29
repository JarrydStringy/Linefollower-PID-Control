#include "Motors.h"
#include <math.h>

void SetUpMotors(){
	DDRB |= (1<<5); //Motor 1 as output
	DDRB |= (1<<6); //Set both 5 and 6 pins to high possibly allowing motors to reverse
	//TCCR1A |= (1<<7)|(1<<5)|(1<<2)|1; //Motor 1 set top and bottom for PWM control
	TCCR1A |= (1<<7)|(1<<5)|1; //Motor 1 set top and bottom for PWM control
	//TCCR1B |= (1<<2); //Motor 1, Timer 0, Prescaler set to 256
	TCCR1B |= (1<<3)|(1<<2); //Motor 1, Timer 0, Prescaler set to 256

	DDRB |= (1<<7); //Motor 2 as output
	//DDRB |= (1<<8); //Let motor 2 go backwards
	DDRD |= (1<<0); //Set both Port B7 and PORT D0 to high possibly allowing motors to reverse
	TCCR0A |= (1<<7)|(1<<5)|(1<<1)|1; //Motor 2 set top and bottom for PWM control
	TCCR0B |= (1<<2); //Motor 2, Timer 1, Prescaler set to 256

	// OCR1A = 0;
	// OCR0A = 0;
}

/*
Sets the motor to move at DC represented by a percentage input
Negative Values cause reverse movement.
*/
void setMotorSpeeds(double motors1, double motors2){
	motors1 = fmin(fmax(motors1, -100), 100);
	motors2 = fmin(fmax(motors2, -100), 100);
	//
	int MSpeeds[2];
	MSpeeds[0] = (double)(motors1 * 255.0/100.0);
	MSpeeds[1] = (double)(motors2 * 255.0/100.0);


	if (motors1>0){
		OCR0A = MSpeeds[0];
		OCR0B = 0;
	}
	else{
		OCR0A = 0;
		OCR0B = -MSpeeds[0];
	}

	if (motors2>0){
		OCR1A = MSpeeds[1];
		OCR1B = 0;
	}
	else{
		OCR1A = 0;
		OCR1B = -MSpeeds[1];
	}
	return;
}
