#include "Sensors.h"

uint8_t sensor_ADMUX[] =  {0b11100100,0b11100101,0b11100110,0b11100111,0b11100011,0b11100010,0b11100001,0b11100000};
uint8_t sensor_ADCSRB[] = {0b00000000,0b00000000,0b00000000,0b00000000,0b00100000,0b00100000,0b00100000,0b00100000};

void ADC_init(){
	// ADC Enable and prescaler of 128
	// 16000000/128 = 125000
	ADMUX |= (1<<REFS1) | (1<<REFS0) | (1<<ADLAR);
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);

	ADCSRA |= (1 << ADSC);
}

// ADC Registers for Sensors

/**
 *	Read any of the sensors who's registers are defined as sensor_ADMUX and
 *  sensor_ADCSRB.
 *
 *	Parameters:
 *		sensor_n - The sensor to be read where 0 is the first sensor  in the
 *      register arrays
 */
uint8_t readSensor(int sensor_n){
  ADMUX = sensor_ADMUX[sensor_n];
  ADCSRB = sensor_ADCSRB[sensor_n];
  ADCSRA |= (1<<ADSC);
  while(ADCSRA & (1<<ADSC));
  return ADCH;
}
