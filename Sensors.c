#include "Sensors.h"
#define THRESHOLD 160
void ADC_init(){
	// ADC Enable and prescaler of 128
	// 16000000/128 = 125000
	ADMUX |= (1<<REFS1) | (1<<REFS0) | (1<<ADLAR);
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);

	ADCSRA |= (1 << ADSC);
}

uint8_t SensorValue(){

	ADMUX = 0b11100011; // clears the bottom 3 bits before ORing
	ADCSRB =0b00100000;
	ADCSRA |= (1<<ADSC);
	while(ADCSRA & (1<<ADSC));
	uint8_t sensor_out = ADCH; // sensor out wasnt assigned

	return sensor_out;
}



double line_Positionx(){
		double line = 0;
		int num = 0;
		uint8_t sensor_ADMUX[] =  {0b11100101,0b11100110,0b11100111,0b11100011,0b11100010,0b11100001};
	  uint8_t sensor_ADCSRB[] = {0b00000000,0b00000000,0b00000000,0b00100000,0b00100000,0b00100000};
		uint8_t sensor[6];
		int x1=0; int x2=0; int x3=0;
		double y1=0; double y2=0; double y3=0;
		double MinVal=255*3;
		double a;

	  int i = 0;
	  while (i<6){
	    ADMUX = sensor_ADMUX[i];
	    ADCSRB = sensor_ADCSRB[i];
	    ADCSRA |= (1<<ADSC);
	    while(ADCSRA & (1<<ADSC));
	    sensor[i] = ADCH;
			i++;
	  }

		i = 0;
		while(i<4){
			if((sensor[i]+sensor[i+1]+sensor[i+2]) < MinVal){
				y1 =sensor[i]; y2 =sensor[i+1]; y3 =sensor[i+2];
				x1 = i; 				x2 = (i+1);				  x3 = (i+2);
				MinVal = (sensor[i]+sensor[i+1]+sensor[i+2]);
			}
			i++;
		}

		a = (y1+y2+2*y3)/2;

		line = y2 - y1 - 2*a*x1 - a;

		return line;

}
