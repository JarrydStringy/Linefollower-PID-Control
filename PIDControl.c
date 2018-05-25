#define F_CPU 16000000UL //set clock speed

#include <avr/io.h> //include library for register names
#include <util/delay.h>
#include "Motors.h"
#include "Sensors.h"

#define Kp 10
#define Kd 0
#define Ki 0

#define INTEGRAL_MAX 80

#define THRESHOLD 110

#define RIGHT_BASE_SPEED 20
#define LEFT_BASE_SPEED 20
#define MAX_CORRECTION 127


double maxVal[] = {0,0,0,0,0,0};
double minVal[] = {255,255,255,255,255,255};

// int lindex=0;

void setup(){
  DDRB |= (1<<0); //set up LED as output
  DDRB |= (1<<1); //set up LED as output
  DDRB |= (1<<2); //set up LED as output
  DDRE |= (1<<6); //set up LED as output

  ADC_init();//sensors.c
  timer0_init();//motors.c
  timer1_init();//motors.c
}

double line_Position(){
  uint8_t sensor_ADMUX[] =  {0b11100101,0b11100110,0b11100111,0b11100011,0b11100010,0b11100001};
  uint8_t sensor_ADCSRB[] = {0b00000000,0b00000000,0b00000000,0b00100000,0b00100000,0b00100000};
  uint8_t sensor[6];
  uint16_t sensor_calib[6];
  uint16_t sum = 0;
  uint16_t weightedSum = 0;
  double error;
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
  while (i<6){
    sensor_calib[i] = (sensor[i] - minVal[i])/(maxVal[i]-minVal[i])*1000;
    sum = sum + sensor_calib[i];
    weightedSum = weightedSum + (i * sensor_calib[i]);
    i++;
  }
  error = 2*(weightedSum/sum) - 5;
  return error;
}

void callibrate(){
  uint8_t sensor_ADMUX[] =  {0b11100101,0b11100110,0b11100111,0b11100011,0b11100010,0b11100001};
  uint8_t sensor_ADCSRB[] = {0b00000000,0b00000000,0b00000000,0b00100000,0b00100000,0b00100000};
  int i = 0;
  while (i<6){
    ADMUX = sensor_ADMUX[i];
    ADCSRB = sensor_ADCSRB[i];
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1<<ADSC));
    if (ADCH > maxVal[i]){
      maxVal[i] = ADCH;
    }
    if (ADCH < minVal[i]){
      minVal[i] = ADCH;
    }
    i++;
  }
}


// void Straight_Detection(double error){
//   last10[lindex] = error;
//   lindex++;
//   if(lindex>9){lindex=0;}
//
//   double lineSum=0;
//   int i = 0;
//   while(i<10){
//     lineSum += last10[i];
//   }
//   i++;
//   double lineAve = lineSum/10;
//   if(lineAve>-1&&lineAve<1){
//     PORTB |= (1>>0);
//   }
//   else{
//     PORTB &= ~(1>>0);
//   }
// }

int main(){
  uint8_t sensor[6];
  int set = 0;

  //Setup Loop (See Setup Function for Full Setup)
  while(set == 0){
    setup();
    if(PINC&(1<<6)){
      callibrate();
      set = 1;
      PORTB |= (1<<2); //turn LED on
    }
  }

  double error;
  double last_error;
  int iteration = 1;
  double errorSum=0;
  double errorAve=0;
  double derivative = 0;
  double integral = 0;
  double correction;
  int8_t motorSpeeds[2] = {0,0};
	while(1){ //infinte loop
    error = line_Position();
    if (error == 10){
      error = last_error;
    }

    errorSum = errorSum + error;

    if(iteration>1000){
      iteration = 1;
      errorSum = 0;
    }


    if( errorSum/iteration > -0.5  &&  errorSum/iteration < 0.5 ){
      PORTB |= (1>>0);
    }
    else{
      PORTB &= ~(1>>0);
    }

    derivative = error - last_error;
    last_error = error;

    integral = fmin(fmax(integral + error,-INTEGRAL_MAX),INTEGRAL_MAX);

    correction = MAX_CORRECTION*(Kp*error + Kd*derivative)/(Kp*5 + Kd*10);
    // correction = Kp*error + Kd*derivative;

    motorSpeeds[0] = fmin(fmax(LEFT_BASE_SPEED - correction, -100), 100);
    motorSpeeds[1] = fmin(fmax(RIGHT_BASE_SPEED + correction, -100), 100);


  	setMotorSpeeds(motorSpeeds);
    iteration++;


      // last10[lindex] = error;
      // lindex++;
      // if(lindex>9){lindex=0;}
      //
      // double lineSum=0;
      // int i = 0;
      // while(i<10){
      //   lineSum += last10[i];
      // }
      // i++;
      // double lineAve = lineSum/10;
      // if(lineAve>-1&&lineAve<1){
      //   PORTB |= (1>>0);
      // }
      // else{
      //   PORTB &= ~(1>>0);
      // }
	}
}
