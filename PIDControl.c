#define F_CPU 16000000UL //set clock speed

#include <avr/io.h> //include library for register names
#include <util/delay.h>
#include "Motors.h"
#include "Sensors.h"

#define intersections 2


#define Kp 25
#define Kd 10
#define Ki 0

#define THRESHOLD 110

#define RIGHT_SLOW_SPEED 15
#define LEFT_SLOW_SPEED 15
#define RIGHT_MID_SPEED 20
#define LEFT_MID_SPEED 20
#define RIGHT_FAST_SPEED 35
#define LEFT_FAST_SPEED 35
#define MAX_CORRECTION 127

  int RIGHT_BASE_SPEED =RIGHT_SLOW_SPEED;
  int LEFT_BASE_SPEED =LEFT_SLOW_SPEED;

int onWhite[2];
int onColour;
int slowZone = 0;

double maxVal[] = {0,0,0,0,0,0,0};
double minVal[] = {255,255,255,255,255,255,255};


// int lindex=0;

void setup(){
  DDRB |= (1<<0); //set up LED as output
  DDRB |= (1<<1); //set up LED as output
  DDRB |= (1<<2); //set up LED as output
  DDRB |= (1<<3); //set up LED as output

  ADC_init();//sensors.c
  SetUpMotors();
}

double line_Position(){
  uint8_t sensor_ADMUX[] =  {0b11100100,0b11100101,0b11100110,0b11100111,0b11100011,0b11100010};
  uint8_t sensor_ADCSRB[] = {0b00000000,0b00000000,0b00000000,0b00000000,0b00100000,0b00100000};
  uint8_t sensor[6];
  double sensor_calib[6];
  double sum = 0;
  double weightedSum = 0;
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
    sensor_calib[i] = 1000 - (sensor[i] - minVal[i])/(maxVal[i] - minVal[i])*1000;
    sum = sum + sensor_calib[i];
    weightedSum = weightedSum + (i * sensor_calib[i]);
    i++;
  }
  error = 2*(weightedSum/sum) - 5;
  if (sum < 300*6){
    error = 10;
  }
  return error;
}

void callibrate(){
  // ADC registers for middle six sensors
  uint8_t sensor_ADMUX[] =  {0b11100100,0b11100101,0b11100110,0b11100111,0b11100011,0b11100010,0b11100001,0b11100000};
  uint8_t sensor_ADCSRB[] = {0b00000000,0b00000000,0b00000000,0b00000000,0b00100000,0b00100000,0b00100000,0b00100000};
  int i = 0;
  // Read ADCs
  while (i<8){
    ADMUX = sensor_ADMUX[i];
    ADCSRB = sensor_ADCSRB[i];
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1<<ADSC));
    //If this is greater than prev. max, save new max
    if (ADCH > maxVal[i]){
      maxVal[i] = ADCH;
    }
    //If this is less than prev. min, save new min
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

void correctError( double correction ){
  double motorSpeeds[2] = {0,0};
  motorSpeeds[0] = fmin(fmax(LEFT_BASE_SPEED - correction, -100), 100);
  motorSpeeds[1] = fmin(fmax(RIGHT_BASE_SPEED + correction, -100), 100);

  uint8_t MSpeeds1; uint8_t MSpeeds2;
  MSpeeds1 = (double)(motorSpeeds[0] * 255.0/100.0);
  MSpeeds2 = (double)(motorSpeeds[1] * 255.0/100.0);

  if (motorSpeeds[0]>=0){
    OCR0A = MSpeeds1;
    OCR0B = 0;
  }
  else{
    OCR0A = 0;
    OCR0B = -MSpeeds1;
  }

  if (motorSpeeds[1]>=0){
    OCR1A = MSpeeds2;
    OCR1B = 0;
  }
  else{
    OCR1A = 0;
    OCR1B = -MSpeeds2;
  }
}


void stopMotors(){
  OCR0A = 0;
  OCR0B = 0;
  OCR1A = 0;
  OCR1B = 0;
}





int detectMarkers(){
  int prevState[2];
  uint8_t sensor_ADMUX[] =  {0b11100001,0b11100000};
  uint8_t sensor_ADCSRB[] = {0b00100000,0b00100000};
  uint8_t sides[2];
  double sides_calib[2];
  int case_ = 0;
  int i = 0;
  while (i<2){
    prevState[i]= onWhite[i];
    ADMUX = sensor_ADMUX[i];
    ADCSRB = sensor_ADCSRB[i];
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1<<ADSC));
    sides[i] = ADCH;
    sides_calib[i] = (sides[i] - minVal[i+6])/(maxVal[i+6] - minVal[i+6])*1000;

    if (sides_calib[i]<100){
      onWhite[i] = 1;
    }
    else {onWhite[i] = 0;}
    if (onWhite[i]==1)PORTB |= (1<<(2*i+1));
    else PORTB &= ~(1<<(2*i+1));
    i++;
    }
    if(onWhite[0]>prevState[0]){
      case_ = case_ + 1;
    }
    if(onWhite[1]<prevState[1]){
      case_ = case_ + 2;
    }
  return case_;
}


int main(){
  uint8_t sensor[6];
  int set = 0;
  setup();
  //
  // motorSpeeds[0] = -100;
  // motorSpeeds[1] = -100;
  // setMotorSpeeds(motorSpeeds[0],motorSpeeds[1]);
  // while(1);

  //Setup Loop (See Setup Function for Full Setup)
  while(set == 0){
    callibrate();
    if(PINC&(1<<6)){
      set = 1;
      PORTB |= (1<<0); //turn LED on
    }
  }

  double error;
  double last_error;
  int iteration = 1;
  int RMarker = 1;
  double errorSum=0;
  double errorAve=0;
  double derivative = 0;
  double correction;
  int case_;

  // int8_t motorSpeeds[2] = {0,0};
	while(1){ //infinte loop
    case_ = detectMarkers();
    // callibrate();
    // SetUpMotors();
    error = line_Position();
    if (error == 10){
      error = last_error;
    }

    errorSum = errorSum + error;

    if(case_ == 1 || iteration>1000){
      iteration = 1;
      errorSum = 0;
    }
    if(case_ == 2){
      if(RMarker == 2*intersections + 2){
        _delay_ms(100);
        stopMotors();
        _delay_ms(1000);
        RMarker = 1;
      }
      else{
        RMarker++;
      }
    }

    if( errorSum/(double)iteration > -0.5  &&  errorSum/(double)iteration < 0.5 ){
      // PORTB &= ~(1<<1);
      PORTB |= (1<<2);
      RIGHT_BASE_SPEED =RIGHT_FAST_SPEED;
      LEFT_BASE_SPEED =LEFT_FAST_SPEED;
    }
    else {
      // PORTB |= (1<<1);
      PORTB &= ~(1<<2);
      RIGHT_BASE_SPEED =RIGHT_MID_SPEED;
      LEFT_BASE_SPEED =LEFT_MID_SPEED;
    }

    derivative = error - last_error;
    last_error = error;

    // correction = MAX_CORRECTION*(Kp*error + Kd*derivative)/(Kp*5 + Kd*10);
    correction = Kp*error + Kd*derivative;

    correctError(correction);
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
