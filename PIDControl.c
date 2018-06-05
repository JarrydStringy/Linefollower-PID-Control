#define F_CPU 16000000UL //set clock speed

#include <avr/io.h> //include library for register names
#include <util/delay.h>
#include "Motors.h"
#include "Sensors.h"

//Hard code of number of intersections
#define intersections 2

//PID control constants, Ki unused
#define Kp 18
#define Kd 26
#define Ki 0

//Speed constants
#define RIGHT_SLOW_SPEED 20
#define LEFT_SLOW_SPEED 20
#define RIGHT_FAST_SPEED 40
#define LEFT_FAST_SPEED 40

  int RIGHT_BASE_SPEED =RIGHT_SLOW_SPEED;
  int LEFT_BASE_SPEED =LEFT_SLOW_SPEED;

int onWhite[2];

double maxVal[] = {0,0,0,0,0,0,0};
double minVal[] = {255,255,255,255,255,255,255};


void setup(){
  DDRB |= (1<<0); //set up LED as output
  DDRB |= (1<<1); //set up LED as output
  DDRB |= (1<<2); //set up LED as output
  DDRB |= (1<<3); //set up LED as output
  DDRD |= (1<<5); //set up LED as output

  ADC_init();//sensors.c
  SetUpMotors();
}


/*
 * line_Position
 *    Outputs the error value (line position) used for PD control
 *    using weighted average of normalised sensor values.
 */
double line_Position(){
  uint8_t sensor_ADMUX[] =  {0b11100100,0b11100101,0b11100110,0b11100111,0b11100011,0b11100010};
  uint8_t sensor_ADCSRB[] = {0b00000000,0b00000000,0b00000000,0b00000000,0b00100000,0b00100000};
  uint8_t sensor[6];
  double sensor_calib[6];
  double sum = 0;
  double weightedSum = 0;
  double error;

  // Read Sensors
  int i = 0;
  while (i<6){
    ADMUX = sensor_ADMUX[i];
    ADCSRB = sensor_ADCSRB[i];
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1<<ADSC));
    sensor[i] = ADCH;
    i++;
  }

  // Normalise values and calculate sum and weightedSum
  i = 0;
  while (i<6){
    sensor_calib[i] = 1000 - (sensor[i] - minVal[i])/(maxVal[i] - minVal[i])*1000;
    sum = sum + sensor_calib[i];
    weightedSum = weightedSum + (i * sensor_calib[i]);
    i++;
  }

  // Calculate error value
  error = 2*(weightedSum/sum) - 5;

  // If the line is not seen return exit value
  if (sum < 300*6){
    error = 10;
  }
  return error;
}

/*
 * callibrate
 *    Checks the current sensor values and stores them if they are new maxima
 *    or minima. Allows callibrated normalisation if run in a loop prior to the
 *    main code loop.
 */
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


/*
 * correctError
 *    Given a pre calculated correction as an input, turn to line.
 */
void correctError( double correction ){
  double motorSpeeds[2] = {0,0};

  //Ensure the values are possible (-100% to 100%)
  motorSpeeds[0] = fmin(fmax(LEFT_BASE_SPEED - correction, -100), 100);
  motorSpeeds[1] = fmin(fmax(RIGHT_BASE_SPEED + correction, -100), 100);

  // Convert motorSpeeds percentages to integer duty cycles 0-255
  uint8_t MSpeeds1; uint8_t MSpeeds2;
  MSpeeds1 = (double)(motorSpeeds[0] * 255.0/100.0);
  MSpeeds2 = (double)(motorSpeeds[1] * 255.0/100.0);

  // Run motor 0 (Forwards or Backwards)
  if (motorSpeeds[0]>=0){
    OCR0A = MSpeeds1;
    OCR0B = 0;
  }
  else{
    OCR0A = 0;
    OCR0B = -MSpeeds1;
  }
  // Run motor 1 (Forwards or Backwards)
  if (motorSpeeds[1]>=0){
    OCR1A = MSpeeds2;
    OCR1B = 0;
  }
  else{
    OCR1A = 0;
    OCR1B = -MSpeeds2;
  }
}

// Stop all motion
void stopMotors(){
  OCR0A = 0;
  OCR0B = 0;
  OCR1A = 0;
  OCR1B = 0;
}

/*
 * detectMarkers
 *    Check if wing sensors see side markers.
 *    Outputs:
 *      1 - only LEFT marker seen
 *      2 - only RIGHT marker seen
 *      3 - both markers seen
 */
int detectMarkers(){
  int prevState[2];
  uint8_t sensor_ADMUX[] =  {0b11100001,0b11100000};
  uint8_t sensor_ADCSRB[] = {0b00100000,0b00100000};
  uint8_t sides[2];
  double sides_calib[2];
  int case_ = 0;
  int i = 0;
  while (i<2){
    // Read Sensor
    prevState[i] = onWhite[i];
    ADMUX = sensor_ADMUX[i];
    ADCSRB = sensor_ADCSRB[i];
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1<<ADSC));
    sides[i] = ADCH;
    sides_calib[i] = (sides[i] - minVal[i+6])/(maxVal[i+6] - minVal[i+6])*1000;
    // Check if current sensor sees white and indicate.
    if (sides_calib[i]<500){
      onWhite[i] = 1;
      PORTB |= (1<<(3*i));
    }
    else{
      onWhite[i] = 0;
      PORTB &= (1<<(3*i));
    }
    i++;
  }
  //Check if seeing left marker (rising edge)
  if(onWhite[0]>prevState[0]){
    case_ = case_ +1;
  }
  // Check if seeing right marker (falling edge)
  if(onWhite[1]<prevState[1]){
    case_ = case_ +2;
  }
  return case_;
}

int main(){
  int set = 0;
  setup();
  // Callibration loop
  while(set == 0){
    callibrate();
    if(PINC&(1<<6)){
      set = 1;
      PORTD |= (1<<5); //turn LED on
    }
  }

  double error;
  double last_error;
  int iteration = 1;
  int RMarker = 0;
  double errorSum=0;
  double derivative = 0;
  double correction;
  int case_;

	while(1){ //infinte loop
    // Read Inputs
    case_ = detectMarkers();
    error = line_Position();

    // If escape error value is returned, assume line is in prev. direction
    if (error == 10){
      error = last_error;
    }

    // errorSum used for straight detection
    errorSum = errorSum + error;

    // On left marker reset straight detection variables
    if(case_ == 1 || iteration>3000){
      iteration = 1;
      errorSum = 0;
    }

    // On right marker count up to stop marker. Stop on stop marker
    if(case_ == 2 || case_ == 3){
      if(RMarker == 2*intersections +1){
        stopMotors();
        _delay_ms(3000);
        RMarker = 0;
      }
      else{
        RMarker++;
      }
    }

    // Check for straights based on average error value
    if(iteration > 50 && errorSum/(double)iteration > -0.5  &&  errorSum/(double)iteration < 0.5 ){
      PORTB |= (1<<2);
      RIGHT_BASE_SPEED =RIGHT_FAST_SPEED;
      LEFT_BASE_SPEED =LEFT_FAST_SPEED;
    }
    else {
      PORTB &= ~(1<<2);
      RIGHT_BASE_SPEED =RIGHT_SLOW_SPEED;
      LEFT_BASE_SPEED =LEFT_SLOW_SPEED;
    }

    // Calculate and perform error correction (PD Control)
    derivative = error - last_error;
    last_error = error;
    correction = Kp*error + Kd*derivative + Ki*integral;

    correctError(correction);

    iteration++;
	}
}
