// ================================================================
// Leonardo Pin Info
// TWI: 2 (SDA) and 3 (SCL). Support TWI communication using the Wire library.
// External Interrupts: 3 (interrupt 0), 2 (interrupt 1), 0 (interrupt 2), 1 (interrupt 3) and 7 (interrupt 4).
// ================================================================

#include <Servo.h>
#include "DataRing.h"

// Declare servo
Servo dsmotor, ebmotor, upmotor; // disc and elbow resp.

// Declare pwm buffer
DataRing<int> p1;
  
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define IMOTOR_PIN 10
#define OMOTOR_PIN 11
#define UPMOTOR_PIN 5
//#define DCMOTOR_PIN 5

// CH2 characteristics
#define UDCENTER (DOWN_MOST + UP_MOST) * .5
#define DOWN_MOST 1300
#define DOWN_TOLER 1490
#define UP_TOLER 1560
#define UP_MOST 1770

// CH4 characteristics
#define LRCENTER (LEFT_MOST + RIGHT_MOST) * .5
#define LEFT_MOST 1260
#define LEFT_TOLER 1420
#define RIGHT_TOLER 1620
#define RIGHT_MOST 1780

// CH1 characteristics
#define R_LRCENTER (R_LEFT_MOST + R_RIGHT_MOST) * .5
#define R_LEFT_MOST 1215
#define R_LEFT_TOLER 1440
#define R_RIGHT_TOLER 1520
#define R_RIGHT_MOST 1715

// CH3 characteristics
#define THROTTLE_MIN_TOLER THCENTER - 100
#define THROTTLE_MAX_TOLER THCENTER + 100
#define THCENTER (THROTTLE_MAX + THROTTLE_MIN) * .5
#define THROTTLE_MAX 1970
#define THROTTLE_MIN 1100
#define THROTTLE_OFF 1020

// Motor characteristics
#define OUT_MOTOR_PWM_MAX 1710
#define OUT_MOTOR_PWM_CENTER 1500
#define OUT_MOTOR_PWM_MIN 1290

#define IN_MOTOR_PWM_MAX 1800
#define IN_MOTOR_PWM_CENTER 1445
#define IN_MOTOR_PWM_MIN 1136

#define UP_MOTOR_PWM_MAX 1800
#define UP_MOTOR_PWM_CENTER 1476
#define UP_MOTOR_PWM_MIN 1200
#define minMotor 90

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile int pwm_value1 = 0;
volatile int prev_time1 = 0;
volatile int pwm_value2 = 0;
volatile int prev_time2 = 0;
volatile int pwm_value3 = 0;
volatile int prev_time3 = 0;

// ================================================================
// Leonardo Pin Info
// TWI: 2 (SDA) and 3 (SCL). Support TWI communication using the Wire library.
// External Interrupts: 3 (interrupt 0), 2 (interrupt 1), 0 (interrupt 2), 1 (interrupt 3) and 7 (interrupt 4).
//
// Uno Pin Info
// External Interrupts: 2 (interrupt 0) and 3 (interrupt 1).
// ================================================================
#define int1 digitalPinToInterrupt(2)
#define int2 digitalPinToInterrupt(3)
#define int3 digitalPinToInterrupt(7)

void rising1() {
  // Pin 0 (interrupt 2) is rising
  attachInterrupt(int1, falling1, FALLING);
  prev_time1 = micros();
}

void falling1() {
  // Pin 0 (interrupt 2) is falling
  attachInterrupt(int1, rising1, RISING);
  pwm_value1 = micros() - prev_time1;
}

void rising2() {
  // Pin 1 (interrupt 3) is rising
  attachInterrupt(int2, falling2, FALLING);
  prev_time2 = micros();
}

void falling2() {
  // Pin 1 (interrupt 3) is falling
  attachInterrupt(int2, rising2, RISING);
  pwm_value2 = micros() - prev_time2;
}

void rising3() {
  // Pin 2 (interrupt 1) is rising
  attachInterrupt(int3, falling3, FALLING);
  prev_time3 = micros();
}

void falling3() {
  // Pin 2 (interrupt 1) is falling
  attachInterrupt(int3, rising3, RISING);
  pwm_value3 = micros() - prev_time3;
}
// ================================================================
// ================================================================

//ISR (PCINT0_vect) {
//  // handle pin change interrupt for D8 to D13 here
//  int temp = micros() - prev_time3;
//  if (temp > 1000 && temp < 2000)
//    pwm_value3 = temp;
//  else
//    prev_time3 = micros();
//
//}  // end of PCINT0_vect

void setup() {
  pinMode(UPMOTOR_PIN, OUTPUT);
//  pinMode(DCMOTOR_PIN, OUTPUT);
//  pinMode(DCMOTOR_PIN + 1, OUTPUT);
  pinMode(9, INPUT_PULLUP);
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  // enable servo control
  dsmotor.attach(IMOTOR_PIN);
  ebmotor.attach(OMOTOR_PIN);
  upmotor.attach(UPMOTOR_PIN);

  // read PWM signals
  attachInterrupt(int1, rising1, RISING);    // int 2 = 0
  attachInterrupt(int2, rising2, RISING);    // int 3 = 1
  attachInterrupt(int3, rising3, RISING);    // int 1 = 2

//  // pin change interrupt (example for D9)
//  PCMSK0 |= bit (PCINT1);  // want pin 9
//  PCIFR  |= bit (PCIF0);   // clear any outstanding interrupts
//  PCICR  |= bit (PCIE0);   // enable pin change interrupts for D8 to D13

  // Communication with PC
  Serial.begin(9600);
}

void loop() {
  volatile bool isPwm1Ready = (pwm_value1 < 2000 && pwm_value1 > 1000) ? true : false;
  volatile bool isPwm2Ready = (pwm_value2 < 2000 && pwm_value2 > 1000) ? true : false;
  volatile bool isPwm3Ready = (pwm_value3 < 2000 && pwm_value3 > 1000) ? true : false;
  volatile bool isConnected = isPwm1Ready && isPwm2Ready && isPwm3Ready ;
  volatile bool isRingBufferReady = p1.isReady();
  volatile bool isReady = isConnected && isRingBufferReady;
  
  if( isConnected ){
    while( !isRingBufferReady ){
      p1.push(pwm_value1);
      isRingBufferReady = p1.isReady();
    }
//    while( true ){
//      Serial.print("Push ");         
//      Serial.print(pwm_value1);
//
//      Serial.print(" p1: ");
//      p1.print();
//
//      Serial.print(" avg: ");
//      Serial.print(p1.average());
//      Serial.print(" dev: ");
//      Serial.println(p1.stdev());
//      
//      p1.push(pwm_value1);
//    }
    
  }

  p1.push(pwm_value1);
  int p1avg = p1.average();

  Serial.print("Connected: ");
  Serial.print(isReady ? "T" : "F");
  Serial.print(" \t0: ");
  Serial.print(p1avg);
  Serial.print(" 1: ");
  Serial.print(pwm_value2);
  Serial.print(" 2: ");
  Serial.print(pwm_value3);
//  Serial.print(" prev_time3: ");
//  Serial.print(prev_time3);

  int dspower = 0, ebpower = 0, dcpower = 0;
  if ( isReady ) {
    dspower = map(zero(pwm_value2, LEFT_TOLER, RIGHT_TOLER, LRCENTER), LRCENTER, RIGHT_MOST, IN_MOTOR_PWM_CENTER, IN_MOTOR_PWM_MIN);

    //  if ebmotor is position control mode, use this
    //  ebpower = cmap(pwm_value1, THROTTLE_MIN, THROTTLE_MAX, OUT_MOTOR_PWM_MIN, OUT_MOTOR_PWM_MAX);
    //  if ebmotor is velocity control mode, use this
    ebpower = map(zero(p1avg, R_LEFT_TOLER, R_RIGHT_TOLER, R_LRCENTER), R_LRCENTER, R_RIGHT_MOST, OUT_MOTOR_PWM_CENTER, OUT_MOTOR_PWM_MIN);

    dcpower = map(zero(pwm_value3, DOWN_TOLER, UP_TOLER, UDCENTER), UDCENTER, UP_MOST, UP_MOTOR_PWM_CENTER, UP_MOTOR_PWM_MAX);
  }
  else {
    dspower = IN_MOTOR_PWM_CENTER;
    ebpower = OUT_MOTOR_PWM_CENTER;
    dcpower = UP_MOTOR_PWM_CENTER;
  }
  
  Serial.print("\t\tpower ds: ");
  Serial.print(dspower);
  Serial.print(" eb: ");
  Serial.print(ebpower);
  Serial.print(" dc: ");
  Serial.println(dcpower);

  dsmotor.writeMicroseconds( constrain(dspower, IN_MOTOR_PWM_MIN, IN_MOTOR_PWM_MAX) );
  ebmotor.writeMicroseconds( constrain(ebpower, OUT_MOTOR_PWM_MIN, OUT_MOTOR_PWM_MAX) );
  upmotor.writeMicroseconds( constrain(dcpower, UP_MOTOR_PWM_MIN, UP_MOTOR_PWM_MAX) );

  delay(5);
}

//void driveMotor(int thisPin, double power) {
//  if (power < 0) {
//    analogWrite(thisPin, 0);
//    thisPin += 1;
//    power *= -1;
//  }
//  else {
//    analogWrite(thisPin + 1, 0);
//  }
//  if (power > 255) {
//    power = 255;
//  }
//  //  if (power > 30 && power < minMotor+10)
//  //    power = map(power, 80, minMotor+10, minMotor-20, minMotor+10);
//  analogWrite(thisPin, power);
//}

inline int zero(int value, const int& a, const int& b, const int& c) {
  return (a < value && value < b) ? c : value;
}

//inline int cmap(int value, const int& a, const int& b, const int& c, const int& d) {
//  return map(constrain(value, a, b), a, b, c, d);
//}

