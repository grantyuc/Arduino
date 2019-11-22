#include <MeetAndroid.h>

MeetAndroid meetAndroid;

const int leftMotor = 5;
const int rightMotor = 9;
const int minMotor = 90; 

void driveMotor(int thisPin, double power){
  if (power < 0) {
    analogWrite(thisPin, 0);
    thisPin += 1;
    power *= -1;
  }
  else {
    analogWrite(thisPin+1, 0);
  }
  if (power > 255) {
    power = 255;
  }
  if (power > 30 && power < minMotor+10)
    power = map(power, 80, minMotor+10, minMotor-20, minMotor+10);
  analogWrite(thisPin, power);
}

void setup(){
 pinMode(5, OUTPUT); 
 pinMode(6, OUTPUT); 
 pinMode(9, OUTPUT); 
 pinMode(10, OUTPUT); 
 Serial1.begin(57600);
 Serial.begin(9600);
 meetAndroid.registerFunction(accelerationDrive, 'A');
}

void loop(){
  meetAndroid.receive();
}

void accelerationDrive(byte flag, byte numOfValues)
{
  float data[numOfValues];
  meetAndroid.getFloatValues(data);
  
  float theta =  180 / PI * asin( data[2] / sqrt( data[0]*data[0] + data[1]*data[1] + data[2]*data[2] ) ) ;
  float phi = 180 / PI * asin( data[0] / sqrt( data[0]*data[0] + data[1]*data[1] ) ); 
  
  if (theta > 80 || theta < -30) {theta = 25; phi = 0;}
  
  int velocity = map(theta, -20, 70, -255, 255);
  int turn = map(phi, -50, 50, -150, 150);
  
  if (velocity < 0) turn = -turn;
  
  driveMotor(leftMotor, velocity-turn );
  driveMotor(rightMotor, velocity+turn );
  
//  for(int i=0; i<3; i++)
//    Serial.println(data[i]);
//  Serial.println(phi);
//  Serial.println(theta);
//  Serial.println("---");
}
