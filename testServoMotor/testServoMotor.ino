#include <Servo.h>

Servo motor;
#define MOTOR_PIN 10
void setup() {
  // put your setup code here, to run once:
  motor.attach(MOTOR_PIN);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int mpower = 1480;
  String str;
  if (Serial.available() > 0) { 
    String inString;
    while (Serial.available() > 0) {
      // read the incoming byte:
      char inChar = Serial.read();
      if (isDigit(inChar)) {
          // convert the incoming byte to a char
          // and add it to the string:
          inString += (char)inChar;
      }
      str = inString;
    }
  }
  mpower = str.toInt();
  // say what you got:
  if(mpower > 1200 && mpower < 1800){
    Serial.print("I received: ");
    Serial.println(mpower);
    motor.writeMicroseconds(mpower);
  }
  delay(500);
}
