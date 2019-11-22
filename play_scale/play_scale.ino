#include "pitches.h"

// constants won't change. They're used here to 
// set pin numbers:
const int buttonPin = 2;     // the number of the pushbutton pin
const int buzzerPin = 13;      // the number of the LED pin
const int voltInPin = 9;

const int halfScale[25]={262, 277, 294, 311, 330, 349, 370, 392, 415, 440, 466, 494, 523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988, 1047};
const int majorScale[15]={NOTE_C4,NOTE_D4,NOTE_E4,NOTE_F4,NOTE_G4,NOTE_A4,NOTE_B4,NOTE_C5,NOTE_D5,NOTE_E5,NOTE_F5,NOTE_G5,NOTE_A5,NOTE_B5,NOTE_C6};
int index=0;

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

double freq = 0;
double lowFreq = NOTE_C4, highFreq = NOTE_C6;
double m = pow(highFreq / lowFreq, 1.0/1023);

void setup() {
  Serial.begin(9600);
  // initialize the LED pin as an output:
  pinMode(buzzerPin, OUTPUT);      
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(voltInPin, INPUT);  
}

void loop(){
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  //freq = lowFreq * pow(m, analogRead(voltInPin));                           //log scale
  freq = map(analogRead(voltInPin), 0, 1023, lowFreq, highFreq);            //normal scale
    
  index = (freq - lowFreq-1)*15/(highFreq - lowFreq);                       //major scale
  //index = (freq - lowFreq-1)*25/(highFreq - lowFreq);                       //half scale
    
  Serial.println(index+1);
    
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == LOW) {
    tone(buzzerPin, majorScale[index]);                                     //major scale
    //tone(buzzerPin, halfScale[index]);                                      //half scale
    //tone(buzzerPin, freq);                                                  //continuous  
    delay(10);
  } 
  else {
    noTone(buzzerPin); 
  }
}
