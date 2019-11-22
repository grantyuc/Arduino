float lastTime = 0.;
int highTime = 500;
float freq = 700;
float period = 1000000.0/freq;

void setup(){
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}

void loop(){
  float thisTime = millis();
  for(highTime = 50; highTime<period; highTime+=period/500){
    while(thisTime-lastTime < 10){
      thisTime = millis();
      digitalWrite(13,HIGH);
      delayMicroseconds(highTime);
      digitalWrite(13,LOW);
      delayMicroseconds(period - highTime);
      Serial.println(highTime/period);
    }
    lastTime = thisTime;
  }
}
