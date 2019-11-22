void setup(){
  Serial.begin(9600);
}

void loop(){
  int a = 1;
  auto p = [a](int x){Serial.println(a + x);} ;
}

