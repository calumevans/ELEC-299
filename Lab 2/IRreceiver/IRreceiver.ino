#include <QSerial.h>

QSerial IRserial;

void setup(){
  Serial.begin(9600);
  IRserial.attach(7,-1);
}


void loop(){
  int val = IRserial.receive(200);
  Serial.print(val);
  Serial.print(",");
  Serial.println((char)val);
  delay(100);
    
}

