/*
  used for telling the robot which starting position it has
*/
#include <QSerial.h>

QSerial IRserial;

void setup(){
  Serial.begin(9600);
  IRserial.attach(-1,12);
}

void loop(){
  IRserial.transmit(1);             //change this so 1,2,3
  delay(100);
}
