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
  IRserial.transmit('L');             //change this so L, C, or R
  delay(100);
}
