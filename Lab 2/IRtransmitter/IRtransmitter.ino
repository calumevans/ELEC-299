#include <QSerial.h>

QSerial IRserial;

void setup(){
  Serial.begin(9600);
  IRserial.attach(-1,12);
}

void loop(){
  IRserial.transmit('3');
  delay(100);
}
