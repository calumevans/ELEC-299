
#include <EEPROM.h>
#include <Servo.h>
#include <QSerial.h>

QSerial IRserial;

//----------------------------------DIGITAL PINS
#define Lbumper             2
#define Rbumper             3
#define Ldirection          4
#define Lspeed              5
#define Rspeed              6
#define Rdirection          7
#define button              8
#define IRreciever          9
#define wheelEncoder        10
#define LED                 11

Servo tilt, grip;   //pins 12, 13

//----------------------------------ANALOG PINS
#define sensorL             0
#define sensorR             1
#define sensorC             2



void setup(){
  Serial.begin(9600);






}

void loop(){


Serial.print("Left/Centre/Right:  ");
      Serial.print(analogRead(sensorL));
      Serial.print(", ");
      Serial.print(analogRead(sensorC));
      Serial.print(", ");
      Serial.println(analogRead(sensorR));






}
