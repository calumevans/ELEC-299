// left should be 245, right should be 128
//our motors have very different speeds

#include <EEPROM.h>

#define button 3
#define Lbumper 4
#define Rbumper 5
#define Lspeed 6
#define Ldirection 7
#define Rspeed 8
#define Rdirection 9
#define Lencoder 10
#define Rencoder 11

byte left_speed;
byte right_speed;

void AdjustSpeeds(){
  if((!digitalRead(Lbumper)) || (!digitalRead(Rbumper))){
    delay(500);
  }
  if((!digitalRead(Lbumper)) && (digitalRead(Rbumper))){
    //left_speed --;
    right_speed = right_speed + 4;
  }else if((digitalRead(Lbumper)) && (!digitalRead(Rbumper))){
    left_speed = left_speed +4;
    //right_speed--;
  }else if(((!digitalRead(Lbumper)) && (!digitalRead(Rbumper)))){
     EEPROM.write(0, left_speed);
     EEPROM.write(1, right_speed);
     Serial.println("SAVED!!");
     delay(100);
     return;                //means we're going straight
  }
}

void Forward(){
    digitalWrite(Ldirection, HIGH);
    digitalWrite(Rdirection, HIGH);
    analogWrite(Lspeed, left_speed);
    analogWrite(Rspeed, right_speed);
}

void setup(){
  Serial.begin(9600);
  pinMode(button,INPUT);
  pinMode(Lbumper,INPUT);
  pinMode(Rbumper,INPUT);
  pinMode(Lspeed,OUTPUT);
  pinMode(Ldirection,OUTPUT);
  pinMode(Rspeed, OUTPUT);
  pinMode(Rdirection,OUTPUT);
  pinMode(Lencoder,INPUT);
  pinMode(Rencoder,INPUT);

  left_speed = EEPROM.read(0);
  right_speed = EEPROM.read(1);
  //left_speed = 140;
  //right_speed = 140;
}

void loop() {
  Serial.println(left_speed);
  Serial.println(right_speed);
  Forward();
  AdjustSpeeds();
}
