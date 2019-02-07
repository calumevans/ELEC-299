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
#define IRrange 5

#define THRESHHOLD 220

byte left_speed;
byte right_speed;


void Pivot(char direction, int degrees){      //pivot function
  int counter = 0;
  if(direction == 'L'){                     //left rotation
    if(degrees == 45){
      while(counter < 20){
      digitalWrite(Ldirection, LOW);
      digitalWrite(Rdirection, HIGH);
      analogWrite(Lspeed, 140);
      analogWrite(Rspeed, 140);
  
      int LwheelVal = digitalRead(Lencoder);
      counter = counter + LwheelVal;
      Serial.print("Counter: ");
      Serial.println(counter);
      }
      counter = 0;
    }
    
  }else if(direction == 'R'){           //right rotation
    if(degrees == 45){
      while(counter < 20){
      digitalWrite(Ldirection, HIGH);
      digitalWrite(Rdirection, LOW);
      analogWrite(Lspeed, left_speed + 40);
      analogWrite(Rspeed, right_speed);
  
      int LwheelVal = digitalRead(Lencoder);
      counter = counter + LwheelVal;
      Serial.print("Counter: ");
      Serial.println(counter);
      }
      counter = 0;
    }  
  }else if(direction == 'E'){
    if(degrees == 180){
      while(counter < 45){
        digitalWrite(Ldirection, HIGH);
        digitalWrite(Rdirection, LOW);
        analogWrite(Lspeed, 220);
        analogWrite(Rspeed, 140);
  
        int LwheelVal = digitalRead(Lencoder);
        counter = counter + LwheelVal;
        Serial.print("Counter: ");
        Serial.println(counter);
      }
    }
  }
}

void Forward(){
    digitalWrite(Ldirection, HIGH);
    digitalWrite(Rdirection, HIGH);
    analogWrite(Lspeed, left_speed);
    analogWrite(Rspeed, right_speed);
}

void Stop(){
    digitalWrite(Ldirection, HIGH);
    digitalWrite(Rdirection, HIGH);
    analogWrite(Lspeed, 0);
    analogWrite(Rspeed, 0);
}

void CheckDistance(){
  delay(500);
  if(analogRead(IRrange) < THRESHHOLD){
    Forward();
  }else if(analogRead(IRrange) >= THRESHHOLD){
    Stop();
    delay(300);
    Backup();
    delay(700);
    Pivot('E',180);
    Forward();
    delay(800);
    Stop();
    delay(3000);
  } 
}

void Backup(){
    digitalWrite(Ldirection, LOW);
    digitalWrite(Rdirection, LOW);
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
}

void loop() {
  CheckDistance();
  Serial.println(analogRead(IRrange));
}
