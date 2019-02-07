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

void WaitButton(){
  if(!digitalRead(button)){
    Serial.println("Start button pressed");
    return;
  }else{
    WaitButton();
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

void Backup(){
    digitalWrite(Ldirection, LOW);
    digitalWrite(Rdirection, LOW);
    analogWrite(Lspeed, left_speed);
    analogWrite(Rspeed, right_speed);
}

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
      analogWrite(Lspeed, 140);
      analogWrite(Rspeed, 140);
  
      int LwheelVal = digitalRead(Lencoder);
      counter = counter + LwheelVal;
      Serial.print("Counter: ");
      Serial.println(counter);
      }
      counter = 0;
    }  
  }else if(direction == 'E'){
    if(degrees == 180){
      while(counter < 35){
        digitalWrite(Ldirection, HIGH);
        digitalWrite(Rdirection, LOW);
        analogWrite(Lspeed, 110);
        analogWrite(Rspeed, 110);
  
        int LwheelVal = digitalRead(Lencoder);
        counter = counter + LwheelVal;
        Serial.print("Counter: ");
        Serial.println(counter);
      }
    }
  }
}

void ReactBumpers(){
  if(!digitalRead(Lbumper)){
    delay(500);
    if(!digitalRead(Rbumper)){
      Backup();
      delay(500);
      Pivot('E',180);
    }else{
       Backup();
       delay(700);
       Pivot('R',45);
    }
  }else if(!digitalRead(Rbumper)){
    delay(500);
    if(!digitalRead(Lbumper)){
      Backup();
      delay(500);
      Pivot('E',180);
    }else{
       Backup();
       delay(700);
       Pivot('L',45);
    }
  }
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
 
  WaitButton();                 //initial button to turn it on
}

void loop(){
  Forward();
  ReactBumpers();
}
