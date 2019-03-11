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
//#define something         #
#define IRreciever          9
#define wheelEncoder        10
#define LED                 11

Servo tilt, grip;   //pins 12, 13

//----------------------------------ANALOG PINS
#define sensorL             0
#define sensorR             1
#define sensorC             2
#define forceBall           3
#define range               4


//----------------------------------DEFINITIONS
int ballData[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //the corresponding zero to ball# will change once completed
#define LTHRESH 850
#define CTHRESH 850
#define RTHRESH 850

byte left_speed;                    //speed stored on each robot for going straight
byte right_speed;

int inters = 0;


//----------------------------------BASIC FUNCTIONS
void forward(){
  digitalWrite(Ldirection, HIGH);
  digitalWrite(Rdirection, HIGH);
  analogWrite(Lspeed, left_speed);
  analogWrite(Rspeed, right_speed);
  Serial.println("Forward");
}

void stopped(){
  analogWrite(Lspeed, 0);
  analogWrite(Rspeed, 0);
    Serial.println("Stopped");

}

void backwards(){
  digitalWrite(Ldirection, LOW);
  digitalWrite(Rdirection, LOW);
  analogWrite(Lspeed, left_speed);
  analogWrite(Rspeed, right_speed);
  Serial.println("Backwards");

}

void pivot(char direction, int degrees){            //pivot function
  int counter = 0;
  switch (degrees){
    case 90:                     //left rotation
      if (direction == 'L'){
        while (counter < 25){
          digitalWrite(Ldirection, LOW);
          digitalWrite(Rdirection, HIGH);
          analogWrite(Lspeed, left_speed);
          analogWrite(Rspeed, right_speed);

          int LwheelVal = digitalRead(wheelEncoder);
          counter = counter + LwheelVal;
          Serial.print("Counter: ");
          Serial.println(counter);
        }
        counter = 0;

      } else if (direction == 'R'){      //right rotation
        while (counter < 25) {
          digitalWrite(Ldirection, HIGH);
          digitalWrite(Rdirection, LOW);
          analogWrite(Lspeed, left_speed);
          analogWrite(Rspeed, right_speed);

          int LwheelVal = digitalRead(wheelEncoder);
          counter = counter + LwheelVal;
          Serial.print("Counter: ");
          Serial.println(counter);
        }
        counter = 0;
      } else {
        Serial.println("Invalid direction");
      }
      break;
    case 180:
      while (counter < 145){
        digitalWrite(Ldirection, HIGH);
        digitalWrite(Rdirection, LOW);
        analogWrite(Lspeed, left_speed);
        analogWrite(Rspeed, right_speed);

        int LwheelVal = digitalRead(wheelEncoder);
        counter = counter + LwheelVal;
        Serial.print("Counter: ");
        Serial.println(counter);
      }
      break;
  }
}


//----------------------------------COMPLEX FUNCTIONS
void grabBall(){
  stopped();
  tilt.write(46);
  for(int i=90;i<166;i++){
    Serial.println(i);
    grip.write(i);
    delay(50);
  }
  delay(4000);
}



void depositBall(){
  stopped();
  tilt.write(58);
  for(int i=166;i>89;i--){
    Serial.println(i);
    grip.write(i);
    delay(50);
  }
  delay(4000);
}


int detectIntersection(){
  if((analogRead(sensorL) > LTHRESH) && (analogRead(sensorC) > CTHRESH) && (analogRead(sensorR) > RTHRESH)) {
      Serial.println("Intersection detected!");
      digitalWrite(LED,HIGH);
      delay(200);               //this delay will change with contrast
      stopped();
      digitalWrite(LED,LOW);
      delay(2000);
      forward();
      delay(300);
      return 1;
  } 
}

void checkBall(){
  if(!digitalRead(Lbumper) || !digitalRead(Rbumper)){
      grabBall();
      Serial.println("grabbing ball");
      backwards();
      delay(1000);
      pivot('E',180);
  }
}

int followLine(){
  while(1){

    checkBall();


    
    delay(400);
    while(analogRead(sensorR) > RTHRESH){     //veering left
      Serial.println("Left loop");
      Serial.println("Left:");
      Serial.println(analogRead(sensorL));
      Serial.println("Center");
      Serial.println(analogRead(sensorC));
      Serial.println("Right:");
      Serial.println(analogRead(sensorR));
      
      digitalWrite(Ldirection, HIGH);
      digitalWrite(Rdirection, HIGH);
      analogWrite(Lspeed, left_speed);
      analogWrite(Rspeed, 0);
      inters = detectIntersection();
      if(inters == 1){
        inters = 0;
        return 1;
        break;
      }
      checkBall();
    }

    while(analogRead(sensorL) > LTHRESH){     //veering right
      Serial.println("right loop");
      Serial.println("Left:");
      Serial.println(analogRead(sensorL));
      Serial.println("Center");
      Serial.println(analogRead(sensorC));
      Serial.println("Right:");
      Serial.println(analogRead(sensorR));
      digitalWrite(Ldirection, HIGH);
      digitalWrite(Rdirection, HIGH);
      analogWrite(Lspeed, 0);
      analogWrite(Rspeed, right_speed);
      inters = detectIntersection();
      if(inters == 1){
        inters = 0;
        return 1;
        break;
      }
      checkBall();
    }
     int inters = detectIntersection();
     if(inters == 1){
        inters = 0;
        return 1;
        break;
     }
     forward();
  }  
}



//----------------------------------SETUP
void setup() {
  Serial.begin(9600);
  
  pinMode(LED, OUTPUT);
  IRserial.attach(9, -1);

  pinMode(Lbumper, INPUT);
  pinMode(Rbumper, INPUT);
  pinMode(Lspeed, OUTPUT);
  pinMode(Ldirection, OUTPUT);
  pinMode(Rspeed, OUTPUT);
  pinMode(Rdirection, OUTPUT);
  pinMode(wheelEncoder, INPUT);

  tilt.attach(12);
  grip.attach(13);
  tilt.write(46);
  left_speed = 100;               //temporary for troubleshooting
  right_speed = 100;
 // left_speed = EEPROM.read(0); //this is how it should be
  //right_speed = EEPROM.read(1);
}


//----------------------------------LOOP
void loop() {

//  line stuff that works
  int distance = distance +  followLine();
  if(distance == 3){
    pivot('L',90);
    distance = 0;
    depositBall();
  }
  checkBall();
  
  
}
