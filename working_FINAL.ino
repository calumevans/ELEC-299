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
#define range               4


//----------------------------------DEFINITIONS
int ballData[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //the corresponding zero to ball# will change once completed
#define LTHRESH 850
#define CTHRESH 850
#define RTHRESH 850

byte left_speed;                    //speed stored on each robot for going straight
byte right_speed;

int inters = 0;


//------------------------------------------------------BASIC FUNCTIONS
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
  int counter = 0;                                  //this function is VERY dependant on battery charge level
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
      while (counter < 110){
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


//------------------------------------------------------COMPLEX FUNCTIONS
//----------------------------------------------SEQUENCE
/*
int identifyStartingPosition(){         //left = 1, centre = 2, right = 3
    int startingPosition = 0;
    Serial.print("Identifying starting position...");

    while(startingPosition == 0){                     //constantly checking for character
      switch(char(IRserial.receive(200))){
          case 'L':
            Serial.println("Left");
            startingPosition = 1;
            break;
          case 'C':
            Serial.println("Center");
            startingPosition = 2;
            break;
          case 'R':
            Serial.println("Right");
            startingPosition = 3;
            break;
      }
      delay(80);
    }
  doubleBlink();
  return startingPosition;
}

void sequence(int location){                      //the sequence the robot follows
  location = identifyStartingPosition();
  Serial.print("Doing sequence #");
  Serial.println(location);
  switch(location){
    case 1:            //left robot: 7,1,13,4,6
      getBall(7);
      doubleBlink();
      
      getBall(1);
      doubleBlink();
      
      getBall(13);
      doubleBlink();
      
      getBall(4);
      doubleBlink();
      
      getBall(6);
      doubleBlink();
      break;  
    case 2:             //centre robot: 8,2,14,5,11
      getBall(8);
      doubleBlink();
      
      getBall(2);
      doubleBlink();
      
      getBall(14);
      doubleBlink();
      
      getBall(5);
      doubleBlink();
      
      getBall(11);
      doubleBlink();
      break;
    case 3:           //right robot: 9,3,15,10,12
      getBall(9);
      doubleBlink();
      
      getBall(3);
      doubleBlink();
      
      getBall(15);
      doubleBlink();
      
      getBall(10);
      doubleBlink();
      
      getBall(12);
      doubleBlink();
      break;      
  }
  Serial.print("Finished sequence!");
  Serial.println(ballData);
}
*/

//----------------------------------------------BALLS
void grabBall(){
  stopped();
  tilt.write(46);
  for(int i=90;i<166;i++){
    Serial.println(i);
    grip.write(i);
    delay(50);
  }
  delay(3000);
}

void depositBall(){
  stopped();
  tilt.write(58);
  for(int i=166;i>89;i--){
    Serial.println(i);
    grip.write(i);
    delay(50);
  }
  delay(3000);
}

void checkBall(){
  if(!digitalRead(Lbumper) || !digitalRead(Rbumper)){
      grabBall();
      Serial.println("grabbing ball");
      turnAround();
  }
}

/*void getBall(int ballNum){
  Serial.print("Ball #");
  Serial.print(ballNum);
  Serial.println(" is being captured");  
  switch(ballNum){            //there are 15 balls in this competition, this is getting the coordinates for the balls and using pos() to travel
    case 1:
      pos(-3,-2);
      checkBall();
      pos(-1,-3);   /go to bin
      break;
    case 2:
      pos(-3,-1);
      break;
    case 3:
      pos(-3,0);
      break;
    case 4:
      pos(-3,1);
      break;
    case 5:
      pos(-3,2);
      break;
    case 6:
      pos(-2,3);
      break;
    case 7:
      pos(-1,3);
      break;
    case 8:
      pos(0,3);
      break;
    case 9:
      pos(1,3);
      break;
    case 10:
      pos(2,3);
      break;
    case 11:
      pos(3,2);
      break;
    case 12:
      pos(3,1);
      break;
    case 13:
      pos(3,0);
      break;
    case 14:
      pos(3,-1);
      break;
    case 15:
      pos(3,-2);
      break;
    default:
      Serial.println("Invalid ball number");
      break; v
  }
ballData[ballNum-1] = 1;          //updates the array to show each ball that is captured (denoted with a 1)
}

*/


//----------------------------------------------NAVIGATION
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

void turnAround(){        //used for after the robot hits a wall (grabbing or depositing a ball)
  backwards();
  delay(1000);
  pivot('E',180); 
}

//----------------------------------------------MISCELLANEOUS
void startButton(){
  Serial.println("Waiting for start button");
  while(digitalRead(button)){}                  //stuck in a loop until button is pressed
  Serial.println("Start button pressed!");
}

void doubleBlink(){
  digitalWrite(LED,HIGH);
  delay(50);
  digitalWrite(LED,LOW);
  delay(50);
}


//----------------------------------------------SETUP
void setup() {
  Serial.begin(9600);
  
  IRserial.attach(9, -1);

  pinMode(Lbumper, INPUT);
  pinMode(Rbumper, INPUT);
  pinMode(Lspeed, OUTPUT);
  pinMode(Ldirection, OUTPUT);
  pinMode(Rspeed, OUTPUT);
  pinMode(Rdirection, OUTPUT);
  pinMode(wheelEncoder, INPUT);
  pinMode(LED, OUTPUT);     //this LED will turn on when an intersection has been detected, and when completes a sequence
  pinMode(button, INPUT);

  //-----------------------
  //startButton();            //program will not continue until button is pressed
  //-----------------------
  
  tilt.attach(12);
  grip.attach(13);
  tilt.write(50);       //the correct height for the arm
  
  left_speed = 100;               //temporary for troubleshooting
  right_speed = 100;
 // left_speed = EEPROM.read(0); //this is how it should be
  //right_speed = EEPROM.read(1);
}


//----------------------------------------------LOOP
void loop() {

//  line stuff that works
  int distance = distance +  followLine();
  Serial.println(distance);
  if(distance == 2){
    pivot('L',90);
    depositBall();
  }
  checkBall();            //currently, checkBall is called in multiple places because the program may be stuck in a loops somewhere
  
  
}
