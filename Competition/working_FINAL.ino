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
#define LED                 10

Servo tilt, grip;   //pins 12, 13

//----------------------------------ANALOG PINS
#define sensorL             0
#define sensorR             1
#define sensorC             2

//----------------------------------DEFINITIONS
#define LTHRESH 850
#define CTHRESH 850
#define RTHRESH 850

byte left_speed = 109;
byte right_speed = 110;
char grabOrDeposit = 'G';         //global variable which charges based on if robot is grabbing/depositing
char startingPosition = '9';      //arbitrary number

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

void pivot(char direction, int degrees){            //UPDATED pivot function
  switch (degrees){
    case 90:                     //left rotation
      if (direction == 'L'){
          Serial.println("Pivot left");
          digitalWrite(Ldirection, LOW);
          digitalWrite(Rdirection, HIGH);
          analogWrite(Lspeed, left_speed);
          analogWrite(Rspeed, right_speed);
          delay(250);
        while(analogRead(sensorL) < LTHRESH){
          digitalWrite(Ldirection, LOW);
          digitalWrite(Rdirection, HIGH);
          analogWrite(Lspeed, left_speed);
          analogWrite(Rspeed, right_speed);
        }
        delay(190);                                            //to make it turn futher
      } else if (direction == 'R'){      //right rotation
          digitalWrite(Ldirection, HIGH);
          digitalWrite(Rdirection, LOW);
          analogWrite(Lspeed, left_speed);
          analogWrite(Rspeed, right_speed);
          delay(250);
        while(analogRead(sensorR) < RTHRESH){
        digitalWrite(Ldirection, HIGH);
          digitalWrite(Rdirection, LOW);
          analogWrite(Lspeed, left_speed);
          analogWrite(Rspeed, right_speed);
        }
        delay(250);                                            //to make it turn futher
      } else {
        Serial.println("Invalid direction");
      }
      break;
    case 180:
      digitalWrite(Ldirection, HIGH);
      digitalWrite(Rdirection, LOW);
      analogWrite(Lspeed, left_speed);
      analogWrite(Rspeed, right_speed);
      delay(1400);                                           //to make it turn futher, battery level dependant
      while(analogRead(sensorR) < RTHRESH){
        digitalWrite(Ldirection, HIGH);
        digitalWrite(Rdirection, LOW);
        analogWrite(Lspeed, left_speed);
        analogWrite(Rspeed, right_speed);
      }
      break;
  }
}

//------------------------------------------------------COMPLEX FUNCTIONS
//----------------------------------------------SEQUENCES
char identifyStartingPosition(){         //left = 1, centre = 2, right = 3
    Serial.print("Identifying starting position...");
    while(startingPosition == '9'){                     //constantly checking for new character
      switch(IRserial.receive(200)){
          case '0':
            Serial.println("Left");
            startingPosition = '0';
            break;
          case '1':
            Serial.println("Center");
            startingPosition = '1';
            break;
          case '2':
            Serial.println("Right");
            startingPosition = '2';
            break;
      }
      delay(80);
    }
  doubleBlink();
  return startingPosition;
}

void sequence(){                      //sequence based on starting position
  char location = identifyStartingPosition();
  Serial.print("Doing sequence #");
  Serial.println(location);
  switch(location){
    case '0':             //left robot: 7,1,13,4,6
     //ball 7
      UCmove(10);
      doubleBlink();

      //ball 1
      stopped();
      delay(200);
      pivot('L',90);
      UCmove(2);
      stopped();
      delay(200);
      pivot('R',90);
      
      //ball 13
      UCmove(3);
      stopped();
      delay(200);
      pivot('R',90);
      UCmove(6);
      stopped();
      delay(200);
      pivot('L',90);

      //ball 4
      UCmove(6);
      stopped();
      delay(200);
      pivot('L',90);
      UCmove(2);
      stopped();
      delay(200);
      pivot('R',90);
      UCmove(4);

      //ball 6
      stopped();
      delay(200);
      pivot('L',90);
      UCmove(1);
      stopped();
      delay(200);
      pivot('R',90);
      UCmove(8);
      stopped();
      delay(200);
      pivot('L',90);
      UCmove(1);
      stopped();
      delay(200);
      pivot('R',90);
      UCmove(1);        //ends on the first intersection
      party();
      break;  
    case '1':             //centre robot: 8,2,14,5,11
      //ball 8
      UCmove(11);
      
      //ball 2
      stopped();
      delay(200);
      pivot('L',90);
      UCmove(4);
      stopped();
      delay(200);
      pivot('R',90);
      UCmove(3);
      
      //ball 14
      stopped();
      delay(200);
      pivot('R',90);
      UCmove(4);
      stopped();
      delay(200);
      pivot('L',90);
      UCmove(6);
      
      //ball 5
      stopped();
      delay(200);
      pivot('L',90);
      UCmove(4);
      stopped();
      delay(200);
      pivot('R',90);
      UCmove(9);
      
      //ball 11
      stopped();
      delay(200);
      pivot('R',90);
      UCmove(4);
      stopped();
      delay(200);
      pivot('L',90);
      UCmove(5);
      party();
      break;
    case '2':             //right robot: 9,3,15,10,12
      //ball 9
      UCmove(12);
      
      //ball 3
      stopped();
      delay(200);
      pivot('L',90);
      UCmove(6);                          //6
      stopped();
      delay(200);
      pivot('R',90);
      UCmove(3);
      
      //ball 15
      stopped();
      delay(200);
      pivot('R',90);
      UCmove(2);
      stopped();
      delay(200);
      pivot('L',90);
      UCmove(1);
      
      //ball 10
      stopped();
      delay(200);
      pivot('R',90);
      UCmove(1);
      stopped();
      delay(200);
      pivot('L',90);
      UCmove(8);
      stopped();
      delay(200);
      pivot('R',90);
      UCmove(1);
      stopped();
      delay(200);
      pivot('L',90);
      UCmove(4);
      
      //ball 12
      stopped();
      delay(200);
      pivot('R',90);
      UCmove(2);
      stopped();
      delay(200);
      pivot('L',90);
      UCmove(4);
      party();
      break;      
  }
  Serial.print("Finished!!!!");
}

//----------------------------------------------BALLS
void grabBall(){
  stopped();
  tilt.write(46);
  for(int j=0;j<2;j++){
    for(int i=40;i<180;i++){
      grip.write(i);
      delay(30);
    }
  }
  tilt.write(170);
  delay(500);
}

void depositBall(){
  stopped();
  tilt.write(180); //goes highest
  delay(300);
  tilt.write(100);
  grip.write(89);
  tilt.write(50);
  delay(100);
  tilt.write(170);
}

void checkWall(){     //depending on the global variable 'grabOrDeposit', a ball will be grabbed or deposited
  if(!digitalRead(Lbumper) || !digitalRead(Rbumper)){
    stopped();
    if(grabOrDeposit == 'G'){          //for grabbing the ball
      backwards();
      delay(300);
      grabBall();
      turnAround();
    }else if(grabOrDeposit == 'D'){    //for depositing the ball
      depositBall();
      turnAround();
    }
    if(grabOrDeposit == 'G'){          //to toggle between grabbing and depositing
      grabOrDeposit = 'D';
    }else{ 
      grabOrDeposit = 'G';
    }
  }
}

//----------------------------------------------NAVIGATION
void UCmove(int L){  //for moving 'L' units
  int UC = 0;
  while(UC < L){
    UC = UC + followLine();
  }
}

int detectIntersection(){
    if((analogRead(sensorL) > LTHRESH) && (analogRead(sensorC) > CTHRESH) && (analogRead(sensorR) > RTHRESH)) {
        Serial.println("Intersection detected!");
        digitalWrite(LED,HIGH);
        delay(350);                                 //how far past the intersection we want it to go
        digitalWrite(LED,LOW);
        return 1;
     }else{
        return 0;
  }
}

int followLine(){
  Serial.println(detectIntersection());
  bluetoothEmergency();
  while(!detectIntersection()){
    checkWall(); 
    if(analogRead(sensorR) > RTHRESH){     //veering left
      Serial.println("veering left");
      analogWrite(Lspeed, left_speed);      
      analogWrite(Rspeed, 0);
      checkWall();
    }else if(analogRead(sensorL) > LTHRESH){     //veering right
      Serial.println("veering right");
      analogWrite(Lspeed, 0);
      analogWrite(Rspeed, right_speed);
      checkWall();
    }else{
      Serial.println("perfectly on line");
      forward();
    }
    Serial.println("forward outside of loop");
    forward();
  }
  return 1;       //when there is intersection
}

void turnAround(){   //used for after the robot hits a wall (grabbing or depositing a ball)
  backwards();
  delay(900);             //how far back from the wall
  pivot('E',180);
}

//----------------------------------------------MISCELLANEOUS
void doubleBlink(){               //used to show that things happen
  for(int i=0;i<2;i++){
    digitalWrite(LED,HIGH);
    delay(50);
    digitalWrite(LED,LOW);
    delay(50);
  }
}

void bluetoothEmergency(){
  //for EPBMX, the serial.begin must be changed to "115200"
  int completion = 0;
  while(Serial.available() && (completion == 0)){         //completion goes to 1 when a task is completed
    int code = Serial.read();
    Serial.print("BT Number Recieved: ");
    Serial.println(int(code));
    
    switch(code){
      case 83:            //pressed S, make the robot stop for 3 seconds
        Serial.println("EMERGENCY STOP: 3 seconds");
        stopped();
        delay(3000);
        completion++;
        break;
      case 76:            //pressed L, make the robot swerve left
        Serial.println("EMERGENCY: swerve left");
        digitalWrite(Rdirection, HIGH);
        analogWrite(Lspeed,0);
        analogWrite(Rspeed, right_speed);
        delay(500);
        completion++;
        break;
      case 82:            //pressed R, make the robot swerve right
        Serial.println("EMERGENCY: swerve right");
        digitalWrite(Ldirection, HIGH);
        analogWrite(Rspeed,0);
        analogWrite(Lspeed, left_speed);
        delay(500);
        completion++;
        break;
    }
  }
  completion = 0;
}

void party(){
  stopped();
  while(1){
    for(int i=40;i<180;i++){
      grip.write(i);
      delay(4);
    }
  }
}
  
//----------------------------------------------SETUP
void setup() {
  Serial.begin(9600);       //115200 for bluetooth module
 
  IRserial.attach(9, -1);
  pinMode(Lbumper, INPUT);
  pinMode(Rbumper, INPUT);
  pinMode(Lspeed, OUTPUT);
  pinMode(Ldirection, OUTPUT);
  pinMode(Rspeed, OUTPUT);
  pinMode(Rdirection, OUTPUT);
  pinMode(LED, OUTPUT);     //this LED will turn on when an intersection has been detected, and when a sequence is completed
  pinMode(button, INPUT);

  tilt.attach(12);
  grip.attach(13);
  tilt.write(170);       //the correct height for the arm
}

//----------------------------------------------LOOP
void loop(){
  sequence();
}
