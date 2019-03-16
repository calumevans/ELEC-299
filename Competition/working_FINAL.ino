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
char grabOrDeposit = 'G';         //global variable which charges based on if robot is grabbing/depositing
int startingPosition = 0;

int X;
int Y;
int UC;
int OX,OY;
int X1,Y1,X2,Y2;

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
      while (counter < 180){
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

int identifyStartingPosition(){         //left = 1, centre = 2, right = 3
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
  startPosition();
  switch(location){
    case 1:             //left robot: 7,1,13,4,6
      getBall(7);    
      getBall(1);
      getBall(13);
      getBall(4);
      getBall(6);
      break;  
    case 2:             //centre robot: 8,2,14,5,11
      getBall(8);
      getBall(2);
      getBall(14);
      getBall(5);
      getBall(11);
      break;
    case 3:             //right robot: 9,3,15,10,12
      getBall(9);
      getBall(3);
      getBall(15);
      getBall(10);
      getBall(12);
      break;      
  }
  Serial.print("Finished sequence!");
//  Serial.println(ballData);
}


//----------------------------------------------BALLS
void grabBall(){
  stopped();
  tilt.write(46);
  for(int i=90;i<166;i++){
    //Serial.println(i);
    grip.write(i);
    delay(50);
  }
  delay(3000);
  resetPosition();
}

void depositBall(){
  stopped();
  tilt.write(58);
  for(int i=166;i>89;i--){
    //Serial.println(i);
    grip.write(i);
    delay(50);
  }
  delay(3000);
  resetPosition();
}

void checkWall(){
  if(!digitalRead(Lbumper) || !digitalRead(Rbumper)){
    if(grabOrDeposit == 'G'){          //for grabbing the ball
      grabBall();
    }else if(grabOrDeposit == 'D'){    //for depositing the ball
      depositBall();
    }
    turnAround();
  }
}

void getBall(int ballNum){
  Serial.print("Ball #");
  Serial.print(ballNum);
  Serial.println(" is being captured");  
  grabOrDeposit = 'G';
  switch(ballNum){            //there are 15 balls in this competition, this is getting the coordinates for the balls and using pos() to travel
    case 1:
      pos(-3,-2);
      checkWall();
      grabOrDeposit = 'D';
      pos(X1,Y1);   //go to bin
      checkWall();
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
      break;
  }
ballData[ballNum-1] = 1;          //updates the array to show each ball that is captured (denoted with a 1)
doubleBlink();
}



//----------------------------------------------NAVIGATION


void startPosition(){
  if (startingPosition == 1){
    X = -1;
    Y = -3;
  }else if(startingPosition == 2){
    X = 0;
    Y = -3;
  }else if(startingPosition == 3){
    X = 1;
    Y = -3;
  }
  OX = X;
  OY = Y;
}


void UCmove(int l){
  while(!UC){
    forward();
    UC = detectIntersection();
    if(!digitalRead(Lbumper) || !digitalRead(Lbumper)){//hits into the wall
        UC++;
    }
  }
}

void pos(int m, int n){ 
  X1 = m;
  Y1 = n;
  X2 = X1 - X;
  Y2 = Y1 - Y;
  if(Y2 == 6){
    UCmove(5);
    if(X1 < 0){
        pivot('L',90);
        UCmove(X2);
        pivot('R',90);
    }else if(X1 > 0){
        pivot('R',90);
        UCmove(X2);
        pivot('L',90);
        UCmove(1);
    }
  }else{
    UCmove(Y2);
    if (X1 < 0){
      pivot('L',90);
    }else if (X1 > 0){
      pivot('R',90);
      UCmove(X2);
    }
  }
}

void resetPosition(){ //After each time pick up and drop the object, we need to call this function
  turnAround();
  if(Y2==6){
    X = -X1;
    Y = -Y1;
    X1 = -OX;
    Y1 = -OY;
  }else{
    if (X1 < 0){
        X = -Y1;
        Y = X1;
        X1 = -OY;
        Y1 = OX;
    }else{
        X = Y1;
        Y = -X1;
        X1 = OY;
        Y1 = -OX;
    }
  }
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

int followLine(){
  while(1){
    checkWall(); 
    delay(400);
    while(analogRead(sensorR) > RTHRESH){     //veering left
      Serial.println("veering left loop");
      Serial.print("Left/Centre/Right:  ");
      Serial.print(analogRead(sensorL));
      Serial.print(", ");
      Serial.print(analogRead(sensorC));
      Serial.print(", ");
      Serial.println(analogRead(sensorR));
      
      digitalWrite(Ldirection, HIGH);
      digitalWrite(Rdirection, HIGH);
      analogWrite(Lspeed, left_speed);
      analogWrite(Rspeed, 0);
      inters = detectIntersection() + bluetoothEmergency();       //if Bluetooth emergency will be 1 if missed an intersection
      if(inters == 1){ 
        inters = 0;
        return 1;
        break;
      }
      checkWall();
    }

    while(analogRead(sensorL) > LTHRESH){     //veering right
      Serial.println("veering right loop");
      Serial.print("Left/Centre/Right:  ");
      Serial.print(analogRead(sensorL));
      Serial.print(", ");
      Serial.print(analogRead(sensorC));
      Serial.print(", ");
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
      checkWall();
    }
     int inters = detectIntersection() + bluetoothEmergency();
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

void doubleBlink(){               //used to denote that things happen during the competition
  for(int i=0;i<2;i++){
    digitalWrite(LED,HIGH);
    delay(50);
    digitalWrite(LED,LOW);
    delay(50);
  }
}

int bluetoothEmergency(){
  //for EPBMX, the serial.begin must be changed to "115200"
  int completion = 0;
  while(Serial.available() && (completion == 0)){         //completion goes to 1 when a task is completed
    int code = Serial.read();
    Serial.print("BT Number Recieved: ");
    Serial.println(code);
    
    switch(code){
      case 72:            //pressed H, make the robot go back to home base
        Serial.println("Going back to home base");
        //pos(0,0);
        Serial.println("Now at home base");
        completion++;
        break;
      case 66:            //pressed B, make the robot go to bin and deposit ball
        //pos(3,-1);     //the position of the bin
        depositBall();
        completion++;
        break;
      case 73:            //pressed I, mark that the robot passed an intersection
        return 1;
        completion++;
        break;
    }
  }
  completion = 0;
  if(!Serial.available()){
    Serial.println("Serial is unavailable");
  }
}
  
  
//----------------------------------------------SETUP
void setup() {
  Serial.begin(115200);
  
  IRserial.attach(9, -1);

  pinMode(Lbumper, INPUT);
  pinMode(Rbumper, INPUT);
  pinMode(Lspeed, OUTPUT);
  pinMode(Ldirection, OUTPUT);
  pinMode(Rspeed, OUTPUT);
  pinMode(Rdirection, OUTPUT);
  pinMode(wheelEncoder, INPUT);
  pinMode(LED, OUTPUT);     //this LED will turn on when an intersection has been detected, and when a sequence is completed
  pinMode(button, INPUT);

  //-----------------------
  //startButton();            //program will not continue until button is pressed
  //-----------------------
  
  tilt.attach(12);
  grip.attach(13);
  tilt.write(50);       //the correct height for the arm
  
  left_speed = 120;               //temporary for troubleshooting
  right_speed = 120;
 // left_speed = EEPROM.read(0); //this is how it should be
  //right_speed = EEPROM.read(1);
}


//----------------------------------------------LOOP
void loop() {

  getBall(1);
     
  
}
