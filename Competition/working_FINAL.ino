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
int Xstart;
int Ystart;
//int UC = 0;
int OX,OY;
int Xgoal,Ygoal,Xpath,Ypath;

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

//*      OLD PIVOT FUNCTION
void pivot(char direction, int degrees){            //pivot function
  int counter = 0;                                  //this function is VERY dependant on battery charge level
  switch (degrees){
    case 90:                     //left rotation
      if (direction == 'L'){
        while (counter < 20){
          Serial.println("Pivot left");
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
        Serial.println("Pivot right");
        while (counter < 25000) {
          digitalWrite(Ldirection, HIGH);
          digitalWrite(Rdirection, LOW);
          analogWrite(Lspeed, left_speed);
          analogWrite(Rspeed, right_speed);
          int LwheelVal = digitalRead(wheelEncoder);
          counter = counter + LwheelVal;
          //Serial.print("Counter: ");
          //Serial.println(counter);
        }
        counter = 0;
      } else {
        Serial.println("Invalid direction");
      }
      break;
    case 180:
      while (counter < 39){
        Serial.println("Pivot 180");
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



//--------------------------------------NEW PIVOT FUNCTIONS
/*void pivotL(){
  int counter = 0;
  while (counter < 9){
    Serial.println("Pivot right");
    digitalWrite(Ldirection, HIGH);
    digitalWrite(Rdirection, LOW);
    analogWrite(Lspeed, left_speed);
    analogWrite(Rspeed, right_speed);
    int RwheelVal = digitalRead(wheelEncoder);
    counter = counter + RwheelVal;
    Serial.print("Counter: ");
    Serial.println(counter);
  }
    
    while (analogRead(sensorL) <= LTHRESH){
      Serial.println("Pivot left");
      digitalWrite(Ldirection, LOW);
      digitalWrite(Rdirection, HIGH);
      analogWrite(Lspeed, left_speed);
      analogWrite(Rspeed, right_speed);
  
}
}
void pivotR(){
  int counter = 0;
  while (counter < 9){
    Serial.println("Pivot right");
    digitalWrite(Ldirection, HIGH);
    digitalWrite(Rdirection, LOW);
    analogWrite(Lspeed, left_speed);
    analogWrite(Rspeed, right_speed);
    int RwheelVal = digitalRead(wheelEncoder);
    counter = counter + RwheelVal;
    Serial.print("Counter: ");
    Serial.println(counter);
  }
  while (analogRead(sensorR) <= RTHRESH){
    Serial.println("Pivot right");
    digitalWrite(Ldirection, HIGH);
    digitalWrite(Rdirection, LOW);
    analogWrite(Lspeed, left_speed);
    analogWrite(Rspeed, right_speed);
  }
}

void pivotB(){
  int counter = 0;
  while (counter < 30){
    Serial.println("Pivot left");
    digitalWrite(Ldirection, LOW);
    digitalWrite(Rdirection, HIGH);
    analogWrite(Lspeed, left_speed);
    analogWrite(Rspeed, right_speed);
    int LwheelVal = digitalRead(wheelEncoder);
    counter = counter + LwheelVal;
    Serial.print("Counter: ");
    Serial.println(counter);
  }
  while (analogRead(sensorL) <= LTHRESH){
    Serial.println("Pivot left");
    digitalWrite(Ldirection, LOW);
    digitalWrite(Rdirection, HIGH);
    analogWrite(Lspeed, left_speed);
    analogWrite(Rspeed, right_speed);
  }        
}*/


//------------------------------------------------------COMPLEX FUNCTIONS
//----------------------------------------------SEQUENCES
int identifyStartingPosition(){         //left = 1, centre = 2, right = 3
    Serial.print("Identifying starting position...");
    while(startingPosition == 0){                     //constantly checking for character
      switch(IRserial.receive(200)){
          case 1:
            Serial.println("Left");
            startingPosition = 1;
            break;
          case 2:
            Serial.println("Center");
            startingPosition = 2;
            break;
          case 3:
            Serial.println("Right");
            startingPosition = 3;
            break;
      }
      delay(80);
    }
  doubleBlink();
  return startingPosition;
}

void sequence(){                      //sequence based on starting position
  int location = identifyStartingPosition();
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
  for(int i=90;i<180;i++){
    //Serial.println(i);
    grip.write(i);
    delay(50);
  }
  tilt.write(170);
  delay(1500);
  resetPosition();
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
  resetPosition();
  /*for(int i=180;i>89;i--){
    //Serial.println(i);
    grip.write(i);
    delay(50);
  }
  delay(3000);
  resetPosition();
  */
}


void checkWall(){     //depending on the global variable 'grabOrDeposit', a ball will be grabbed or deposited
  if(!digitalRead(Lbumper) || !digitalRead(Rbumper)){
    stopped();
    if(grabOrDeposit == 'G'){          //for grabbing the ball
      backwards();
      delay(400);
      grabBall();
    }else if(grabOrDeposit == 'D'){    //for depositing the ball
      depositBall();
    }
    if(grabOrDeposit == 'G'){          //for grabbing the ball
      grabOrDeposit = 'D';
    }else{    //for depositing the ball
      grabOrDeposit = 'G';
    }
  }
}


void getBall(int ballNum){        //only ball #1 has coords mapped out so far
  Serial.print("Ball #");
  Serial.print(ballNum);
  Serial.println(" is being captured");  
  grabOrDeposit = 'G';
  switch(ballNum){
    case 1:
      pos(-3,-2);
      checkWall();
      grabOrDeposit = 'D';
      pos(Xgoal,Ygoal);   //go to bin
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
      grabOrDeposit = 'G';
      straight();
      turnAround();
      straight();
      turnAround();
      break;
    case 8:
      grabOrDeposit = 'G';
      straight();
      turnAround();
      straight();
      turnAround();
      break;
    case 9:
      grabOrDeposit = 'G';
      straight();
      turnAround();
      straight();
      turnAround();
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

//----------------------------------------------POSITION
void startPosition(){         //defining initial coordinates
  if (startingPosition == 1){
    Serial.println("starting position set to 1");
    Xstart = -1;
    Ystart = -3;
  }else if(startingPosition == 2){
    Xstart = 0;
    Ystart = -3;
  }else if(startingPosition == 3){
    Xstart = 1;
    Ystart = -3;
  }
  OX = Xstart;
  OY = Ystart;
}

void UCmove(int L){  //for moving 'L' units
  int UC =0;
  while(UC < abs(L)){
    Serial.println("UC is ");
    Serial.println(UC);
    UC = UC + followLine();
    if(!digitalRead(Lbumper) || !digitalRead(Lbumper)){//hits into the wall
        UC++;
    }
  }
}

void pos(int m, int n){     //the main position function
  Xgoal = m;
  Ygoal = n;
  Xpath = Xgoal - Xstart;
  Ypath = Ygoal - Ystart;
  if(Ypath == 6){        //if the top row
    Serial.println("y == 6");
    UCmove(5);
    if(Xgoal < 0){
        pivot('L',90);
        UCmove(Xpath);
        pivot('R',90);
    }else if(Xgoal > 0){
        pivot('R',90);
        UCmove(Xpath);
        pivot('L',90);
        UCmove(1);
    }
  }else{              //anything else
    UCmove(Ypath);
    Serial.println("y != 6");
    if (Xgoal < 0){
      pivot('L',90);
    }else if (Xgoal > 0){
      pivot('R',90);
    }
    UCmove(Xpath);
    Serial.print("Xpath: ");
    Serial.println(Xpath);
  }
}

void resetPosition(){     //after completing a task, coordinates are reset
  turnAround();
  if(Ypath==6){
    Xstart = -Xgoal;
    Ystart = -Ygoal;
    Xgoal = -OX;
    Ygoal = -OY;
  }else{
    if (Xgoal < 0){
        Xstart = -Ygoal;
        Ystart = Xgoal;
        Xgoal = -OY;
        Ygoal = OX;
    }else{
        Xstart = Ygoal;
        Ystart = -Xgoal;
        Xgoal = OY;
        Ygoal = -OX;
    }
  }
}

//----------------------------------------------NAVIGATION
int detectIntersection(){
  if(((analogRead(sensorL) > LTHRESH) && (analogRead(sensorC) > CTHRESH) && (analogRead(sensorR) > RTHRESH))) {
      Serial.println("Intersection detected!");
      digitalWrite(LED,HIGH);
      delay(200);               //this delay will change with contrast
      stopped();
      digitalWrite(LED,LOW);
      delay(2000);
      forward();
      delay(500);
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
      inters = detectIntersection();       //if Bluetooth emergency will be 1 if missed an intersection
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
     int inters = detectIntersection();
     if(inters == 1){
        inters = 0;
        return 1;
        break;
     }
     forward();
  }  
}

int straight(){
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
      
      checkWall();
    }
    while((analogRead(sensorL) > LTHRESH) && (analogRead(sensorR) > RTHRESH)){
      forward();
    }
     
     forward();
  }  
}

void turnAround(){   //used for after the robot hits a wall (grabbing or depositing a ball)
  backwards();
  delay(1000);
  pivot('L',180);
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
  /*if(!Serial.available()){
    Serial.println("Serial is unavailable -bluetooth");
  }*/
}
  
  
//----------------------------------------------SETUP
void setup() {
  Serial.begin(9600);       //115200 for bluetoothM
  
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
  tilt.write(170);       //the correct height for the arm
  
  left_speed = 104;               //temporary for troubleshooting
  right_speed = 105;
  //left_speed = EEPROM.read(0); //this is how it should be
  //right_speed = EEPROM.read(1);
}


//----------------------------------------------LOOP
void loop(){
  /*
  //This is the code for the competition
  sequence();
  while(1) Serial.println("done");
  */
  Xstart = -3;
  Ystart = -2;
  pos(2,1);
  stopped;
  delay(30000);
  //pivot('R', 90);
  //stopped();
  //delay(3000);

  //Serial.println(digitalRead(2));
  //Serial.println(digitalRead(3));
  //startingPosition = 1;
  //startPosition();
  //getBall(1);
 
 
  //for testing:
  //startingPosition = 1;       //in the competition this is done with the IR reciever
  //getBall(1);
  
  
}
