//--------------------------------------------BASIC FUNCTIONS
void forward(){
    digitalWrite(Ldirection, HIGH);
    digitalWrite(Rdirection, HIGH);
    analogWrite(Lspeed, left_speed);
    analogWrite(Rspeed, right_speed);
}


void stop(){
    analogWrite(Lspeed, 0);
    analogWrite(Rspeed, 0);
}

void backwards(){
    digitalWrite(Ldirection, LOW);
    digitalWrite(Rdirection, LOW);
    analogWrite(Lspeed, left_speed);
    analogWrite(Rspeed, right_speed);
}

void CheckDistance(){
  delay(500);
  if(analogRead(IRrange) < THRESHHOLD){
    Forward();
  }else if(analogRead(IRrange) >= THRESHHOLD){
    Stop();
    delay(300);
    Forward();
  } 
}

void pivot(char direction, int degrees){             //pivot function
  int counter = 0;
  switch(degrees){
      case 45:                     //left rotation
        if(direction == 'L'){
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
            
      }else if(direction == 'R'){        //right rotation
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
      }else{
          Serial.println("Invalid direction"); 
      }
      break;
    case 180:
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
      break;
  }
}


//----------------------------------COMPLEX FUNCTIONS

int identifyStartingPosition(){
    int position = 0;
    Serial.print("Position: ");

    while(position == 0){
      int val = IRserial.receive(200);
      
      switch(char(val)){
          case 'L':
            Serial.println("Left");
            position = 1;
            break;
          case 'C':
            Serial.println("Center");
            position = 2;
            break;
          case 'R':
            Serial.println("Right");
            position = 3;
            break;
      }
      delay(100);
    }
    return position;
}


void turnAround(){              //just randomly thought that this would be a good idea
    
       
}

void followLine(){
int val1 = analogRead(A0);
 int val2 = analogRead(A1);
 int val3 = analogRead(A2);

 Serial.println("Value 1:");
 Serial.println(val1);
 Serial.println("Value 2:");
 Serial.println(val2);
 Serial.println("Value 3:");
 Serial.println(val3);
 

if (val3>LTHRESH){
   analogWrite(spdR, 100);
   analogWrite(spdL, 0);
  }
  else if(val2>RTHRESH){
    analogWrite(spdL, 100);
   analogWrite(spdR, 0);
  }
  else if(val1>CTHRESH){
   analogWrite(spdL, 100);
   analogWrite(spdR, 100);
  }


}



}

int detectIntersection(){                   //I think this will work? It return a single 1 when all three sensors sense black
    while(1){
        Serial.print(lineL);           //just printing the sensor values for seeing what threshholds we need
        Serial.print(", ");
        Serial.print(lineC);
        Serial.print(", ");
        Serial.println(lineR);
        if((analogRead(lineL) > THRESH) && (analogRead(lineC) > THRESH) && (analogRead(lineR) > THRESH)){
            Serial.println("Intersection detected!");
            break;
        }   
    }
    return 1;
}

void pos(int x, int y){
    
    
    
    
}


void getBall(int ballNum){
  Serial.print("Ball #");
  Serial.print(ballNum);
  Serial.println(" is being captured");
  
  switch(ballNum){            //there are 15 balls in this competition, this is getting the coordinates for the balls and using pos() to travel
    case 1:
      pos(0,1);
      break;
    case 2:
        pos(0,2);
      break;
    case 3:
        pos(0,3);
      break;
    case 4:
        pos(0,4);
      break;
    case 5:
        pos(0,5);
      break;
    case 6:
        pos(1,6);
      break;
    case 7:
        pos(2,6);
      break;
    case 8:
        pos(3,6);
      break;
    case 9:
        pos(4,6);
      break;
    case 10:
        pos(5,6);
      break;
    case 11:
        pos(6,5);
      break;
    case 12:
        pos(6,4);
      break;
    case 13:
        pos(6,3);
      break;
    case 14:
        pos(6,2);
      break;
    case 15:
        pos(6,1);
      break;
    default:
      Serial.println("Invalid ball number");
      break; 
  }
ballData[ballNum-1] = 1;          //updates the array to show each ball that is captured (denoted with a 1)
}


void grabBall(){

#include <Servo.h>
Servo myservo1, myservo2, myservo3;

int gripperPad = A5;

int grip = 0;
int angle = 0;

void setup() {
  // put your setup code here, to run once:
  
   myservo1.attach(11);
   myservo2.attach(12);
   myservo3.attach(13);
  
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  angle = 0 ;
  myservo1.write(angle);
  grip = analogRead (gripperPad);

  while(grip < 500){
    grip = analogRead (gripperPad);
    angle++;
    myservo1.write(angle);
    Serial.println(String(grip));
    delay(100);
    
  }  
    
  Serial.println(String(grip));

  }


}

void depositBall(){



}

void collisionDetection(){



}

void bluetoothEmergency(){
  //for EPBMX, the serial.begin must be changed to "115200"

  while(Serial.available() && (completion == 0)){         //completion goes to 1 when a task is completed
    int code = Serial.read();
    Serial.print("BT Number Recieved: ");
    Serial.println(code);
    
    switch(code){
      case ##:            //make the robot go back to home base
        Serial.println("Going back to home base");
        pos(0,0);
        Serial.println("Now at home base");
        completion++;
        break;
      case ##:            //go to deposit ball
        pos(3,-1);     //the position of the bin
        depositBall();
        completion++;
        break;
    }
  }
  completion = 0;
  if(!Serial.available()){
    Serial.println("Serial is unavailable");
  }
}
