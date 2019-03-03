//--------------------------------------------BASIC FUNCTIONS
void forward(){
    digitalWrite(Ldirection, HIGH);
    digitalWrite(Rdirection, HIGH);
    analogWrite(Lspeed, left_speed);
    analogWrite(Rspeed, right_speed);
}


void stop(){
    digitalWrite(Ldirection, HIGH);
    digitalWrite(Rdirection, HIGH);
    analogWrite(Lspeed, 0);
    analogWrite(Rspeed, 0);
}

void backwards(){
    digitalWrite(Ldirection, LOW);
    digitalWrite(Rdirection, LOW);
    analogWrite(Lspeed, left_speed);
    analogWrite(Rspeed, right_speed);
}

void pivot(char direction, int degrees){             //pivot function
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


void followLine(){




}

void detectIntersection(){




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
