//--------------------------------------------SUPER BASIC FUNCTIONS
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

void backup(){
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


//----------------------------------HARD FUNCTIONS

int identifyStartingPosition(){



}

void followLine(){




}

void detectIntersection(){




}

void determineBall(){


}

void grabBall(){



}

void depositBall(){



}

void collisionDetection(){



}

void bluetoothReset(){




}
