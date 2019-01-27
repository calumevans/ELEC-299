int speed1 = 6;
int direction1 = 7;
int speed2 = 5;
int direction2 = 4;

int wheel = 12;
int led = 13;
int wheelVal = 0;
int counter = 0;

void setup() {
  Serial.begin(9600);
  pinMode(direction1, OUTPUT);
  pinMode(direction2, OUTPUT);
  pinMode(speed1, OUTPUT);
  pinMode(speed2, OUTPUT);
  
  pinMode(wheel, INPUT);
  pinMode(led, OUTPUT);
}

void loop() {     //forward

  while(counter < 100){
    digitalWrite(direction1, HIGH);
    digitalWrite(direction2, HIGH);
    analogWrite(speed1, 128);
    analogWrite(speed2, 128);
   
    
    wheelVal = digitalRead(wheel);
    counter = counter + wheelVal;
   
    Serial.print("Counter: ");
    Serial.println(counter);
    Serial.println("loop 1");
  }
  counter = 0;

  while(counter < 39){            //backwards
        Serial.println("loop 2");

    digitalWrite(direction1, LOW);
    digitalWrite(direction2, LOW);
    analogWrite(speed1, 128);
    analogWrite(speed2, 128);

    wheelVal = digitalRead(wheel);
    counter = counter + !wheelVal;
        Serial.print("Counter: ");
    Serial.println(counter);
  }

  counter = 0;

  while(counter < 20){            //turn
        Serial.println("loop 3");

    digitalWrite(direction1, HIGH);
    digitalWrite(direction2, LOW);
    analogWrite(speed1, 128);
    analogWrite(speed2, 128);


    wheelVal = digitalRead(wheel);
    counter = counter + wheelVal;
        Serial.print("Counter: ");
    Serial.println(counter);
  }

   counter = 0;

  while(counter == 0){              //stop
        Serial.println("loop 4");

    digitalWrite(direction1, HIGH);
    digitalWrite(direction2, LOW);
    analogWrite(speed1, 0);
    analogWrite(speed2, 0);


  }
  
 
}

   
