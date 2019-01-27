#include <Servo.h>

Servo pan, tilt, grip;
int led = 13;
int gripper = 11;

int force = 0;


void setup() {
  Serial.begin(9600);
  pan.attach(8);
  tilt.attach(9);
  grip.attach(10);
  
  pinMode(led, OUTPUT);
  pinMode(gripper, INPUT);
  grip.write(40);
}

void loop() {
  
  pan.write(90);
  tilt.write(70);
  

  
  for(int i = 40; i<=180; i++){
    
    grip.write(i);
    delay(50);
    force = digitalRead(gripper);
    Serial.println(force);
    if((force == 1) && (i>120)){
      digitalWrite(led, !force);
      grip.write(i);
      exit(0);
    }
    
    }
  }



