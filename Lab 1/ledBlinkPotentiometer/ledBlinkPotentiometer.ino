int button = 7; //initialize button
int led = 13; //initialize led
int pot = 2; //initialize potentiometer
int value = 0; //button value
int potval = 0; //potentiometer value

void setup(){
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(button, INPUT);
  pinMode(led, OUTPUT);
}

void loop(){

  value = digitalRead(button);
  if(value == 0){
    digitalWrite(led, LOW);
  }else{
    potval = analogRead(pot);    //get the value of potentiometer (0-1015)
    Serial.println(potval);      //print that value to the serial monitor
    digitalWrite(led, LOW);       
    delay(potval);               //delays for the milliseconds of the value of potentiometer
    digitalWrite(led, HIGH);
    delay(potval);
  }
}
