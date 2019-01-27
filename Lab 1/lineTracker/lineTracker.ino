//line tracker
#define LTHRESH 800
#define CTHRESH 800
#define RTHRESH 800

int speed1 = 6;
int direction1 = 7;
int speed2 = 5;
int direction2 = 4;

int sensorL = 0;
int sensorR = 1;
int sensorC = 2;

int valueL = 0;
int valueC = 0;
int valueR = 0;


void setup() {
  Serial.begin(9600);
  pinMode(direction1, OUTPUT);
  pinMode(direction2, OUTPUT);
  pinMode(speed1, OUTPUT);
  pinMode(speed2, OUTPUT);
}

void loop() {
  delay(2000);
  
  while(valueL <= LTHRESH){
    valueL = analogRead(sensorL);
    valueC = analogRead(sensorC);
    valueR = analogRead(sensorR);
  
    delay(400);                 //Serial printing the sensor data for a better understanding
    Serial.print(valueL);
    Serial.print(", ");
    Serial.print(valueC);
    Serial.print(", ");
    Serial.println(valueR);


    digitalWrite(direction1, HIGH);   //initial motor settings commanding the robot to move forward
    digitalWrite(direction2, HIGH);
    analogWrite(speed1, 79);
    analogWrite(speed2, 79);
  }
  
  analogWrite(speed1, 0);         //the while loop has broken by this point, so motor condition to stop
  analogWrite(speed2, 0);
}
