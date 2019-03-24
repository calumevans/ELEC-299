//this code is for checking the values of the line sensors
//----------------------------------ANALOG PINS
#define sensorL             0
#define sensorR             1
#define sensorC             2

void setup(){
  Serial.begin(9600);
}
void loop(){
  Serial.print("Left/Centre/Right:  ");
  Serial.print(analogRead(sensorL));
  Serial.print(", ");
  Serial.print(analogRead(sensorC));
  Serial.print(", ");
  Serial.println(analogRead(sensorR));
}
