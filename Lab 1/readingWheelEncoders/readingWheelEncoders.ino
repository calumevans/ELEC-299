int wheel = 12;
int led = 13;

int wheelVal = 0;

void setup() {
  Serial.begin(9600);
  pinMode(wheel, INPUT);
  pinMode(led, OUTPUT);
}

void loop() {
  delay(500);
  wheelVal = digitalRead(wheel);
  digitalWrite(led, wheelVal);
  Serial.println(wheelVal);
}
