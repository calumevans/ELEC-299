
void setup() {
Serial.begin(115200);     //EPBMX
}

void loop() {
  if(Serial.available()){     //is 1 if available, so it will go into the statement
    int code = Serial.read();
    Serial.println(code); 
    if(code == 89){
      Serial.println("Hello World");
    }
  }
}
