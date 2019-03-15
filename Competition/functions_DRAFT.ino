

void bluetoothEmergency(){
  //for EPBMX, the serial.begin must be changed to "115200"

  while(Serial.available() && (completion == 0)){         //completion goes to 1 when a task is completed
    int code = Serial.read();
    Serial.print("BT Number Recieved: ");
    Serial.println(code);
    
    switch(code){
      case 1:            //make the robot go back to home base
        Serial.println("Going back to home base");
        pos(0,0);
        Serial.println("Now at home base");
        completion++;
        break;
      case 50:            //make the robot go back to home base
        pos(3,-1);     //the position of the bin
        depositBall();
        completion++;
        break;
    }
      case 51:            //make the robot go back to home base
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
