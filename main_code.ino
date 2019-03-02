#include <EEPROM.h>
#include <functions.h'>

//----------------------------------DIGITAL PINS
#define sensorIR            2
#define startButton         3
#define Lbumper             4
#define Rbumper             5
#define Lspeed              6
#define Ldirection          7
#define Rspeed              8
#define Rdirection          9
#define wheelEncoder        10

Servo pan, tilt, grip   //pins 11, 12, 13

//----------------------------------ANALOG PINS
#define forceBall           0
#define lineL               1
#define lineC               2
#define lineR               3
#define range               4

byte left_speed;                    //speed store on each robot for going straight
byte right_speed;



//----------------------------------SETUP
void setup(){
  Serial.begin(9600);
  pinMode(sensorIR, INPUT);
  pinMode(startButton,INPUT);
  pinMode(Lbumper,INPUT);
  pinMode(Rbumper,INPUT);
  pinMode(Lspeed,OUTPUT);
  pinMode(Ldirection,OUTPUT);
  pinMode(Rspeed, OUTPUT);
  pinMode(Rdirection,OUTPUT);
  pinMode(Lencoder,INPUT);
 
  pan.attach(11);
  tilt.attach(12);
  grip.attach(13);
  

  left_speed = EEPROM.read(0);
  right_speed = EEPROM.read(1);
 
}


//----------------------------------LOOP
void loop(){




}
