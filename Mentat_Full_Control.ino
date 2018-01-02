#include <NewPing.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

Servo sonarTurret;
const int delayTime = 2000;
const int sonarServoPin = 9;

#define TRIGGER_PIN 12
#define ECHO_PIN 11
#define MAX_DISTANCE 350
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
unsigned int distanceRight;
unsigned int distanceLeft;


const int irLeft = 2; //infrared pin variables
const int irRight = 3;
volatile int wallLeft = 0;     //infrared read variables
volatile int wallRight = 0;

void setup()
{
  Serial.begin(9600);
  Serial.println("sonar test");
  Serial.println(sonarRead());


  pinMode (irRight, INPUT);  //IR sensor setup
  pinMode (irLeft, INPUT);
  






  sonarTurret.attach(sonarServoPin);  //Ultrasonic Ranging Turret Servo Set up

  AFMS.begin();              //start your engines
  myMotor1->setSpeed(125);
  myMotor1->run(FORWARD);
  myMotor1->run(RELEASE);

  myMotor2->setSpeed(125);
  myMotor2->run(FORWARD);
  myMotor2->run(RELEASE);

  myMotor3->setSpeed(125);
  myMotor3->run(FORWARD);
  myMotor3->run(RELEASE);

  myMotor4->setSpeed(125);
  myMotor4->run(FORWARD);
  myMotor4->run(RELEASE);
}                              //end motor setup

void loop(){
  turretCenter();
  sonarReadForward();
  wallRight = digitalRead(irRight);
  wallLeft = digitalRead(irLeft);
  while (sonarReadForward() > 30 && wallRight == HIGH && wallLeft == HIGH){    //default forward movement
    allForward();
  }

  if (wallLeft == LOW){
    turnRight();
    delay(30);
  }

  if (wallRight == LOW){
    turnLeft();
    delay(30);
  }

  while (sonarReadForward() < 30){
    allStop();
    distanceRight = scanRight();
    if (distanceRight > 30){
      turnRight();
      break;
    }

    distanceLeft = scanLeft();
    if (distanceLeft > 30){
      turnLeft();
      break;
    }
    else{
      reverseLeft();
      break;
    }
  }
  if(wallRight == LOW && wallLeft == LOW){
    reverseRight;
  }
  
}




void allForward()                //start define functions
{
  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
  myMotor3->run(FORWARD);
  myMotor4->run(FORWARD);
}
void allStop()
{
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
  myMotor3->run(RELEASE);
  myMotor4->run(RELEASE);
}
void turnRight()
{
  myMotor1->run(RELEASE);
  myMotor2->run(FORWARD);
  myMotor3->run(FORWARD);
  myMotor4->run(RELEASE);
}
void turnLeft()
{
  myMotor1->run(FORWARD);
  myMotor2->run(RELEASE);
  myMotor3->run(RELEASE);
  myMotor4->run(FORWARD);
}
void reverseLeft(){
  myMotor1->run(BACKWARD);
  myMotor2->run(RELEASE);
  myMotor3->run(RELEASE);
  myMotor4->run(BACKWARD);
}
void reverseRight(){
  myMotor1->run(RELEASE);
  myMotor2->run(BACKWARD);
  myMotor3->run(BACKWARD);
  myMotor4->run(RELEASE);
}
int scanLeft(){
  unsigned int lookLeft;
  sonarTurret.write(180);
  delay(1000);
  lookLeft = sonarRead();
  return(lookLeft);
}
int scanRight(){
  unsigned int lookRight;
  sonarTurret.write(0);
  delay(1000);
  lookRight = sonarRead();
  return(lookRight);
}
void turretCenter(){
  sonarTurret.write(85);
  delay(delayTime);
}
int sonarRead(){
  delay(50);
  unsigned int distance = sonar.ping();
  return(distance / 74 / 2);
}
int sonarReadForward(){
  delay(35);
  unsigned int distance = sonar.ping();
  return(distance / 74 / 2);
}











