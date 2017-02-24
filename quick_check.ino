#include <Servo.h>

Servo rotateServo;
Servo baseServo;
Servo armServo;
Servo gripperServo;

Servo gripperServo2;

float servoPos;

float velox = 1;

int delaySpeed = 5;

int joyRotate = 0;
int rotateVal;
int rotatePos;

int joyBase = 1;
int baseVal;
int basePos;

int joyArm = 2;
int armVal;
int armPos;

int joyGripper = 3;
int gripperVal;
int gripperPos;

int buttonBlue = 7;
int buttonGreen = 6;
int buttonRed = 5
int buttonJoyR = 4;
int buttonJoyL = 3;

int buttonStateB;
int buttonStateG;
int buttonStateR;
int buttonStateJR;
int buttonStateJL;

int statusLED = 2;
 
void setup() {
 pinMode(buttonBlue, INPUT);
 pinMode(buttonGreen, INPUT);
 pinMode(buttonRed, INPUT);
 pinMode(buttonJoyR, INPUT);
 pinMode(buttonJoyL, INPUT);
 
 pinMode(statusLED, OUTPUT);
  
 initialize();
}


void loop() {
 movement(velox, delaySpeed);
 buttonCheck();
}


void movement(velox, delaySpeed){
 rotateVal = analogRead(joyRotate);
  if(rotateVal > 800){
    servoPos = rotateServo.read() + velox;
    rotateServo.write(servoPos);}
  if(rotateVal < 200){
    servoPos = rotateServo.read() - velox;
    rotateServo.write(servoPos);}
delay(delaySpeed);

baseVal = analogRead(joyBase);
  if(baseVal > 800){
    servoPos = baseServo.read() - velox;
    baseServo.write(servoPos);}
  if(baseVal < 200){
    servoPos = baseServo.read() + velox;
    baseServo.write(servoPos);}
delay(delaySpeed);

armVal = analogRead(joyArm);
  if(armVal > 700){
    servoPos = armServo.read() + velox;
    armServo.write(servoPos);}
  if(armVal < 300){
    servoPos = armServo.read() - velox;
    armServo.write(servoPos);}
delay(delaySpeed);
 
 gripperVal = analogRead(joyGripper);
  if(gripperVal > 700){
    servoPos = gripperServo.read() + (velox/gripperFactor);
    gripperServo.write(servoPos);}
  if(gripperVal < 300){
    servoPos = gripperServo.read() - (velox/gripperFactor);
    gripperServo.write(servoPos);}
 
}

void buttonCheck(){
 if(buttonStateB != digitalRead(buttonBlue)){
    toggleServo():
 if(buttonStateG != digitalRead(buttonGreen)){
   toggleGyro();
 if(buttonStateR != digitalRead(buttonRed)){
   recall();
 }


void initialize(){
  rotateServo.attach(8);
  delay(15);
  rotateServo.write(90);
  delay(15);
  
  baseServo.attach(9);
  delay(15);
  baseServo.write(180);
  delay(15);

  armServo.attach(10);
  delay(15);
  armServo.write(90);
  delay(15);
    
  gripperServo.attach(11);
  delay(15);
  gripperServo.write(90);
  delay(15);
 
  initRotate = rotateServo.read();
  initBase = baseServo.read();
  initArm = armServo.read();
  initGripper = gripperServo.read();
  
  buttonStateB = digitalRead(buttonBlue);
  buttonStateG = digitalRead(buttonGreen);
  buttonStateR = digitalRead(buttonRed);
  buttonStateJR = digitalRead(buttonJoyR);
  buttonStateJL = digitalRead(buttonJoyL);
}
  
  void toggleServo(){
   if(servoState == 0){
    gripperServo2.write(180);
   }
   else{
    gripperServo2.write(90);
   }
   buttonStateB = digitalRead(buttonBlue);
  }
  
  void toggleGyro(){
   if(gyroState = 0){
    gyroState = 1;
   }
   else{
    gyroState = 0;
   }
   buttonStateG = digitalRead(buttonGreen);
  }
   
void recall(){
 gripperServo.write(initPosGripper);
 baseServo.write(initPosBase);
 armServo.write(initPosArm);
 rotateServo.write(initPosRotate);
 buttonStateR = digitalRead(buttonRed);
}
