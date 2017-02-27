#include <math.h> 
#include <Servo.h>

Servo rotateServo;
Servo baseServo;
Servo armServo;
Servo gripperServo;

int p;

float sideA;
float sideB;

float xList[] = {1};
float yList[] = {1};

float gyro;    
float alpha;
float gamma;

int buttonBlue = 7;
int buttonGreen = 6;
int buttonRed = 5;

int prevButtonB;
int prevButtonG;
int prevButtonY;

int initPosR;
int initPosB;
int initPosA;
int initPosG;
int initPosO;
int initPosS;


void setup() {
 p = 0;
 
 PINMODE(buttonBlue, INPUT);
 PINMODE(buttonGreen, INPUT);
 PINMODE(buttonRed, INPUT);
 PINMODE(statusLED, OUTPUT);
 
 rotateServo.attach(8);
 baseServo.attach(9);
 armServo.attach(10);
 gripperServo.attach(11);
 orientationServo.attach(12);
 sortServo.attach(13);
 
 prevButtonB = digitalRead(buttonBlue);
 prevButtonG = digitalRead(buttonGreen);
 prevButtonR = digitalRead(buttonRed);
 
 initPosR = rotateServo.read();
 initPosB = baseServo.read();
 initPosA = armServo.read();
 initPosG = gripperServo.read();
 
 initialize();
}

void loop() {
  // put your main code here, to run repeatedly:
 buttonCheck();
 servoMovement();
}

void sort(){
  flip();
  for(i = p; i < p + 10; i ++){
    gyro = getGyroVals;
    sideC = sqrt(xList[i]*xList[i] + (250 - yList[i]*yList[i]);
    theta = atan(xList[i]/(250 + yList[i]);
    alpha = asin(sideA*(sin(gyro))/sideB);
    gamma = asin(sideC*(sin(gyro))/sideB);
   
    stabilize();       
    delay(15);             
    rotateServo.write(theta);
    delay(15);
    baseServo.write(alpha);
    delay(15);
    armServo.write(gamma);
    delay(15);
                 
    stabilize();
                 
    automate();}
  p += 10;
  flip();
}

void buttonCheck(){
 if(prevButtonB != digitalRead(buttonBlue)){
   sort();
   prevButtonB = digitalRead(buttonBlue);}
 if(prevButtonG != digitalRead(buttonGreen)){
   calibrate();
   prevButtonG = digitalRead(buttonGreen);}
 if(prevButtonR != digitalRead(buttonRed)){
   recall();
   prevButtonR = digitalRead(buttonRed);}
}
               
void flip(int o){
 if(f == 0){
   orientationServo.write(180);
   orientation = 1;}
 else if(f == 1){
   orientationServo.write(0);
   orientation = 0;}
 status();
}
                 
void automate();
   sortServo.write(placeholderVal);
   delay(placeholderTime);
   sortServo.write(oppositePlaceholderVal);
}

void servoMovement(){
 int rotateVal = analogRead(joyRX);
 int baseVal = analogRead(joyRY);
 int armVal = analogRead(joyLY);
 
 if(rotateVal > 800){
   move(rotateServo, velox);}
 if(rotateVal < 200){
   move(rotateServo, -velox);}
 if(baseVal > 800){
   move(baseServo, velox);}
 if(baseVal < 200){
   move(baseServo, -velox);}
 if(armVal > 800){
   move(armServo, velox);}
 if(armVal < 200){
   move(armServo, -velox);}
}
                 
void move(Servo servoObj, int v){
 servoObj.write(v);
 stabilize();
 delay(speed);
}

void recall(){
 status();
 baseServo.write(initPos);
 delay(100);
 armServo.write(initPos);
 delay(100);
 rotateServo.write(initPos);
 delay(100);
 gripperServo.write(initPos);
 delay(100);
 status();
}
                 
void callibrate(){
 status();
 //Placeholder for code
 status();
}
 
void status(){
 if(digitalRead(statusLED) == LOW){
  statusLED.write(HIGH);}
 else{
  statusLED.write(LOW);}
} 
 
 
 
 
