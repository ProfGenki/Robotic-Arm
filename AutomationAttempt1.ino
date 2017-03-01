/*===============================================================================================================*
*                                                                                                                *
*                                              Variables                                                         *
*                                                                                                                *
*================================================================================================================*/

//=======================================Libraries===========================================================\\

#include <math.h> 
#include <Servo.h>


//=========================================Angles=============================================================\\

float gyro;    
float alpha;
float gamma;

float currentAngle; //Current angle of gyroscope. 

//===========================================Buttons===========================================================\\

#define buttonBlue = 7;
#define buttonGreen = 6;
#define buttonRed = 5;

int prevButtonB;
int prevButtonG;
int prevButtonY;

//=========================================Control============================================================\\

int p;

const int velox = 1;
const int moveSpeed = 5;
const int automateSpeed = "placeholder";

const float sideA = "placeholder";
const float sideB = "placeholder";
const float heightG = "placeholder";

float xList[] = {1};
float yList[] = {1};

//======================================Servo Objects========================================================\\

Servo rotateServo;
Servo baseServo;
Servo armServo;
Servo gripperServo;

int initPosR = "placeholder";
int initPosB = "placeholder";
int initPosA = "placeholder";
int initPosG = "placeholder";
int initPosO = "placeholder";
int initPosS = "placeholder";

/*===============================================================================================================*
*                                                                                                                *
*                                                Set Up                                                          *
*                                                                                                                *
*================================================================================================================*/

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
  
 initialize();
}

/*===============================================================================================================*
*                                                                                                                *
*                                               Main Loop                                                        *
*                                                                                                                *
*================================================================================================================*/


void loop() {
 buttonCheck();
 servoMovement();
}

/*===============================================================================================================*
*                                                                                                                *
*                                               Functions                                                        *
*                                                                                                                *
*================================================================================================================*/
                 
//============================================Automation=========================================================\\                 

//=====Automate=====\\ 
void automate();
   sortServo.write(placeholderVal);
   delay(automateSpeed);
   sortServo.write(oppositePlaceholderVal);
   delay(automateSpeed);
}
     
//=====Flip=====\\ 
void flip(int o){
 if(f == 0){
   orientationServo.write(180);
   orientation = 1;}
 else if(f == 1){
   orientationServo.write(0);
   orientation = 0;}
 status();
}

//=====Sort=====\\                  
void sortPos(){
  flip();
  for(i = p; i < p + 10; i ++){
    gyro = getGyroVals;
    sideC = sqrt(xList[i]*xList[i] + (250 - yList[i]*yList[i]);
    theta = atan(xList[i]/(250 + yList[i]);
    alpha = asin(sideA*(sin(gyro))/sideB);
    gamma = asin(sideC*(sin(gyro))/sideB);
                 
    float alphaNeg = atan(heightG/sideC);
    alpha -= alphaNeg;
   
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
                 
//=========================================Control Functions=======================================================\\
                 
//=====Button Check=====\\                                   
void buttonCheck(){
 if(prevButtonB != digitalRead(buttonBlue)){
   sortPos();
   prevButtonB = digitalRead(buttonBlue);}
 if(prevButtonG != digitalRead(buttonGreen)){
   calibrate();
   prevButtonG = digitalRead(buttonGreen);}
 if(prevButtonR != digitalRead(buttonRed)){
   recall();
   prevButtonR = digitalRead(buttonRed);}
}
                 
//=====Recall=====\\ 
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
                 
//=====Status=====\\ 
void status(){
 if(digitalRead(statusLED) == LOW){
  statusLED.write(HIGH);}
 else{
  statusLED.write(LOW);}
} 
                 
//=====Initialize=====\\ 
void initialize(){
 baseServo.write(initPosB);
 delay(100);
 armServo.write(initPosA);
 delay(100);
 rotateServo.write(initPosR);
 delay(100);
 gripperServo.write(initPosG);
 delay(100);
 orientationServo.write(initPosO);
 delay(100);
 sortServo.write(InitPosS);
 delay(100);
 
 prevButtonB = digitalRead(buttonBlue);
 prevButtonG = digitalRead(buttonGreen);
 prevButtonR = digitalRead(buttonRed);
 
 currentAngle = initPosG;
 
 calibrate();
 
 stabilize();
} 
                 
//============================================Gyro Functions======================================================\\
                 
//=====Callibrate=====\\                                  
void callibrate(){
 status();
 //
 status();
} 
                
//=====getGyroVals=====\\                 
float getGyroVals(){

}
                 
//=====Stabilize=====\\ 
void stabilize(){
 gyro = getGyroVals();
 currentAngle -= gyro;
 gripperServo.write(currentAngle);
}
                             
//==========================================Servo Functions========================================================\\
                 
//=====Move=====\\                                              
void move(Servo servoObj, int v, int currentAngle){
 servoObj.write(currentAngle + v);
 stabilize();
 delay(movementSpeed);
}
                 
//=====ServoMovement=====\\                  
void servoMovement(){
 int rotateVal = analogRead(joyRX);
 int baseVal = analogRead(joyRY);
 int armVal = analogRead(joyLY);
 
 int currentAngleR = rotateServo.read();
 int currentAngleB = baseServo.read();
 int currentAngleA = armServo.read();
 
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
                 
