#include <Servo.h>
Servo testservo;
Servo testservo2;
Servo testservo3;

int joyTest3 = 3;
int joyVal3;
int servoPos3;

int joyTest2 = 1;
int joyVal2;
int servoPos2;

int joyTest =2;
int joyVal;
int servoPos;

void setup() {
 // pinMode(7,INPUT);
  //inMode(6, INPUT);
  testservo.attach(10);
  delay(15);
  testservo.write(90);
  delay(15);
  
  testservo2.attach(9);
  delay(15);
  testservo2.write(180);
  delay(15);

  testservo3.attach(8);
  delay(15);
  testservo3.write(90);
  delay(15);
    
Serial.begin(9600);
}

void loop() {
 /* Serial.print("Joystick 0: ");
  Serial.print(analogRead(0));
  Serial.print(".  ");
  delay(2000);

  Serial.print("Joystick 1: ");
  Serial.print(analogRead(1));
  Serial.print(".  ");
  delay(2000);

  Serial.print("Joystick 2: ");
  Serial.print(analogRead(2));
  Serial.print(".  ");
  delay(750);

  Serial.print("Joystick 3: ");
  Serial.print(analogRead(3));
  Serial.print(".  ");
  delay(750);
  */

joyVal = analogRead(joyTest);
  if(joyVal > 800){
    servoPos = testservo.read() + 1;
    testservo.write(servoPos);}
  if(joyVal < 200){
    servoPos = testservo.read() - 1;
    testservo.write(servoPos);}
delay(5);

joyVal2 = analogRead(joyTest2);
  if(joyVal2 > 800){
    servoPos2 = testservo2.read() - 1;
    testservo2.write(servoPos2);}
  if(joyVal2 < 200){
    servoPos2 = testservo2.read() + 1;
    testservo2.write(servoPos2);}
delay(5);

joyVal3 = analogRead(joyTest3);
  if(joyVal3 > 700){
    servoPos3 = testservo3.read() + 1;
    testservo3.write(servoPos3);}
  if(joyVal3 < 300){
    servoPos3 = testservo3.read() - 1;
    testservo3.write(servoPos3);}
delay(5);

}
