int gyroPin;
float gyroVoltage = 5;
float gyroZeroVoltage; //Look at datasheet
float gyroSensitivity = 0.007;
float rotationThreshold = 1;
float currentAngle = 0;

int p;

float sideA;
float sideB;

float xList[] = {1};
float yList[] = {1};

float gyro;    
float alpha;
float gamma;


void setup() {
 p = 0;
 

}

void loop() {
  // put your main code here, to run repeatedly:

}

/*void servoPos(){
  for(i = p; i = p + 9; i ++){
    sideC = sqrt(xList[i]**2 + (25 - yArray[i]**2);
    theta = atan(xList[i]/(25 + yArray[i]);
    alpha = asin(sideA*sin(gyro)/sideB);
    gamma = asin(sideC*sin(gyro)/sideB);

    rotateServo.write(theta + 90);
    delay(15);
    baseServo.write(alpha);
    delay(15);
    armServo.write(gamma);
    delay(15);

    automate();}
  p += 10;}
*/
/* float gyroAngle(){
  float gyroRate = analogRead(gyroPin)*gyroVoltage/1023;  //Converts quid to voltage
  gyroRate -= gyroZeroVoltage; //Voltage offset

  if(gyroRate >= rotationThreshold || gyroRate <= -rotationThreshold){
    gyroRate/= 100;
    currentAngle += gyroRate;}

//Keeps angles between 0-359 degrees
    if(currentAngle<0){
      currentAngle +=360;
      else if(currentAngle > 359){
        currentAngle -=360;
      }
  }
}
*/
