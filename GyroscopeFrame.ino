





/*========================================================================================*
*                                                                                         *
*                                   Variables                                             *
*                                                                                         *
*=========================================================================================*/

//======================Libraries==========================================\\
#include <Wire.h>  //for I2C communication
#include <Servo.h>   

//=========================================================================\\

#define CTRL_REG1 0x20   
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23

//========================Servos===========================================\\

Servo gripperY;   //Servo Objects
Servo gripperZ;

int initPosY = 90;   //Initial Positions for servos
int initPosZ = 90;   //Initial Positions for servos

float angleY;
float angleZ;
float angleX;

float currentAngleY = 90.0;
float currentAngleZ = 90.0;

//=======================Control==========================================\\
int measurementRange = 500;   //+ or - 500 Degrees per Second

float thresholdVal = 0.0175;    //Gyro Sensitivity in millidegrees per digit

float zeroRate = 15.0;      //Digital Zero.   + or - 15 

int sampleCycle = 100;   //Hertz (Cycles per Second)

int samplePeriod =  1000/sampleCycle;  //ms

float xIntegral = 0.0;
float yIntegral = 0.0;
float zIntegral = 0.0;
//======================================================================\\

int Addr = 105;                 // I2C address of gyro

//======================================================================\\

int x, y, z;  //Creates variables for x,y,z axes


/*======================================================================================*
 *                                                                                      * 
 *                        Setup                                                         *
 *                                                                                      *
 *======================================================================================*/

 
void setup(){
  Wire.begin();
  Serial.begin(9600);
  
  gripperY.attach(8);
  gripperZ.attach(9);
  
  writeI2C(CTRL_REG1, 0x1F);    // Turn on all axes, disable power down
  writeI2C(CTRL_REG3, 0x08);    // Enable control ready signal
  writeI2C(CTRL_REG4, 0x80);    // Set scale (500 deg/sec)
  
  delay(100);                   // Wait to synchronize 
  
  gripperY.write(initPosY);
  gripperZ.write(initPosZ);
}


/*========================================================================================*
*                                                                                         *
*                                  Loop                                                   *
*                                                                                         *
*=========================================================================================*/


void loop(){
  getGyroValues();           // Get new values
  // In following Dividing by 114 reduces noise 
  Serial.print(" Raw Y:"); Serial.print(y / 114);
  Serial.print(" Raw Z:"); Serial.println(z / 114);
  delay(100);                   // Short delay between reads
  currentAngleY = gripperY.read();
  currentAngleZ = gripperZ.read();
  gripperY.write(currentAngleY - y);
  gripperZ.write(currentAngleZ - z);
}


/*========================================================================================*
*                                                                                         *
*                                  Functions                                              *
*                                                                                         *
*=========================================================================================*/

void getGyroValues() {
  
    byte MSB, LSB;

    MSB = readI2C(0x29);
    LSB = readI2C(0x28);
    x = ((MSB << 8) | LSB);  //Not Needed
    if(abs(x) >= thresholdVal){xIntegral += x;}
    x /= thresholdVal;
    
    MSB = readI2C(0x2B);
    LSB = readI2C(0x2A);
    y = ((MSB << 8) | LSB);  //Needed when gripper is in standard pos.
    if(abs(y) >= thresholdVal){yIntegral += x;}
    y /= thresholdVal;
    
    MSB = readI2C(0x2D);
    LSB = readI2C(0x2C);
    z = ((MSB << 8) | LSB);   //Needed when gripper is rotated
    if(abs(z) >= thresholdVal){zIntegral += z;}
    z /= thresholdVal;
}

//==============================================================================\\

int readI2C (byte regAddr) {
    Wire.beginTransmission(Addr);
    Wire.write(regAddr);                // Register address to read
    Wire.endTransmission();             // Terminate request
    Wire.requestFrom(Addr, 1);          // Read a byte
    while(!Wire.available()) { };       // Wait for receipt
    return(Wire.read());                // Get result
}

//================================================================================\\

void writeI2C (byte regAddr, byte val) {
    Wire.beginTransmission(Addr);
    Wire.write(regAddr);
    Wire.write(val);
    Wire.endTransmission();
}

//==================================================================================\\

void gyroIntegral(){   //Integral = Sigma(gyroscope values)*sampling period from 0 tot "t".
  getGyroValues();
  angleX = xIntegral/(samplePeriod*114);
  angleY = yIntegral/(samplePeriod*114);
  angleZ = zIntegral/(samplePeriod*114);
  Serial.print("Angle Y: \n");
  Serial.print(angleY);
  Serial.print("Angle Z: \n");
  Serial.print(angleZ);
}

//==================================================================================\\

void gyroMovements(){
  gyroIntegral();
  
  currentAngleY -= angleY;
  currentAngleZ -= angleZ;

  while(abs(currentAngleY) > 360){
    if(currentAngleY > 360){
      currentAngleY -= 360; 
    }
    else{
      currentAngleY += 360;
    }
  }

  while(abs(currentAngleZ) > 360){
    if(currentAngleZ > 360){
      currentAngleZ -= 360; 
    }
    else{
      currentAngleZ += 360;
    }
  }
  
  gripperY.write(currentAngleY);
  gripperZ.write(currentAngleZ);
}

