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

/* Jarzebski Stuff
 // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  gyroscope.calibrate();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  gyroscope.setThreshold(3);
  
  calibrate(uint8_t samples)
{
    // Set calibrate
    useCalibrate = true;

    // Reset values
    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;
    float sigmaX = 0;
    float sigmaY = 0;
    float sigmaZ = 0;

    // Read n-samples
    for (uint8_t i = 0; i < samples; ++i)
    {
	readRaw();
	sumX += r.XAxis;
	sumY += r.YAxis;
	sumZ += r.ZAxis;

	sigmaX += r.XAxis * r.XAxis;
	sigmaY += r.YAxis * r.YAxis;
	sigmaZ += r.ZAxis * r.ZAxis;
	
	delay(5);
    }

    // Calculate delta vectors
    d.XAxis = sumX / samples;
    d.YAxis = sumY / samples;
    d.ZAxis = sumZ / samples;

    // Calculate threshold vectors
    thresholdX = sqrt((sigmaX / samples) - (d.XAxis * d.XAxis));
    thresholdY = sqrt((sigmaY / samples) - (d.YAxis * d.YAxis));
    thresholdZ = sqrt((sigmaZ / samples) - (d.ZAxis * d.ZAxis));

    // If already set threshold, recalculate threshold vectors
    if (actualThreshold > 0)
    {
	setThreshold(actualThreshold);
    }
}

setThreshold(uint8_t multiple)
{
    if (multiple > 0)
    {
	// If not calibrated, need calibrate
	if (!useCalibrate)
	{
	    calibrate();
	}
	
	// Calculate threshold vectors
	t.XAxis = thresholdX * multiple;
	t.YAxis = thresholdY * multiple;
	t.ZAxis = thresholdZ * multiple;
    } else
    {
	// No threshold
	t.XAxis = 0;
	t.YAxis = 0;
	t.ZAxis = 0;
    }

    // Remember old threshold value
    actualThreshold = multiple;
}

