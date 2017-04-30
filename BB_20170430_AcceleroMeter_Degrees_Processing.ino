/*
* The code is released under the GNU General Public License.
* Developed by www.codekraft.it
* 
* EVEN THOUGH IT IS SAID, THAT THIS CODE SHOULD WORK OUT, IT DOESN'T DO AT ALL
*/

#include "BMI160.h"
#include "CurieIMU.h"

#include "MadgwickAHRS.h"
Madgwick madG;

#define M_PI   3.14159265358979323846264338327950288

// Repeat part of the code every X miliseconds
#define runEvery(t) for (static long _lasttime;\
                         (uint16_t)((uint16_t)millis() - _lasttime) >= (t);\
                         _lasttime += (t))

//Set up a timer Variable
uint32_t timer;

int ax, ay, az;         // accelerometer values
int gx, gy, gz;         // gyrometer values
float yaw, pitch, roll;
float ypr[3];

int factor = 200; // variable by which to divide gyroscope values, used to control sensitivity
// note that an increased baud rate requires an increase in value of factor

const int ledPin = 13;      // activity LED pin
boolean blinkState = false; // state of the LED

void setup() {
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open
  
  Serial.println("Initializing IMU device...");

  CurieIMU.begin();
  CurieIMU.initialize(); // This seems to be a bad line of the code, if this method is use without the one before, nothing happens

  CurieIMU.setFullScaleGyroRange(BMI160_GYRO_RANGE_500);
  CurieIMU.setGyroRate(BMI160_GYRO_RATE_100HZ);
  CurieIMU.setFullScaleAccelRange(BMI160_ACCEL_RANGE_2G);


    //IMU device must be resting in a horizontal position for the following calibration procedure to work correctly!

    //Serial.print("Starting Gyroscope calibration...");
    CurieIMU.autoCalibrateGyroOffset();
    //Serial.println(" Done");
    //Serial.print("Starting Acceleration calibration...");
    CurieIMU.autoCalibrateXAccelOffset(0);
    CurieIMU.autoCalibrateYAccelOffset(0);
    CurieIMU.autoCalibrateZAccelOffset(1);
    //Serial.println(" Done");

    //Serial.print("Enabling Gyroscope/Acceleration offset compensation...");
    CurieIMU.setGyroOffsetEnabled(true);
    CurieIMU.setAccelOffsetEnabled(true);
    //Serial.println(" Enabling OK");

    timer = micros();       // Initialize timer
    //Serial.println(" Timer initialized");

    Serial.println(" Initialization done!");
}

void loop() {
  runEvery(10){         // Exetutes this part of the code every 10 miliseconds -> 100Hz
    timer = micros();    // Reset the timer  
    // read raw accel/gyro measurements from device
    ax = CurieIMU.getAccelerationX();
    ay = CurieIMU.getAccelerationY();
    az = CurieIMU.getAccelerationZ();
    gx = CurieIMU.getRotationX();
    gy = CurieIMU.getRotationY();
    gz = CurieIMU.getRotationZ();
  
    // use function from MagdwickAHRS.h to return quaternions
    madG.updateIMU(gx / factor, gy / factor, gz / factor, ax, ay, az);
      
    ypr[0] = madG.getYaw()* 180.0 / M_PI;
    ypr[2] = madG.getRoll()* 180.0 / M_PI;
    ypr[1] = madG.getPitch()* 180.0 / M_PI;
    
    serialPrintFloatArr(ypr,3);
    Serial.print('\n');
/*    Serial.print("yaw:\t");
    Serial.print(ypr[0]);
    Serial.print("\t roll:\t");
    Serial.print(ypr[2]);
    Serial.print("\t pitch:\t");
    Serial.println(ypr[1]);
*/    
    runEvery(1000){
      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(ledPin, blinkState);
    }
  }
}


void serialPrintFloatArr(float * arr, int length) {
  for(int i=0; i<length; i++) {
    serialFloatPrint(arr[i]);
    Serial.print(",");
  }
}


void serialFloatPrint(float f) {
  byte * b = (byte *) &f;
  for(int i=0; i<4; i++) {
    
    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);
    
    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    
    Serial.print(c1);
    Serial.print(c2);
  }
}


void writeArr(void * varr, uint8_t arr_length, uint8_t type_bytes) {
  byte * arr = (byte*) varr;
  for(uint8_t i=0; i<arr_length; i++) {
    writeVar(&arr[i * type_bytes], type_bytes);
  }
}


// thanks to Francesco Ferrara and the Simplo project for the following code!
void writeVar(void * val, uint8_t type_bytes) {
  byte * addr=(byte *)(val);
  for(uint8_t i=0; i<type_bytes; i++) {
    Serial.write(addr[i]);
  }
}
