/*
  Arduino Nano 33 BLE LSM9DS1 -  Simple AHRS Output

  This example reads the acceleration, angular velocity, and magnetic field values from the LSM9DS1
  sensor, feeds them into a Madgwick filter algorithm, and continuously prints the resulting
  roll, pitch, and yaw approximations to the Serial Monitor or Serial Plotter.

  The circuit:
  - Arduino Nano 33 BLE with LSM9DS1

  created  08 Mar 2021
  by Miller Sakmar

  This example code is in the public domain.
*/

#include <AHRS_Nano33BLE_LSM9DS1.h>
#include <Arduino_LSM9DS1.h>

// Pin definitions
int myLed  = 13;

// Variables
float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;
float temperature;

//Magnetometer calibration variables
float mx_raw, my_raw, mz_raw;
float mx_off, my_off, mz_off;

// press 'o' to display live data feed, any other key to stop
char MODE;

int clk = 0;

void setup()
{
  Serial.begin(38400);
  while (!Serial);
  Serial.println("Waited for Serial.");
  
  // Initialize LED pin
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  if (!IMU2.begin()) { Serial.println(F("Failed to initialize IMU!")); while (1);  }
  IMU2.setMagnetODR(8);
  IMU2.setMagnetFS(0);

  MODE = 'o';
  
  if (!IMU.start())
  {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("Perform gyro and accel self test.");
  if (!IMU.selftestLSM9DS1())
  {
    Serial.println("Failed self test"); // check function of gyro and accelerometer via self test
    while (1);
  }
  else
  {  
    Serial.println("LSM9DS1 is online and passed self test...");

    Serial.print("accel sensitivity is "); Serial.print(IMU.accelerationSensitivity()); Serial.println(" LSB/mg");
    Serial.print("gyro sensitivity is "); Serial.print(IMU.gyroscopeSensitivity()); Serial.println(" LSB/mdps");
    Serial.print("mag sensitivity is "); Serial.print(IMU.magnometerSensitivity()); Serial.println(" LSB/mGauss");

    Serial.println("Calibrate gyro and accel");
    IMU.calibrateAccelGyro(); // Calibrate gyro and accelerometers, load biases in bias registers

    float* accelBias = IMU.getAccelBias();
    float* gyroBias = IMU.getGyroBias();

    Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
    Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);

//    IMU.calibrateMag();
//    float* magBias = IMU.getMagBias();
//    Serial.println("mag biases (mG)"); Serial.println(1000.*magBias[0]); Serial.println(1000.*magBias[1]); Serial.println(1000.*magBias[2]); 

    IMU.initLSM9DS1(); 
    Serial.println("LSM9DS1 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  }
}

void loop() {
  if (Serial.available() > 0) {
      MODE = Serial.read();      
  }
    
  if (IMU.accelerometerReady()) {  // check if new accel data is ready  
    IMU.readAccel(ax, ay, az);
  } 

  if (IMU.gyroscopeReady()) {  // check if new gyro data is ready  
    IMU.readGyro(gx, gy, gz);
  }

//  if (IMU.magnometerReady()) {  // check if new mag data is ready  
//    IMU.readMag(mx, my, mz);
//  }

  if (IMU2.magnetAvailable()) {
    IMU2.readRawMagnet(mx_raw, my_raw, mz_raw);  
  } 
//
//  // MAG4 
//  mx_off = (mx_raw * 1000) + 5592.424495;
//  my_off = (my_raw * 1000) - 9392.597321;
//  mz_off = (mz_raw * 1000) + 24054.460811;
//
//  mx = 0.928366*mx_off + 0.030235*my_off - 0.023291*mz_off;
//  my = 0.030235*mx_off + 0.984991*my_off + 0.007333*mz_off;
//  mz = -0.023291*mx_off + 0.007333*my_off + 0.911193*mz_off;

  // MAG_RAW8
  mx_off = (mx_raw * 1000) - 6884.941836;
  my_off = (my_raw * 1000) - 24358.428846;
  mz_off = (mz_raw * 1000) + 28659.439568;

  mx = 1.099547*mx_off + 0.047500*my_off - 0.119563*mz_off;
  my = 0.047500*mx_off + 1.139382*my_off - 0.006013*mz_off;
  mz = -0.119563*mx_off - 0.006013*my_off + 1.152146*mz_off;

  deltat = IMU.updateDeltat(); //this have to be done before calling the fusion update
  
  // Mag units in nT, need to be in mG
  IMU.MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, -mx*0.01, -my*0.01, mz*0.01, deltat);
//  IMU.MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, -mx, -my, mz, deltat);

  roll = IMU.rollDegrees();    //you could also use rollRadians()
  pitch = IMU.pitchDegrees();
  yaw = IMU.yawDegrees();

  clk++;
  if (clk >= 10) {
    if (MODE == 'o') {
      Serial.print(millis());
      Serial.print("        ");
      Serial.print(-1 * roll);
      Serial.print("        ");
      Serial.print(-1 * pitch);
      Serial.print("        "); 
      Serial.print(-1 * yaw);
      Serial.println();
      clk = 0;
    } else {
      //do nothing, stop printing data  
    }
  }
}
