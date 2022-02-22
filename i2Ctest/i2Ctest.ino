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

// #include <AHRS_Nano33BLE_LSM9DS1.h>
#include <Wire.h>
#include "AHRS_Nano33BLE_LSM9DS1.h"

#define mainAddress 0x3C
#define OUT_X_H_G 0x19
#define OUT_Y_H_G 0x1B
#define OUT_Z_H_G 0x1D

// Pin definitions
int myLed  = 13;

// Variables
float gxi, gyi, gzi, gxo, gyo, gzo, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;
float temperature;

void setup()
{
  Serial.begin(38400);
  while (!Serial);
  Serial.println("Waited for Serial.");

  digitalWrite(7, HIGH);
  
  // Initialize LED pin
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  
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

    IMU.calibrateMag();
    float* magBias = IMU.getMagBias();
    Serial.println("mag biases (mG)"); Serial.println(1000.*magBias[0]); Serial.println(1000.*magBias[1]); Serial.println(1000.*magBias[2]); 

    IMU.initLSM9DS1(); 
    Serial.println("LSM9DS1 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

    // ADDED
    /* Wire.beginTransmission(mainAddress);
    Wire.write(8);
    Wire.endTransmission(); */

    Serial.println("Assumption is data is in the following format:");
    Serial.println("uptime (milliseconds), roll (degrees), pitch (degrees), yaw (degrees), gyrotemperatureC (Celcius)");
    Serial.println("Begin Outputting Data!");
  }
}

void loop()
{  
  if (IMU.accelerometerReady()) {  // check if new accel data is ready  
    IMU.readAccel(ax, ay, az);
  }

  if (IMU.gyroscopeReady()) {  // check if new gyro data is ready  
    IMU.readGyro(gxi, gyi, gzi);
  }

  if (IMU.magnometerReady()) {  // check if new mag data is ready  
    IMU.readMag(mx, my, mz);
  }

  deltat = IMU.updateDeltat(); //this have to be done before calling the fusion update
  IMU.MadgwickQuaternionUpdate(ax, ay, az, gxi*PI/180.0f, gyi*PI/180.0f, gzi*PI/180.0f, -mx, my, mz, deltat);

  roll = IMU.rollDegrees();    //you could also use rollRadians()
  pitch = IMU.pitchDegrees();
  yaw = IMU.yawDegrees();
  temperature = IMU.readTempC();

  Wire.beginTransmission(mainAddress);
  Wire.write(OUT_X_H_G);
  Wire.write(OUT_Y_H_G);
  Wire.write(OUT_Z_H_G);

  Wire.endTransmission();

  Wire.requestFrom(mainAddress, 3);

  if (Wire.available() <= 3) {
    gxo = Wire.read();
    gyo = Wire.read();
    gzo = Wire.read();

    Serial.print("gx = ");
    Serial.print(gxo);
    Serial.print("   gy = ");
    Serial.print(gyo);
    Serial.print("   gz = ");
    Serial.println(gzo);
  }

/*  Serial.print(millis());
  Serial.print(',');
  Serial.print(roll);
  Serial.print(',');
  Serial.print(pitch);
  Serial.print(',');
  Serial.print(yaw);
  Serial.print(',');
  Serial.println(temperature); */

  digitalWrite(myLed, !digitalRead(myLed));

}
