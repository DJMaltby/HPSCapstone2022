#include <Wire.h>

#include "AHRS_Nano33BLE_LSM9DS1.h"

void receiveEvent(int numBytes);

void requestEvent();

union floatToBytes {
  
    char byteValue[4];
    float floatValue;
  
} roll, pitch, yaw;

enum {ROLL, PITCH, YAW};

float gx, gy, gz, ax, ay, az, mx, my, mz;
// float pitch, roll, yaw;
float deltat;

int CMD = 0;

// gyroData roll, pitch, yaw;

void setup() {
  Wire.begin(8);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);

  if (!IMU.start()) {
    while (1);
  } 

  if (!IMU.selftestLSM9DS1()) {
    while (1);
  }

  IMU.calibrateAccelGyro();
  float* accelBias = IMU.getAccelBias();
  float* gyroBias = IMU.getGyroBias();

  IMU.calibrateMag();
  float* magBias = IMU.getMagBias();

  IMU.initLSM9DS1();
}

void loop() {
  if (IMU.accelerometerReady()) {  // check if new accel data is ready  
    IMU.readAccel(ax, ay, az);
  } 

  if (IMU.gyroscopeReady()) {  // check if new gyro data is ready  
    IMU.readGyro(gx, gy, gz);
  }

  if (IMU.magnometerReady()) {  // check if new mag data is ready  
    IMU.readMag(mx, my, mz);
  }

  deltat = IMU.updateDeltat(); //this have to be done before calling the fusion update
  IMU.MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, -mx, my, mz, deltat);

  roll.floatValue = IMU.rollDegrees();    //you could also use rollRadians()
  pitch.floatValue = IMU.pitchDegrees();
  yaw.floatValue = IMU.yawDegrees();

/*  roll_s = (int) roll;
  pitch_s = (int) pitch;
  yaw_s = (int) yaw; */
}

void receiveEvent(int numBytes) {
  if (Wire.available()) {
    CMD = Wire.read();  
  }
}

void requestEvent() {
  if (CMD == ROLL) {
    Wire.write(roll.byteValue, 4);
  }
  if (CMD == PITCH) {
    Wire.write(pitch.byteValue, 4);  
  }
  if (CMD == YAW) {
    Wire.write(yaw.byteValue, 4);  
  }
}
