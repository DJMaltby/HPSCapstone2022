#include <Wire.h>
#include <I2C_Anything.h>

#include <AHRS_Nano33BLE_LSM9DS1.h>

void receiveEvent(int numBytes);

void requestEvent();

/*
union floatToBytes {
  
    char byteValue[4];
    float floatValue;
  
} roll, pitch, yaw; */

// enum {ROLL, PITCH, YAW};

float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;

char MODE;

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

  MODE = '0';
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

  roll = IMU.rollDegrees();    //you could also use rollRadians()
  pitch = IMU.pitchDegrees();
  yaw = IMU.yawDegrees();

  /*
  roll.floatValue = IMU.rollDegrees();    //you could also use rollRadians()
  pitch.floatValue = IMU.pitchDegrees();
  yaw.floatValue = IMU.yawDegrees();

  roll_s = (int) roll;
  pitch_s = (int) pitch;
  yaw_s = (int) yaw; */
}

void receiveEvent(int numBytes) {
  MODE = Wire.read();  
}

void requestEvent() {
  if (MODE == '1') {
    I2C_writeAnything(gx);
    I2C_writeAnything(gy);
    I2C_writeAnything(gz);  
  } else if (MODE == '2') {
    I2C_writeAnything(ax);
    I2C_writeAnything(ay);
    I2C_writeAnything(az);  
  } else if (MODE == '3') {
    I2C_writeAnything(mx);
    I2C_writeAnything(my);
    I2C_writeAnything(mz);
  } else {
    I2C_writeAnything(roll);
    I2C_writeAnything(pitch);
    I2C_writeAnything(yaw);
  }
  /*
  I2C_writeAnything(gx);
  I2C_writeAnything(gy);
  I2C_writeAnything(gz);

  I2C_writeAnything(ax);
  I2C_writeAnything(ay);
  I2C_writeAnything(az);

  I2C_writeAnything(mx);
  I2C_writeAnything(my);
  I2C_writeAnything(mz); */

  /*
  if (CMD == ROLL) {
    I2C_writeAnything(roll);
    Wire.write(roll.byteValue, 4);
  }
  if (CMD == PITCH) {
    I2C_
    Wire.write(pitch.byteValue, 4);  
  }
  if (CMD == YAW) {
    Wire.write(yaw.byteValue, 4);  
  } */
}
