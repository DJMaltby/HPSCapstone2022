#include <Wire.h>

#include <AHRS_Nano33BLE_LSM9DS1.h>

float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;

int gx_s, gy_s, gz_s, ax_s, ay_s, az_s, mx_s, my_s, mz_s;
int roll_s, pitch_s, yaw_s;

void setup() {
  Wire.begin(8);
  Wire.onRequest(requestEvent);

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

  roll = IMU.rollDegrees();    // you could also use rollRadians()
  pitch = IMU.pitchDegrees();
  yaw = IMU.yawDegrees();

  ax_s = (int) ax;
  ay_s = (int) ay;
  az_s = (int) az;

  gx_s = (int) gx;
  gy_s = (int) gy;
  gz_s = (int) gz;

  mx_s = (int) mx;
  my_s = (int) my;
  mz_s = (int) mz;

  roll_s = (int) roll;
  pitch_s = (int) pitch;
  yaw_s = (int) yaw;  
}

void requestEvent() {
  Wire.write(gx_s);
  Wire.write(gy_s);
  Wire.write(gz_s);

  Wire.write(ax_s);
  Wire.write(ay_s);
  Wire.write(az_s);

  Wire.write(mx_s);
  Wire.write(my_s);
  Wire.write(mz_s);
  
  Wire.write(roll_s);
  Wire.write(pitch_s);
  Wire.write(yaw_s);

  /*
  if (CMD == ROLL) {
    Wire.write(roll.byteValue, 4);  
  }
  if (CMD == PITCH) {
    Wire.write(pitch.byteValue, 4);  
  }
  if (CMD == YAW) {
    Wire.write(yaw.byteValue, 4);  
  } */
}
