#include <Wire.h>
#include <I2C_Anything.h>

#include <AHRS_Nano33BLE_LSM9DS1.h>

void getReferenceData();
void serialDiagnostics();

/*
union floatToBytes {
  
    char byteValue[4];
    float floatValue;
  
} roll, pitch, yaw; */

// enum {ROLL, PITCH, YAW};

float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;

// Main Module Variables
float gx_m, gy_m, gz_m, ax_m, ay_m, az_m, mx_m, my_m, mz_m;
float roll_m, pitch_m, yaw_m;

char MODE;

// gyroData roll, pitch, yaw;

void setup() {
  Wire.begin();
  Serial.begin(38400);
  while (!Serial);
  Serial.println("Waited for Serial.");

  if (!IMU.start()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("Perform gyro and accel self test.");
  if (!IMU.selftestLSM9DS1()) {
    Serial.println("Failed self test"); // check function of gyro and accelerometer via self test
    while (1);
  } else {  
    Serial.println("LSM9DS1 is online and passed self test...");
  
    Serial.print("accel sensitivity is "); Serial.print(IMU.accelerationSensitivity()); Serial.println(" LSB/mg");
    Serial.print("gyro sensitivity is "); Serial.print(IMU.gyroscopeSensitivity()); Serial.println(" LSB/mdps");
    Serial.print("mag sensitivity is "); Serial.print(IMU.magnometerSensitivity()); Serial.println(" LSB/mGauss");
  }

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

  MODE = '0';
}

void loop() {
  // FOR TESTING
  if (Serial.available() > 0) {
    MODE = Serial.read();      
  }
  
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

  getReferenceData();
  serialDiagnostics();

  /*
  roll.floatValue = IMU.rollDegrees();    //you could also use rollRadians()
  pitch.floatValue = IMU.pitchDegrees();
  yaw.floatValue = IMU.yawDegrees();

  roll_s = (int) roll;
  pitch_s = (int) pitch;
  yaw_s = (int) yaw; */
}

/*
void receiveEvent(int numBytes) {
  MODE = Wire.read();  
} */

/*void requestEvent() {
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
  } 
} */

void getReferenceData() {
  Wire.beginTransmission(8);
  Wire.write(MODE);
  Wire.endTransmission();
  
  Wire.requestFrom(8,12);
  if (Wire.available() >= 12) {
    if (MODE == '1') {
      I2C_readAnything(gx_m);
      I2C_readAnything(gy_m);
      I2C_readAnything(gz_m);         
    } else if (MODE == '2') {
      I2C_readAnything(ax_m);
      I2C_readAnything(ay_m);
      I2C_readAnything(az_m);    
    } else if (MODE == '3') {
      I2C_readAnything(mx_m);
      I2C_readAnything(my_m);
      I2C_readAnything(mz_m);
    } else {
      I2C_readAnything(roll_m);
      I2C_readAnything(pitch_m);
      I2C_readAnything(yaw_m);  
    }
  }
}

void serialDiagnostics() {
 // Serial.print(millis());
 // Serial.print(',');

 if (MODE == '1') {
  Serial.print("roll velocities: ");
  Serial.print(gx);
  Serial.print(", ");
  Serial.print(gx_m);
  Serial.print(";   pitch velocities: ");
  Serial.print(gy);
  Serial.print(", ");
  Serial.print(gy_m);
  Serial.print(";   yaw velocities: ");
  Serial.print(gz);
  Serial.print(", ");
  Serial.println(gz_m);
 } else if (MODE == '2') {
  Serial.print("x-axis accelerations: ");
  Serial.print(ax);
  Serial.print(", ");
  Serial.print(ax_m);
  Serial.print(";   y-axis accelerations: ");
  Serial.print(ax);
  Serial.print(", ");
  Serial.print(ay_m);
  Serial.print(";   z-axis accelerations: ");
  Serial.print(az);
  Serial.print(", ");
  Serial.println(az_m);
 } else if (MODE == '3') {
  Serial.print("x-axis magnetic fields: ");
  Serial.print(mx);
  Serial.print(", ");
  Serial.print(mx_m);
  Serial.print(";   y-axis magnetic fields: ");
  Serial.print(my);
  Serial.print(", ");
  Serial.print(my_m);
  Serial.print(";   z-axis magnetic fields: ");
  Serial.print(mz);
  Serial.print(", ");
  Serial.println(mz_m);
 } else {
  Serial.print(-1 * roll_m);
  Serial.print(" ");
  Serial.print(-1 * roll);
  Serial.print(" ");
  Serial.print(-1 * pitch_m);
  Serial.print(" ");
  Serial.print(-1 * pitch);
  Serial.print(" ");
  Serial.print(-1 * yaw_m);
  Serial.print(" ");
  Serial.println(-1 * yaw);
 }
}
