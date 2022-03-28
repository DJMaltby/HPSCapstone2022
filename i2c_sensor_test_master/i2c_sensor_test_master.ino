#include <Wire.h>
#include <I2C_Anything.h>

#include <AHRS_Nano33BLE_LSM9DS1.h>

/*
union floatToBytes {
  char byteValue[4];
  float floatValue;
  
} roll, pitch, yaw;

enum {ROLL, PITCH, YAW};

int i;
int charIndex = 0; */

// int roll, pitch, yaw;

float gx, gy, gz, ax, ay, az, mx, my, mz;
float deltat;

float roll_m, pitch_m, yaw_m;
float roll_s, pitch_s, yaw_s;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(38400);
  Serial.println("Waited for Serial.");
  
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

    Serial.println("Calibrating gyro and accel");
    IMU.calibrateAccelGyro(); // Calibrate gyro and accelerometers, load biases in bias registers

    float* accelBias = IMU.getAccelBias();
    float* gyroBias = IMU.getGyroBias();

    Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
    Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);

    Serial.println("Calibrating mag");
    IMU.calibrateMag();
    float* magBias = IMU.getMagBias();
    Serial.println("mag biases (mG)"); Serial.println(1000.*magBias[0]); Serial.println(1000.*magBias[1]); Serial.println(1000.*magBias[2]); 

    IMU.initLSM9DS1(); 
    Serial.println("LSM9DS1 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  }
}

void loop() {
  // put your main code here, to run repeatedly:
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

  roll_m = IMU.rollDegrees();    //you could also use rollRadians()
  pitch_m = IMU.pitchDegrees();
  yaw_m = IMU.yawDegrees();

  Wire.requestFrom(8,12);

  if (Wire.available() >= 12) {
    I2C_readAnything(roll_s);
    I2C_readAnything(pitch_s);
    I2C_readAnything(yaw_s);  
  }

  /*
  for (i = ROLL; i > YAW; i++) {
    Wire.beginTransmission(8);
    Wire.write(i);
    Wire.endTransmission();
    
    Wire.requestFrom(8, 4);

    if (i == ROLL && Wire.available() >= 4) {
      for (charIndex = 0; charIndex < 4; charIndex++) {
        roll.byteValue[charIndex] = Wire.read();  
      } 
    }

    if (i == PITCH && Wire.available() >= 4) {
      for (charIndex = 0; charIndex < 4; charIndex++) {
        pitch.byteValue[charIndex] = Wire.read();  
      } 
    }

    if (i == YAW && Wire.available() >= 4) {
      for (charIndex = 0; charIndex < 4; charIndex++) {
        yaw.byteValue[charIndex] = Wire.read();  
      }
    }
    
/*
    if (i == ROLL) {
      while (Wire.available()) {
        roll.byteValue[charIndex] = Wire.read();
        charIndex++;
      }
    }

    if (i == PITCH) {
       while (Wire.available()) {
        pitch.byteValue[charIndex] = Wire.read();
        charIndex++;
      } 
    }

    if (i == YAW) {
       while (Wire.available()) {
        yaw.byteValue[charIndex] = Wire.read();
        charIndex++;
      } 
    }

    charIndex = 0;
  } */
  
  Serial.print("roll: ");
  Serial.print(roll_m - roll_s);
  Serial.print(",  pitch:");
  Serial.print(pitch_m - pitch_s);
  Serial.print(",  yaw:");
  Serial.println(yaw_m - yaw_s);

  // i = ROLL;
  
  // delay(100);
}
