#include <Wire.h>

#include <AHRS_Nano33BLE_LSM9DS1.h>

float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;

//int gx_s, gy_s, gz_s, ax_s, ay_s, az_s, mx_s, my_s, mz_s;
int gx_m, gy_m, gz_m, ax_m, ay_m, az_m, mx_m, my_m, mz_m;
int gx_t, gy_t, gz_t, ax_t, ay_t, az_t, mx_t, my_t, mz_t;

int roll_s, pitch_s, yaw_s;
int roll_m, pitch_m, yaw_m;
int roll_t, pitch_t, yaw_t;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);

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

  roll = IMU.rollDegrees();    //you could also use rollRadians()
  pitch = IMU.pitchDegrees();
  yaw = IMU.yawDegrees();

  ax_m = (int) ax;
  ay_m = (int) ay;
  az_m = (int) az;

  gx_m = (int) gx;
  gy_m = (int) gy;
  gz_m = (int) gz;

  mx_m = (int) mx;
  my_m = (int) my;
  mz_m = (int) mz;

  roll_m = (int) roll;
  pitch_m = (int) pitch;
  yaw_m = (int) yaw;
  
  Wire.requestFrom(8,12);

  if (Wire.available() >= 12) {
    gx_t = gx_m - Wire.read();
    gy_t = gy_m - Wire.read();
    gz_t = gz_m - Wire.read();

    ax_t = ax_m - Wire.read();
    ay_t = ay_m - Wire.read();
    az_t = az_m - Wire.read();

    mx_t = mx_m - Wire.read();
    my_t = my_m - Wire.read();
    mz_t = mz_m - Wire.read();
    
    roll_s = Wire.read();
    if (roll_s > 180) {
      roll_s = roll_s - 255;  
    }
    pitch_s = Wire.read();
    if (pitch_s > 180) {
      pitch_s = pitch_s - 255;  
    }
    yaw_s = Wire.read();
    if (yaw_s > 180) {
      yaw_s = yaw_s - 255;  
    }
  }

  roll_t = roll_m - roll_s;
  pitch_t = pitch_m - pitch_s;
  yaw_t = yaw_m - yaw_s;

  Serial.println();

  Serial.print("roll velocity: ");
  Serial.print(gx_t);
  Serial.print(",  pitch velocity: ");
  Serial.print(gy_t);
  Serial.print(",  yaw velocity: ");
  Serial.println(gz_t);

  Serial.print("x-axis acceleration: ");
  Serial.print(ax_t);
  Serial.print(",  y-axis acceleration: ");
  Serial.print(ay_t);
  Serial.print(",  z-axis acceleration: ");
  Serial.println(az_t);

  Serial.print("x-axis magnetic field: ");
  Serial.print(mx_t);
  Serial.print(",  y-axis magnetic field: ");
  Serial.print(my_t);
  Serial.print(",  z-axismagnetic field: ");
  Serial.println(mz_t);
  
  Serial.print("roll: ");
/*  Serial.print(roll_s);
  Serial.print(" - ");
  Serial.print(roll_m);
  Serial.print(" = "); */
  Serial.print(roll_t);
  
  Serial.print(",  pitch: ");
/*  Serial.print(pitch_s);
  Serial.print(" - ");
  Serial.print(pitch_m);
  Serial.print(" = "); */
  Serial.print(pitch_t);
  
  Serial.print(",  yaw: ");
/*  Serial.print(yaw_s);
  Serial.print(" - ");
  Serial.print(yaw_m);
  Serial.print(" = "); */
  Serial.println(yaw_t);

  delay(50);

/*
  for (i = ROLL; i > YAW; i++) {
    Wire.beginTransmission(8);
    Wire.write(i);
    Wire.endTransmission();
    
    Wire.requestFrom(8, 4);
    
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
  }
  
  Serial.print("roll: ");
  Serial.print(roll.floatValue);
  Serial.print(",  pitch:");
  Serial.print(pitch.floatValue);
  Serial.print(",  yaw:");
  Serial.println(yaw.floatValue);

  i = ROLL;
  
  delay(500); */
}
