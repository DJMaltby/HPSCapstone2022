#include <LSM9DS1.h>

#include <Wire.h>

// one of these two (may be) the address for LSM9DS1
// #define mainAddress 0x3C
#define mainAddress 0x6B

// x-axis gyroscope outputs
#define OUT_X_L_G 0x18
#define OUT_X_H_G 0x19

// y-axis gyroscope outputs
#define OUT_Y_L_G 0x1A
#define OUT_Y_H_G 0x1B

// z-axis gyroscope outputs
#define OUT_Z_L_G 0x1C
#define OUT_Z_H_G 0x1D

// Control register 1 for gyroscope
#define CTRL_REG1_G 0x10

int gx, gy, gz, ax, ay, az;

void setup() {
  Wire.begin();
  Serial.begin(38400);
  delay(100);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ"); 

  Wire.beginTransmission(mainAddress);

  // 119 Hz, 2000 dps, 16 Hz BW
  Wire.write(CTRL_REG1_G);
  Wire.write(78);
  
  Wire.endTransmission(); 
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(mainAddress);
  Wire.write(OUT_X_L_G);
  Wire.write(OUT_Y_L_G);
  Wire.write(OUT_Z_L_G);

  Wire.endTransmission();

  Wire.requestFrom(mainAddress, 3);

  Serial.print("bytes read: ");
  Serial.println(Wire.available());

  if (3 >= Wire.available()) {
    gx = Wire.read();
    gy = Wire.read();
    gz = Wire.read();

    Serial.print("gx = ");
    Serial.print(gx);
    Serial.print("   gy = ");
    Serial.print(gy);
    Serial.print("   gz = ");
    Serial.println(gz);
  }

}
