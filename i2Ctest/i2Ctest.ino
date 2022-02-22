#include <Wire.h>

#define mainAddress 0x3C
#define OUT_X_L_G 0x18
#define OUT_X_H_G 0x19
#define OUT_Y_L_G 0x1A
#define OUT_Y_H_G 0x1B
#define OUT_Z_L_G 0x1C
#define OUT_Z_H_G 0x1D
#define CTRL_REG3_G 0x12

int gx, gy, gz, ax, ay, az;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(38400);
  delay(100);

  Wire.beginTransmission(mainAddress);
  Wire.write(CTRL_REG3_G);
  Wire.write(1<<7);
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

  if (Wire.available() <= 3) {
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
