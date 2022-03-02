#include <Wire.h>

int roll, pitch, yaw;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.requestFrom(8, 3);

  if (Wire.available() >= 3) {
    roll = Wire.read();
    pitch = Wire.read();
    yaw = Wire.read();

    Serial.print("roll: ");
    Serial.print(roll);
    Serial.print(",  pitch:");
    Serial.print(pitch);
    Serial.print(",  yaw:");
    Serial.println(yaw);
  }
  
  delay(500);
}
