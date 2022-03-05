#include <Wire.h>

union floatToBytes {
  char byteValue[4];
  float floatValue;
  
} roll, pitch, yaw;

enum {ROLL, PITCH, YAW};

int i;
int charIndex = 0;

// int roll, pitch, yaw;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
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
  
  delay(500);
}
