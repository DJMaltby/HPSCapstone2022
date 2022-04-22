#include <Arduino_LSM9DS1.h>

//#include <Wire.h>
//#include "HMC5883L.h"

//HMC5883L compass; //Copy the folder "HMC5883L" in the folder "C:\Program Files\Arduino\libraries" and restart the arduino IDE.

float x, y, z;

void setup()
{   
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  IMU.setMagnetODR(8);
  IMU.setMagnetFS(0);
  //Wire.begin();  
  //compass = HMC5883L();  
  //setupHMC5883L();       
}

void loop()
{
  //if (IMU.magneticFieldAvailable()) {
  //  IMU.readMagneticField(x,y,z);
  //}

  // Uncalibrated, raw data
  if (IMU.magnetAvailable()) {
    IMU.readRawMagnet(x,y,z);
  }
  
  Serial.flush(); 
  Serial.print(x * 1000.0);        // raw output in microTesla (could be wrong here), data needs to be in nanoTesla
  Serial.print("  ");
  Serial.print(y * 1000.0);
  Serial.print("  ");
  Serial.print(z * 1000.0);
  Serial.println();

  /*
  Serial.print(xv); 
  Serial.print(",");
  Serial.print(yv);
  Serial.print(",");
  Serial.print(zv);
  Serial.println(); */

  //delay(100); 
} 

/*
void setupHMC5883L()
{  
  compass.SetScale(0.88);
  compass.SetMeasurementMode(Measurement_Continuous);
}
 
void getHeading()
{ 
  MagnetometerRaw raw = compass.ReadRawAxis();
  xv = (float)raw.XAxis;
  yv = (float)raw.YAxis;
  zv = (float)raw.ZAxis;
} */
