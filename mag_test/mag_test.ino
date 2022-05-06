#include <Arduino_LSM9DS1.h>
#include <SPI.h>
#include <SD.h>

//#include <Wire.h>
//#include "HMC5883L.h"

//HMC5883L compass; //Copy the folder "HMC5883L" in the folder "C:\Program Files\Arduino\libraries" and restart the arduino IDE.

float x, y, z;

//SD Card Variables
File ddata;
String file = "mag_raw8.txt";

void setup()
{   
  Serial.begin(9600);
  while (!Serial);

  if (!IMU2.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  IMU2.setMagnetODR(8);      // (0..8)->{0.625,1.25,2.5,5.0,10,20,40,80,400}Hz
  IMU2.setMagnetFS(1);       // 0=±400.0; 1=±800.0; 2=±1200.0 , 3=±1600.0  (µT) 

  if (!SD.begin(4)) { //Select the CS pin (Pin 4)
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    while(1);         
  }
  ddata = SD.open(file, FILE_WRITE);
  if (ddata) {
    // If you see this message, you can unplug the USB cable.
    //  Data will start to be collected on the SD card.
    Serial.println("SD Card Initialization Complete");
  } else {
    // if the file didn't open, print an error:
    Serial.println("Error: cannot write to file");
    while(1);
  }   
}

void loop()
{
  //if (IMU.magneticFieldAvailable()) {
  //  IMU.readMagneticField(x,y,z);
  //}

  // Uncalibrated, raw data
  if (IMU2.magnetAvailable()) {
    IMU2.readRawMagnet(x,y,z);
  }
  
  ddata.flush(); 
  ddata.print(x * 1000.0);        // raw output in microTesla (could be wrong here), data needs to be in nanoTesla
  ddata.print("         ");
  ddata.print(y * 1000.0);
  ddata.print("         ");
  ddata.print(z * 1000.0);
  ddata.println();

  /*
  Serial.print(xv); 
  Serial.print(",");
  Serial.print(yv);
  Serial.print(",");
  Serial.print(zv);
  Serial.println(); */

  delay(10); 
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
