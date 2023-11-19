#include "MPU9250.h"
#include <math.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#define ss Serial2
#define laserSerial Serial1

MPU9250 mpu;

// Baud Rate Setup
static const uint32_t SerialMonitorBaud = 115200;
static const uint32_t GPSBaud = 9600;
static const uint32_t laserBaud = 9600;

// Devices Objects
TinyGPSPlus gps;

unsigned char buf[50]; 
int index = 0; 

void SetupLaserSerial(){
  laserSerial.begin(laserBaud);
}

void laserOn(){
  byte data[9] = {0xAE, 0xA7, 0x05, 0x00, 0x40, 0x01, 0x46, 0xBC, 0xBE};  //Laser ON
  laserSerial.write(data, sizeof(data));
  delay(500);
}

void laserOff(){
  byte data[9] = {0xAE, 0xA7, 0x05, 0x00, 0x40, 0x00, 0x45, 0xBC, 0xBE};  //Laser OFF
  laserSerial.write(data, sizeof(data));
  delay(500);
}

void sendSingleMeasurementSignal(){
  byte data[8] = {0xAE, 0xA7, 0x04, 0x00, 0x05, 0x09, 0xBC, 0xBE};  //Single Measure
  laserSerial.write(data, sizeof(data));
  delay(500);
}

void sendContinusMeasurementSignal(){
  byte data[8] = {0xAE, 0xA7, 0x04, 0x00, 0x0E, 0x12, 0xBC, 0xBE};  //Continus Measure
  laserSerial.write(data, sizeof(data));
  delay(500);
}

void sendStopMeasurementSignal(){
  byte data[8] = {0xAE, 0xA7, 0x04, 0x00, 0x0F, 0x13, 0xBC, 0xBE};  //Stop Measure
  laserSerial.write(data, sizeof(data));
  delay(500);
}

bool isLaserSerialAvailable(){
  return laserSerial.available() > 0;
}

unsigned char readLaserSerial(){
  return laserSerial.read();
}

float waitForDistance(){
  while(true)
  if (isLaserSerialAvailable()){
    buf[index] = readLaserSerial();
    index++;
    if (buf[index-1] == 190){  
      if(uint8_t(buf[4])==133)
      {
        float distance = float(uint8_t(buf[7])*256+uint8_t(buf[8]))/10;
        return distance;
      }
      index=0;
    }
  }
}

float getDistance(){
  sendContinusMeasurementSignal();
  float distance = waitForDistance();
  sendStopMeasurementSignal();
  return distance;
}

void setupMPU() {
  Wire.begin();
  delay(2000);
  mpu.setup(0x68);

  Serial.println("Calibrating MPU9250...");

  // Calibrate the MPU9250
  mpu.calibrateAccelGyro();
  mpu.calibrateMag();

  // Set the magnetic declination for Baghdad
  float magneticDeclination = +4.92;  // Baghdad magnetic declination is +4° 55'

  mpu.setMagneticDeclination(magneticDeclination);
  Serial.println("Calibration complete.");
  delay(3000);
}

float getOffsetedYaw() {
  // Get the yaw angle
  float yaw = mpu.getYaw();
  // Ensure the yaw is within the range [0, 360)
  yaw = fmod(yaw + 360 - 90, 360);
  return yaw;
}

float getCompassHeading(){
  while(true){
    if (mpu.update()) {
      static uint32_t prev_ms = millis();
      if (millis() > prev_ms + 25) {
          float heading = getOffsetedYaw();
          prev_ms = millis();
          return heading;
      }
    }  
  }
}

// Function to calculate new coordinates based on Haversine formula
void calculateNewCoordinates(float currentLat, float currentLng, float compassHeading, float distance, float &newLat, float &newLng) {
    // Earth radius in kilometers
    const double earthRadius = 6371.0;

    // Convert compass heading to radians
    double headingRad = compassHeading * M_PI / 180.0;

    // Convert distance to kilometers
    double distanceKm = distance / 1000.0;

    // Calculate new latitude and longitude
    double latRad = currentLat * M_PI / 180.0;
    double lngRad = currentLng * M_PI / 180.0;

    double newLatRad = asin(sin(latRad) * cos(distanceKm / earthRadius) + cos(latRad) * sin(distanceKm / earthRadius) * cos(headingRad));
    double newLngRad = lngRad + atan2(sin(headingRad) * sin(distanceKm / earthRadius) * cos(latRad), cos(distanceKm / earthRadius) - sin(latRad) * sin(newLatRad));

    // Convert new coordinates back to degrees
    newLat = newLatRad * 180.0 / M_PI;
    newLng = newLngRad * 180.0 / M_PI;
}

void setup()
{
    Serial.begin(SerialMonitorBaud);
    Serial.println("Setting Up IMU and GPS.");         
    ss.begin(GPSBaud); 
    Serial.println("GPS Started.");
    setupMPU();
    SetupLaserSerial();
    laserOn();
}

void loop()
{
  while (ss.available() > 0){
    if (gps.encode(ss.read())){
        if (gps.location.isValid()){
            // IMU Values Read
            float heading = getCompassHeading();

            // GPS reading
            float lat = gps.location.lat();
            float lng = gps.location.lng();
            float nofSat = gps.satellites.value();
            float year = gps.date.year();
            float month = gps.date.month();
            float day = gps.date.day();
            float hour = gps.time.hour() + 3;
            float minute = gps.time.minute();
            float second = gps.time.second();
            float centisecond = gps.time.centisecond();

            // Distance Measurement
            float distance = getDistance();

            float newLat, newLng;
            calculateNewCoordinates(lat, lng, heading, distance, newLat, newLng);
            
            Serial.print("Current Latitude:");
            Serial.println(lat, 6);
            Serial.print("Current Longitude:");
            Serial.println(lng, 6);
            Serial.print("Number of connected Satellites:");
            Serial.println(gps.satellites.value());
            Serial.print("Compass Heading:");
            Serial.print(heading);
            Serial.println("\xC2\xB0");
            Serial.print("Distance:");
            Serial.print(distance);
            Serial.println("m");
            Serial.print("New Latitude: ");
            Serial.println(newLat, 6);
            Serial.print("New Longitude: ");
            Serial.println(newLng, 6);
            Serial.println("*************************************************");
            Serial.println("");
        }
    }
  }

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}
