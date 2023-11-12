#include <MPU9250.h>
#include <Wire.h>
#include <math.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#define ss Serial2
#define laserSerial Serial3

// Baud Rate Setup
static const uint32_t SerialMonitorBaud = 115200;
static const uint32_t GPSBaud = 9600;
static const uint32_t laserBaud = 9600;

// Devices Objects
TinyGPSPlus gps;
MPU9250 IMU (Wire , 0x68);

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

// Useful Methods
float magnetometerToCompass(float mx, float my, float mz, float declination) {
    float heading;
    // Calculate the magnitude of the magnetic field vector
    float magnitude = sqrt(mx * mx + my * my + mz * mz);
    // Calculate the pitch angle in degrees
    float pitch = asin(-mz / magnitude) * 180.0 / M_PI;
    // Calculate the raw yaw angle in degrees
    float rawYaw = atan2(my, -mx) * 180.0 / M_PI;
    // Apply the magnetic declination angle to correct for the difference between magnetic north and true north
    float yaw = rawYaw + declination;
    // Convert the yaw angle to the range of 0 to 360 degrees
    if (yaw < 0) {
        yaw += 360;
    } else if (yaw >= 360) {
        yaw -= 360;
    }
    // Calculate the final heading by combining the pitch and yaw angles
    heading = atan2(sin(yaw * M_PI / 180.0), cos(yaw * M_PI / 180.0) * cos(pitch * M_PI / 180.0)) * 180.0 / M_PI;
    // Convert the heading to the range of 0 to 360 degrees
    if (heading < 0) {
        heading += 360;
    } else if (heading >= 360) {
        heading -= 360;
    }
    return heading;
}

float calculateMagneticDeclination(float lon, float lat, float mx, float my, float mz) {
    double latRad = radians(lat);
    double lonRad = radians(lon);
    
    double Bx = mx * cos(latRad) + mz * sin(latRad);
    double By = mx * sin(lonRad) * sin(latRad) + my * cos(lonRad) - mz * sin(lonRad) * cos(latRad);
    
    double magneticDeclination = atan2(-By, -Bx) * 180 / PI;
    if (magneticDeclination < 0) {
      magneticDeclination += 360.0;
    }
    return magneticDeclination;
}


void setup()
{
    Serial.begin(SerialMonitorBaud);
    Serial.println("Setting Up IMU and GPS.");         
    IMU.begin();
    ss.begin(GPSBaud); 
    Serial.println("IMU and GPS Started.");
    SetupLaserSerial();
    laserOn();
}

void loop()
{
  while (ss.available() > 0){
    if (gps.encode(ss.read())){
        if (gps.location.isValid()){
            // IMU Values Read
            IMU.readSensor();

            // Accelerometer data
            float ax = IMU.getAccelX_mss();
            float ay = IMU.getAccelY_mss();
            float az = IMU.getAccelZ_mss();

            // Gyroscope data
            float gx = IMU.getGyroX_rads();
            float gy = IMU.getGyroY_rads();
            float gz = IMU.getGyroZ_rads();

            // Magnetometer data
            float mx = IMU.getMagX_uT();
            float my = IMU.getMagY_uT();
            float mz = IMU.getMagZ_uT();

            // Temperature reading
            float temperature = IMU.getTemperature_C();

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
            float magneticDeclination = calculateMagneticDeclination(lng, lat, mx, my, mz);
            float heading = magnetometerToCompass(mx, my, mz, magneticDeclination);

            // Distance Measurement
            float distance = getDistance();
            
            Serial.print("lng:");
            Serial.println(lng, 6);
            Serial.print("lat:");
            Serial.println(lat, 6);
            Serial.print("Number of connected Satellites:");
            Serial.println(gps.satellites.value());
            Serial.print("MX:");
            Serial.println(mx);
            Serial.print("MY:");
            Serial.println(my);
            Serial.print("MZ:");
            Serial.println(mz);
            Serial.print("Magnetic Declination:");
            Serial.print(magneticDeclination);
            Serial.println("\xC2\xB0");
            Serial.print("Compass Heading:");
            Serial.print(heading);
            Serial.println("\xC2\xB0");
            Serial.print("Distance:");
            Serial.print(distance);
            Serial.println("m");
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
