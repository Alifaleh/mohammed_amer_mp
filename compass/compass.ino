#include "MPU9250.h"

MPU9250 mpu;

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


void setup() {
  Serial.begin(115200);
  setupMPU();
}

void loop() {
  float heading = getCompassHeading();
  Serial.print("Compass Heading: ");
  Serial.print(heading, 2);
  Serial.println(" degrees");
}
