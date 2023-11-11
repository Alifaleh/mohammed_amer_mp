#include <MPU9250.h>
#include <Wire.h>
#include <math.h>

MPU9250 IMU (Wire , 0x68);



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


void setup() {                  
  Serial.begin(115200);            
  IMU.begin();                  
}

void loop() {
  IMU.readSensor();
  //Accelerometer data code
  Serial.print("Accelerometer X axis: ");   
  Serial.print(IMU.getAccelX_mss(), 3);     // to get the accelerometer value from the data buffer in the X direction, these values are in meter per second square
  Serial.print("               Accelerometer Y axis: ");
  Serial.print(IMU.getAccelY_mss(), 3);
  Serial.print("               Accelerometer Z axis: ");
  Serial.println(IMU.getAccelZ_mss(), 3);
  //Gyroscope data code
  Serial.print("Gyroscope X axis(radians): ");
  Serial.print(IMU.getGyroX_rads(), 3);        // gets the gyroscope value from the data buffer in the X direction, these vavlues are in radians per second
  Serial.print("           Gyroscope Y axis(radians): ");
  Serial.print(IMU.getGyroY_rads(), 3);
  Serial.print("           Gyroscope Z axis(radians): ");
  Serial.println(IMU.getGyroZ_rads(), 3);
  //Magnetometer data code
  float mx = IMU.getMagX_uT();
  float my = IMU.getMagY_uT();
  float mz = IMU.getMagZ_uT();
  float declination = 4.8833;
  Serial.print("Magnetometer X axis(MicroTesla): ");
  Serial.print(mx, 3);                //gets the magnetometer value from the data buffer in the X direction, these are in microtesla
  Serial.print("    Magnetometer Y axis(MicroTesla): ");
  Serial.print(my, 3);
  Serial.print("    Magnetometer Z axis(MicroTesla): ");
  Serial.println(mz, 3);
  //Temperature reading
  Serial.print("Temperature: ");
  Serial.println(IMU.getTemperature_C(), 2);         // gets the temperature value from the data buffer and returns it in units of C
  Serial.print("Compass Head: ");
  Serial.print(magnetometerToCompass(mx, my, mz, declination), 2);
  Serial.print("\xC2\xB0");
  Serial.println();
  Serial.print("*********** Next buffer data *****************");
  Serial.println("    ");
  Serial.println("    ");
  Serial.println("    ");
  Serial.println("    ");
  delay(2000);
}
