#define laserSerial Serial3
unsigned char buf[50]; 
int index = 0; 

void SetupLaserSerial(){
  laserSerial.begin(9600);
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

void setup() {
  Serial.begin(19200);
  SetupLaserSerial();
  laserOn();
}



void loop() {
  float distance = getDistance();
  Serial.print(distance);
  Serial.println("m");
  delay(1000);
}
