#include <Wire.h>
uint8_t IMUAddress = 0x68;

int var;
int16_t accX;
int16_t accY;
int16_t accZ;
int16_t tempRaw;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;

unsigned long isRed = 0;
byte pwmRed = 0;
byte pwmGreen = 255;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  i2cWrite(0x6B,0x00); // Disable sleep mode 
  var = 0;
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  analogWrite(3, pwmGreen);
  analogWrite(5, pwmRed);
}
void loop() {
  while ( var < 100 ){
    uint8_t* data = i2cRead(0x3B,14);
    accX = ((data[0] << 8) | data[1]);
    accY = ((data[2] << 8) | data[3]);
    accZ = ((data[4] << 8) | data[5]);
    tempRaw = ((data[6] << 8) | data[7]);
    gyroX = ((data[8] << 8) | data[9]);
    gyroY = ((data[10] << 8) | data[11]);
    gyroZ = ((data[12] << 8) | data[13]);
    var++;
  }
  uint8_t* data = i2cRead(0x3B,14);
  accX = ((data[0] << 8) | data[1]);
  accY = ((data[2] << 8) | data[3]);
  accZ = ((data[4] << 8) | data[5]);
  tempRaw = ((data[6] << 8) | data[7]);
  gyroX = ((data[8] << 8) | data[9]);
  gyroY = ((data[10] << 8) | data[11]);
  gyroZ = ((data[12] << 8) | data[13]);
  
  double gyroXrate = (double)gyroX/131.0;
  double gyroYrate = (double)gyroY/131.0;
  double gyroZrate = (double)gyroZ/131.0;
  
  int result = abs(gyroXrate) + abs(gyroYrate) + abs(gyroZrate);
  
  if (result > 700) {
    isRed = millis();
    pwmRed = 255;
    pwmGreen = 0;
    analogWrite(3, pwmGreen);
    analogWrite(5, pwmRed);
  }

  if ( pwmRed != 0 && millis() > isRed + 50 ) {
    isRed = millis();
    analogWrite(3, min(++pwmGreen, 255));
    analogWrite(5, max(--pwmRed, 0));
    Serial.println(pwmRed);
  }
}

void i2cWrite(uint8_t registerAddress, uint8_t data){
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission(); // Send stop
}
uint8_t* i2cRead(uint8_t registerAddress, uint8_t nbytes) {
  uint8_t data[nbytes];
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false); // Don't release the bus
  Wire.requestFrom(IMUAddress, nbytes); // Send a repeated start and then release the bus after reading
  for(uint8_t i = 0; i < nbytes; i++)
    data [i]= Wire.read();
  return data;
}
