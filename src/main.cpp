#include <Arduino.h>
#include <Wire.h>

void setup()
{
  Serial.begin(9600);
}

int16_t b_acc_x, b_acc_y, b_acc_z, b_gyro_x, b_gyro_y, b_gyro_z, b_mag_x, b_mag_y, b_mag_z;
float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z;

float acc_resolution = 2048; // 2048 bits / g.

void loop()
{
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  b_acc_x = Wire.read() << 8 | Wire.read();
  b_acc_y = Wire.read() << 8 | Wire.read();
  b_acc_z = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  b_gyro_x = Wire.read() << 8 | Wire.read();
  b_gyro_y = Wire.read() << 8 | Wire.read();
  b_gyro_z = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  b_mag_x = Wire.read() << 8 | Wire.read();
  b_mag_y = Wire.read() << 8 | Wire.read();
  b_mag_z = Wire.read() << 8 | Wire.read();

  acc_x = b_acc_x / acc_resolution;
  acc_y = b_acc_y / acc_resolution;
  acc_z = b_acc_z / acc_resolution;

  gyro_x = b_gyro_x / acc_resolution;
  gyro_y = b_gyro_y / acc_resolution;
  gyro_z = b_gyro_z / acc_resolution;

  mag_x = b_mag_x / acc_resolution;
  mag_y = b_mag_y / acc_resolution;
  mag_z = b_mag_z / acc_resolution;
}
