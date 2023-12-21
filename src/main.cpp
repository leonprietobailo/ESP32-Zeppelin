#include <Arduino.h>
#include <Wire.h>

// IMU VARIABLES
int16_t b_acc_x, b_acc_y, b_acc_z, b_gyro_x, b_gyro_y, b_gyro_z, b_mag_x, b_mag_y, b_mag_z;
float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z;
float acc_resolution = 2048; // 2048 bits / g.

void setup()
{
  Serial.begin(9600);

  callibrateImu();
}

void loop()
{
  readImu();
  printData();
}

float acc_x_cal, acc_y_cal, acc_z_cal, gyro_x_cal, gyro_y_cal, gyro_z_cal, mag_x_cal, mag_y_cal, mag_z_cal;
bool imuCallibrated = false;

void callibrateImu()
{
  for (u_int8_t i = 0; i < 4000; i++)
  {
    readImu();

    acc_x_cal += acc_x / 4000;
    acc_y_cal += acc_y / 4000;
    acc_z_cal += acc_z / 4000;

    gyro_x_cal += gyro_x / 4000;
    gyro_y_cal += gyro_y / 4000;
    gyro_z_cal += gyro_z / 4000;

    mag_x_cal += mag_x / 4000;
    mag_y_cal += mag_y / 4000;
    mag_z_cal += mag_z / 4000;
  }
  imuCallibrated = true;
}

void readImu()
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

void printData()
{
  Serial.print("ACC_X: ");
  Serial.print(acc_x);
  Serial.print("\tACC_Y: ");
  Serial.print(acc_y);
  Serial.print("\tACC_Z: ");
  Serial.print(acc_z);

  Serial.print("\tGYRO_X: ");
  Serial.print(gyro_x);
  Serial.print("\tGYRO_Y: ");
  Serial.print(gyro_y);
  Serial.print("\tGYRO_Z: ");
  Serial.print(gyro_z);

  Serial.print("\tMAG_X: ");
  Serial.print(mag_x);
  Serial.print("\tMAG_Y: ");
  Serial.print(mag_y);
  Serial.print("\tMAG_Z: ");
  Serial.println(mag_z);
}
