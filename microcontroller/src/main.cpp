#include <WiFi.h>
#include <Wire.h>
#include <MPU6500_WE.h>
#include <esp_task_wdt.h>
#include <QMC5883LCompass.h>

// HMC5883 Initialization.
QMC5883LCompass mag;

// MPU6500 Initialization.
#define MPU6500_ADDR 0x68
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);
uint64_t executionTime;
xyzFloat g, gyro;

// WiFi connection.
const char *ssid = "XTA.CAT-7E5AA4";
const char *password = "ZqqKpgwn";
WiFiServer server(80);

// Function declarations
void TaskServer(void *pvParameters);
void ReadUnits();
void Controllers();
void Actuators();
void Diagnostics();

enum FlightMode
{
  FM_DISABLED,
  FM_DEFAULT,
  FM_DEAD_RECKONING,
  FM_TEST_MOTORS
};

FlightMode FLIGHT_MODE = FM_DISABLED;

void setup()
{
  Wire.begin();
  // Disable WDT.
  disableCore0WDT();
  disableCore1WDT();

  // Serial com.
  Serial.begin(115200);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  // Start the server
  server.begin();
  Serial.println("Server started");
  Serial.println(WiFi.localIP());

  // Create tasks
  xTaskCreatePinnedToCore(
      TaskServer,   /* Function to implement the task */
      "TaskServer", /* Name of the task */
      100000,       /* Stack size in words */
      NULL,         /* Task input parameter */
      1,            /* Priority of the task */
      NULL,         /* Task handle. */
      0);           /* Core where the task should run */

  // HMC5883 Setup
  mag.init();
  mag.setCalibrationOffsets(-345.00, 252.00, -1223.00);
  mag.setCalibrationScales(1.10, 0.96, 0.95);

  // MPU6500 Setup
  if (!myMPU6500.init())
  {
    Serial.println("MPU6500 does not respond");
  }
  else
  {
    Serial.println("MPU6500 is connected");
  }
  Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
  delay(1000);
  myMPU6500.autoOffsets();
  Serial.println("Done!");
  myMPU6500.enableGyrDLPF();
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
  myMPU6500.setSampleRateDivider(5);
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G);
  myMPU6500.enableAccDLPF(true);
  myMPU6500.setAccDLPF(MPU6500_DLPF_6);
  delay(200);
}

void loop()
{
  ReadUnits();
  Controllers();
  Actuators();
  Diagnostics();

  if (micros() - executionTime > 4000)
  {
    Serial.println("WARNING: Long loop. PIDs may get unstable.");
  }
  while (micros() - executionTime < 4000)
  {
  }
  executionTime = micros();
}

void ReadUnits()
{
  // HMC5883 READOUTS
  mag.read();

  // MPU6500 READOUTS
  g = myMPU6500.getGValues();
  gyro = myMPU6500.getGyrValues();
}
void Controllers()
{
  // INCLUDE PID CONTROLLERS CODE.
}
void Actuators()
{
  switch (FLIGHT_MODE)
  {
  case FM_DISABLED:
    /* code */
    break;

  case FM_DEFAULT:
    /* code */
    break;

  case FM_DEAD_RECKONING:
    /* code */
    break;

  case FM_TEST_MOTORS:
    /* code */
    break;

  default:
    break;
  }
}
void Diagnostics()
{
  int x = mag.getX();
  int y = mag.getY();
  int z = mag.getZ();

  float heading = atan2(y, x) * 180.0 / PI;
  if (heading < 0)
  {
    heading += 360;
  }
  Serial.print("x: ");
  Serial.print(x);
  Serial.print("\t");

  Serial.print("y: ");
  Serial.print(y);
  Serial.print("\t");

  Serial.print("z: ");
  Serial.print(z);
  Serial.print("\t");

  Serial.print("A: ");
  Serial.println(heading);
}

void TaskServer(void *pvParameters)
{
  (void)pvParameters;

  for (;;)
  {
    // Check if a client has connected
    WiFiClient client = server.available();
    if (client)
    {
      Serial.println("New Client.");
      // Wait until the client sends some data
      while (!client.available())
      {
        delay(1);
      }

      // Read the first line of the request
      String request = client.readStringUntil('\r');
      Serial.println(request);
      client.flush();

      // Match the request
      if (request.indexOf("/CMD1") != -1)
      {
        Serial.println("Received unassigned CMD1");
      }
      else if (request.indexOf("/CMD2") != -1)
      {
        Serial.println("Received unassigned CMD2");
      }
      else if (request.indexOf("/CMD3") != -1)
      {
        Serial.println("Received unassigned CMD3");
      }
      else if (request.indexOf("/CMD4") != -1)
      {
        Serial.println("Received unassigned CMD4");
      }
      else if (request.indexOf("/CMD5") != -1)
      {
        Serial.println("Received unassigned CMD5");
      }

      client.println("HTTP/1.1 200 OK");
      client.println("Content-type:text/html");
      client.println();
      client.println("Command received");
      client.println();

      client.stop();
      Serial.println("Client disconnected");
    }
  }
}