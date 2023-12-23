#include <WiFi.h>
#include <Wire.h>
#include <MPU6500_WE.h>
#include <esp_task_wdt.h>

// MPU6500 Initialization.
#define MPU6500_ADDR 0x68
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);
uint64_t executionTime;

// WiFi connection.
const char *ssid = "XTA.CAT-7E5AA4";
const char *password = "ZqqKpgwn";
WiFiServer server(80);

// Function declarations
void TaskServer(void *pvParameters);

void setup()
{
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

  // MPU6500 SETUP
  Wire.begin();
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
  xyzFloat gValue = myMPU6500.getGValues();
  xyzFloat gyr = myMPU6500.getGyrValues();
  float resultantG = myMPU6500.getResultantG(gValue);

  if (micros() - executionTime > 4000)
  {
    Serial.println("WARNING: Long loop. PIDs may get unstable.");
  }
  while (micros() - executionTime < 4000)
  {
  }
  executionTime = micros();
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
      else if (request.indexOf("/CMD2") != -1)
      {
        Serial.println("Received unassigned CMD3");
      }
      else if (request.indexOf("/CMD2") != -1)
      {
        Serial.println("Received unassigned CMD4");
      }
      else if (request.indexOf("/CMD2") != -1)
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
