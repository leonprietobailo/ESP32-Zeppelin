#include <WiFi.h>
#include <Wire.h>
#include <MPU6500_WE.h>
#include <esp_task_wdt.h>
#include <QMC5883LCompass.h>
// #include <Fusion.h>

#define SAMPLE_RATE = 100.0; // Hz
uint64_t startGyro;

// HMC5883 Initialization.
QMC5883LCompass mag;

// MPU6500 Initialization.
#define MPU6500_ADDR 0x68
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);
uint64_t executionTime;
xyzFloat acc, vel, pos, g, gyro;

// WiFi connection.
const char *ssid = "POCO X3 NFC";
const char *password = "omegaisugly";
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

// const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
// const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
// const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
// const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
// const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
// const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
// const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
// const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

// // Initialise algorithms
// FusionOffset offset;
// FusionAhrs ahrs;

// const FusionAhrsSettings settings = {
//     .convention = FusionConventionNwu,
//     .gain = 0.5f,
//     .gyroscopeRange = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
//     .accelerationRejection = 10.0f,
//     .magneticRejection = 10.0f,
//     .recoveryTriggerPeriod = 5 * 250, /* 5 seconds */
// };

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
    Serial.print("Connecting to WiFi... Status: ");
    Serial.println(WiFi.status()); // This will print the status code
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("Connected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("Failed to connect to WiFi. Check your credentials or hardware.");
  }

  // Start the server
  server.begin();
  Serial.println("Server started");
  Serial.println(WiFi.localIP());

  // Create tasks
  xTaskCreatePinnedToCore(
      TaskServer,   /* Function to implement the task */
      "TaskServer", /* Name of the task */
      10000,        /* Stack size in words */
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

  // FusionOffsetInitialise(&offset, 100);
  // FusionAhrsInitialise(&ahrs);
  // FusionAhrsSetSettings(&ahrs, &settings);

  startGyro = micros();
}

void loop()
{
  ReadUnits();
  Controllers();
  Actuators();
  // Diagnostics();
  Serial.println(micros() - executionTime);
  if (micros() - executionTime > 1.0 / 100)
  {
    Serial.println("WARNING: Long loop. PIDs may get unstable. " + micros() - executionTime);
  }
  while (micros() - executionTime < 1 / 100)
  {
  }
  executionTime = micros();
}
int n = 0;
xyzFloat g_avg;

void ReadUnits()
{
  // HMC5883 READOUTS
  mag.read();

  // MPU6500 READOUTS
  // acc = myMPU6500.getCorrectedAccRawValues();
  xyzFloat g_read = myMPU6500.getGValues();
  g_avg += g_read;
  g.x = g.x * 0.95 + g_read.x * 0.05;
  g.y = g.y * 0.95 + g_read.y * 0.05;
  g.z = g.z * 0.95 + g_read.z * 0.05;
  gyro = myMPU6500.getGyrValues();

  // Velocity
  // vel += g * 9.81 * 4000e-6;
  // pos += vel * 4000e-6;

  // const clock_t timestamp = (micros() - startGyro) / 1e6;
  // FusionVector gyroscope = {gyro.x, gyro.y, gyro.z};
  // FusionVector accelerometer = {g_read.x, g_read.y, g_read.z};

  // gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
  // accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);

  // gyroscope = FusionOffsetUpdate(&offset, gyroscope);

  // static clock_t previousTimestamp;
  // const float deltaTime = (float)(timestamp - previousTimestamp) / (float)CLOCKS_PER_SEC;
  // previousTimestamp = timestamp;

  // FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);

  // const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
  // const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

  // Serial.printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f\n",
  //               euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
  //               earth.axis.x, earth.axis.y, earth.axis.z);

  // if (n == 10)
  // {
  //   vel += g_avg * 9.81 * n * 4000e-6 / n;
  //   pos += vel * 4000e-6 + g_avg * 9.81 * n * 4000e-6 * 0.5 * n * 4000e-6;
  //   n = 0;
  // }
  // else
  // {
  //   n++;
  // }

  // Serial.print("gx: ");
  // Serial.print(g.x);
  // Serial.print("\t");

  // Serial.print("gy: ");
  // Serial.print(g.y);
  // Serial.print("\t");

  // Serial.print("gz: ");
  // Serial.print(g.z);

  // Serial.print("x: ");
  // Serial.print(pos.x);
  // Serial.print("\t");

  // Serial.print("y: ");
  // Serial.print(pos.y);
  // Serial.print("\t");

  // Serial.print("z: ");
  // Serial.println(pos.z);
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