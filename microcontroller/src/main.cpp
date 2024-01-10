#include <WiFi.h>
#include <Wire.h>
#include <MPU6500_WE.h>
#include <esp_task_wdt.h>
#include <QMC5883LCompass.h>
#include <Fusion.h>
#include <soc/rtc_wdt.h>
#include <PID_v1.h>

#define SAMPLE_RATE = 100.0; // Hz
uint64_t startGyro;

// QMC5883 Initialization.
QMC5883LCompass mag;

// MPU6500 Initialization.
#define MPU6500_ADDR 0x68
MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);
uint64_t executionTime;
xyzFloat acc, vel, pos, g, gyro;

// Define the GPIO pins for the PWM signals
const int pwmPin1_CW = 0; // G0: Left
const int pwmPin1_CCW = 4;

const int pwmPin2_CW = 16; // G4: Right
const int pwmPin2_CCW = 17;

// Define PWM channel, frequency, and resolution
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int freq = 10000;   // Frequency in Hz
const int resolution = 8; // Resolution in bits (8 bits -> 0-255 value range)

// Define Duty Cicles for motor control.
float rightMotorDutyCicle, leftMotorDutyCicle;

// PID Initialization
double Setpoint, Input, Output;
double Kp = 0.09, Ki = 0.01, Kd = 0.25;
float pid_out_left, pid_out_right;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int pidMin = -50, pidMax = 50;

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
double unwrap(double previousAngle, double newAngle);
void changeMotorSpeed(int leftDutyCicle, int rightDutyCicle);
void resetIntegrator();

enum FlightMode
{
  FM_DISABLED,
  FM_HOLD_HEADING,
  FM_FLIGHTPLAN,
  FM_TEST_MOTORS
};

FlightMode FLIGHT_MODE = FM_DISABLED;

const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

// Initialise algorithms
FusionOffset offset;
FusionAhrs ahrs;

// Misc
int n = 0;
xyzFloat g_avg;
float vel_x, vel_y, vel_z, pos_x, pos_y, pos_z, roll, pitch;
double yaw;
u64_t flightplanStart;

const FusionAhrsSettings settings = {
    .convention = FusionConventionNwu,
    .gain = 0.5f,
    .gyroscopeRange = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
    .accelerationRejection = 10.0f,
    .magneticRejection = 10.0f,
    .recoveryTriggerPeriod = 5 * 250, /* 5 seconds */
};

xyzFloat gyroReads;

void setup()
{
  /** PWMs*/
  // Grounded H-Bridge PINs
  pinMode(pwmPin1_CCW, OUTPUT);
  pinMode(pwmPin2_CCW, OUTPUT);
  digitalWrite(pwmPin1_CCW, LOW);
  digitalWrite(pwmPin2_CCW, LOW);

  // PWM H-Bridge PINs
  ledcSetup(pwmChannel1, freq, resolution);
  ledcSetup(pwmChannel2, freq, resolution);
  ledcAttachPin(pwmPin1_CW, pwmChannel1);
  ledcAttachPin(pwmPin2_CW, pwmChannel2);
  ledcWrite(pwmChannel1, 0);
  ledcWrite(pwmChannel2, 0);

  // PIDs
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(pidMin, pidMax);
  Setpoint = 0;
  Input = 0;

  Wire.begin();
  // Disable WDT.
  disableCore0WDT();
  disableCore1WDT();
  rtc_wdt_protect_off();
  rtc_wdt_disable();

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

  FusionOffsetInitialise(&offset, 100);
  FusionAhrsInitialise(&ahrs);
  FusionAhrsSetSettings(&ahrs, &settings);

  startGyro = micros();

  for (int i = 0; i < 4000; i++)
  {
    gyroReads += myMPU6500.getGyrRawValues() / 131 / 4000;
  }
  Serial.println("Gyro Meas");
  Serial.println(myMPU6500.getGyrRawValues().x / 131);
  Serial.println(gyroReads.x);
}

void loop()
{
  ReadUnits();
  Controllers();
  Actuators();
  // Diagnostics();
  // Serial.println(micros() - executionTime);
  if (micros() - executionTime > 1e4)
  {
    Serial.println("WARNING: Long loop. PIDs may get unstable. " + micros() - executionTime);
  }
  while (micros() - executionTime < 1e4)
  {
  }
  executionTime = micros();
}

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
  gyro = myMPU6500.getGyrValues(); // / 131 - gyroReads;

  // Velocity
  // vel += g * 9.81 * 4000e-6;
  // pos += vel * 4000e-6;

  const clock_t timestamp = clock();
  FusionVector gyroscope = {gyro.x, gyro.y, gyro.z};
  FusionVector accelerometer = {g_read.x, g_read.y, g_read.z};

  gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
  accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);

  gyroscope = FusionOffsetUpdate(&offset, gyroscope);

  static clock_t previousTimestamp;
  const float deltaTime = (float)(timestamp - previousTimestamp) / (float)CLOCKS_PER_SEC;
  previousTimestamp = timestamp;

  FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);

  const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
  const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);

  // Serial.printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f, X %0.1f, Y %0.1f, Z %0.1f\n",
  //               euler.angle.roll, euler.angle.pitch, euler.angle.yaw,
  //               earth.axis.x, earth.axis.y, earth.axis.z);
  roll = euler.angle.roll;
  pitch = euler.angle.pitch;
  yaw = unwrap(yaw, euler.angle.yaw);

  vel_x += earth.axis.x * deltaTime;
  pos_x += vel_x * deltaTime + 0.5 * earth.axis.x * deltaTime * deltaTime;
  // Serial.println(pos_x);

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
  Input = yaw;
  bool result = myPID.Compute();

  double outputCapped = Output / 3.0;

  Serial.printf("Input: %f    Setpoint: %f    Output: %f    Yaw: %f    Boolean: %d", Input, Setpoint, Output, yaw, result);
  Serial.println();
}

void Actuators()
{
  switch (FLIGHT_MODE)
  {
  case FM_DISABLED:
  {
    rightMotorDutyCicle = 0;
    leftMotorDutyCicle = 0;
  }
  break;

  case FM_HOLD_HEADING:
  {
    Setpoint = 0;
    float throttle = 50;
    rightMotorDutyCicle = 1.02 * throttle - Output;
    leftMotorDutyCicle = 0.98 * throttle + Output;
  }
  break;

  case FM_FLIGHTPLAN:
  {
    if (millis() - flightplanStart < 5000)
    {
      Setpoint = 90;
    }
    // else if (millis() - flightplanStart < 10000)
    // {
    //   Setpoint = 90;
    // }
    // else if (millis() - flightplanStart < 15000)
    // {
    //   Setpoint = 180;
    // }
    // else if (millis() - flightplanStart < 20000)
    // {
    //   Setpoint = 90;
    // }
    // else if (millis() - flightplanStart < 25000)
    // {
    //   Setpoint = 0;
    //   FLIGHT_MODE = FM_DISABLED;
    // }
    float throttle = 50;
    rightMotorDutyCicle = 1.02 * throttle - Output;
    leftMotorDutyCicle = 0.98 * throttle + Output;
  }

  case FM_TEST_MOTORS:
  {
    rightMotorDutyCicle = 100;
    leftMotorDutyCicle = 100;
  }
  break;

  default:
    break;
  }

  changeMotorSpeed(leftMotorDutyCicle, rightMotorDutyCicle);

  // Serial.printf("Left Motor: %f    Right Motor: %f Map1: %f Map2: %f", leftMotor, rightMotor, leftMotor / 2.55, rightMotor / 2.55);

  // Serial.println();
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

      // Check if the client is still connected
      if (client.connected())
      {
        // Read the first line of the request
        String request = client.readStringUntil('\r');
        Serial.println(request);

        // Start of the response
        client.println("HTTP/1.1 200 OK");
        client.println("Content-type:text/html");
        client.println();

        // Match the request and send a response
        if (request.indexOf("/CMD1") != -1)
        {
          FLIGHT_MODE = FM_DISABLED;
          client.println("Switched to: \"FM_DISABLED\"");
        }
        else if (request.indexOf("/CMD2") != -1)
        {
          FLIGHT_MODE = FM_HOLD_HEADING;
          client.println("Switched to: \"FM_HOLD_HEADING\"");
          resetIntegrator();
        }
        else if (request.indexOf("/CMD3") != -1)
        {
          FLIGHT_MODE = FM_FLIGHTPLAN;
          flightplanStart = millis();
          client.println("Switched to: \"FM_FLIGHTPLAN\"");
        }
        else if (request.indexOf("/CMD4") != -1)
        {
          FLIGHT_MODE = FM_TEST_MOTORS;
          client.println("Switched to: \"FM_TEST_MOTORS\"");
        }
        else if (request.indexOf("/CMD5") != -1)
        {
          client.printf("%0.1f,%0.1f,%0.1f", roll, pitch, yaw);
        }
        else
        {
          client.println("Unknown Command");
        }

        client.println(); // End of the response

        // Flush and stop client after sending the response
        client.flush();
        client.stop();
        Serial.println("Client disconnected");
      }
      else
      {
        Serial.println("Client disconnected before request could be read.");
      }
    }
  }
}

double unwrap(double previousAngle, double newAngle)
{
  double delta = newAngle - previousAngle;
  if (delta > 180)
    delta -= 360;
  else if (delta < -180)
    delta += 360;
  return previousAngle + delta;
}

void changeMotorSpeed(int leftDutyCicle, int rightDutyCicle)
{
  // rightDutyCicle *= 1.05;
  // leftDutyCicle *= 0.95;
  if (leftDutyCicle > 100)
  {
    leftDutyCicle = 100;
  }
  else if (leftDutyCicle < 0)
  {
    leftDutyCicle = 0;
  }

  if (rightDutyCicle > 100)
  {
    rightDutyCicle = 100;
  }
  else if (rightDutyCicle < 0)
  {
    rightDutyCicle = 0;
  }

  ledcWrite(pwmChannel1, rightDutyCicle);
  ledcWrite(pwmChannel2, leftDutyCicle);
}

void resetIntegrator()
{
  myPID.SetOutputLimits(0.0, 1.0);       // Forces minimum up to 0.0
  myPID.SetOutputLimits(-1.0, 0.0);      // Forces maximum down to 0.0
  myPID.SetOutputLimits(pidMin, pidMax); // Set the limits back to normal
}