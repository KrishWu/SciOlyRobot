#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>  // Use the BNO08x Library
#include <PID_v1.h>
#include <math.h>
#include <L298N.h>
#include <Encoder.h>

// Left motor
const int IN1 = 3;
const int IN2 = 1;
const int ENA = 0;
const int C1A = 22;
const int C2A = 11;

// Right motor
const int IN3 = 4;
const int IN4 = 5;
const int ENB = 19;
const int C1B = 8;
const int C2B = 15;

// Define motor objects in FULL4WIRE mode for ULN2003
L298N leftMotor(ENA, IN1, IN2);
L298N rightMotor(ENB, IN3, IN4);

Encoder leftEncoder(C1A, C2A);
Encoder rightEncoder(C1B, C2B);

// BNO08x IMU object
BNO08x imu;

// Robot parameters
const double maxSpeed = 255.0;  // rpm
const double wheelDiameter = 0.06;
const double wheelCircumference = wheelDiameter * PI;
const double maxSpeedMS = maxSpeed / 60 * wheelCircumference;  // Maximum speed in m/s
const float lookaheadDistance = 0.15;                          // Lookahead distance in meters
const float wheelBase = 0.175;                                 // Distance between wheels in meters
const float startX = 0.0;
const float startY = -0.25;

double inputLeft, outputLeft, setPointLeft;
double kPLeft = 1.0, kILeft = 0.1, kDLeft = 0.05;
PID leftMotorPID(&inputLeft, &outputLeft, &setPointLeft, kPLeft, kILeft, kDLeft, DIRECT);

double inputRight, outputRight, setPointRight;
double kPRight = 1.0, kIRight = 0.1, kDRight = 0.05;
PID rightMotorPID(&inputRight, &outputRight, &setPointRight, kPRight, kIRight, kDRight, DIRECT);

double inputPurePursuit, outputPurePursuit, setPointPurePursuit;
double kPPurePursuit = 500.0, kIPurePursuit = 50.0, kDPurPursuit = 0.0;
PID purePursuitPID(&inputPurePursuit, &outputPurePursuit, &setPointPurePursuit, kPPurePursuit, kIPurePursuit, kDPurPursuit, DIRECT);

long startTime;                     // Start time of the program
float totalDistanceOfPath;
long desiredTotalDuration = 76000;  // Total desired duration in milliseconds (e.g., 20 seconds)

void setup() {
  Serial.begin(115200);

  // Blink the LED to check if setup() is running
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }

  Serial.println("Setup starting");

  setupIMU();

  leftMotorPID.SetMode(AUTOMATIC);
  rightMotorPID.SetMode(AUTOMATIC);

  startTime = millis();
  totalDistanceOfPath = calculateDistanceRemaining();

  purePursuitPID.SetMode(AUTOMATIC);
  purePursuitPID.SetOutputLimits(30, 255);
  setPointPurePursuit = totalDistanceOfPath / ((desiredTotalDuration - (millis() - startTime)) / 1000.0);

  // driveForwardDistance(wheelCircumference);
  // rotateLeftExact();

  // Adjust the final move to fit within the desired total duration
  // long elapsedTime = millis() - startTime;
  // long remainingTime = desiredTotalDuration - elapsedTime;

  // if (remainingTime > 0) {
  // Serial.print("Adjusting final move to complete in remaining time: ");
  // Serial.println(remainingTime);
  // driveForwardDistanceTimed(0.1, remainingTime);

  // } else {
  // driveForwardDistance(0.1);
  // }
  // Serial.print("FinishedTime:");
  // Serial.print(millis() - startTime);
  Serial.println(",");

  Serial.println("Finished setup");
}

void loop() {
  if (imu.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  // if (imu.getSensorEvent() == true && imu.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
  //   float yaw = imu.getYaw();
  //   Serial.print("Yaw:");
  //   Serial.print(yaw);
  //   Serial.print(",");
  //   Serial.println();
  // }

  updateRobotState();
  purePursuit();

  delay(10);
}