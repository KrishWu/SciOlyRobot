#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>  // Use the BNO08x Library
#include <PID_v1.h>
#include <math.h>
#include <L298N.h>
#include <Encoder.h>

// Pin definitions for ULN2003 and 28BYJ-48
#define LEFT_IN1 4
#define LEFT_IN2 5
#define LEFT_IN3 19
#define LEFT_IN4 20
#define RIGHT_IN1 23
#define RIGHT_IN2 8
#define RIGHT_IN3 10
#define RIGHT_IN4 22

// Left motor
const int IN1 = 0;
const int IN2 = 0;
const int C1A = 0;
const int C2A = 0;

// Right motor
const int IN3 = 0;
const int IN4 = 0;
const int C1B = 0;
const int C2B = 0;

// Define motor objects in FULL4WIRE mode for ULN2003
L298N leftMotor(IN1, IN2);
L298N rightMotor(IN3, IN4);

Encoder leftEncoder(C1A, C2A);
Encoder rightEncoder(C1B, C2B);

// BNO08x IMU object
BNO08x imu;

// Robot parameters
const double maxSpeed = 300; // rpm
const double wheelDiameter = 0.076;
const double wheelCircumference = wheelDiameter * PI;
const double maxSpeedMS = maxSpeed / 60 * wheelCircumference;          // Maximum speed in m/s
const float lookaheadDistance = 0.5; // Lookahead distance in meters
const float wheelBase = 0.2;         // Distance between wheels in meters
const float startX = 0.0;
const float startY = 0.0;

double inputLeft, outputLeft, setPointLeft;
double kPLeft = 1.0, kILeft = 0.1, kDLeft = 0.05;
PID leftMotorPID(&inputLeft, &outputLeft, &setPointLeft, kPLeft, kILeft, kDLeft, DIRECT);

double inputRight, outputRight, setPointRight;
double kPRight = 1.0, kIRight = 0.1, kDRight = 0.05;
PID rightMotorPID(&inputRight, &outputRight, &setPointRight, kPRight, kIRight, kDRight, DIRECT);

long startTime;               // Start time of the program
long desiredTotalDuration = 64000;  // Total desired duration in milliseconds (e.g., 20 seconds)

void setup() {
  Serial.begin(115200);

  setupIMU();

  leftMotorPID.SetMode(AUTOMATIC);
  rightMotorPID.SetMode(AUTOMATIC);

  startTime = millis();

  driveForwardDistance(0.25);
  rotateLeftExact();

  // Adjust the final move to fit within the desired total duration
  long elapsedTime = millis() - startTime;
  long remainingTime = desiredTotalDuration - elapsedTime;

  if (remainingTime > 0) {
    Serial.print("Adjusting final move to complete in remaining time: ");
    Serial.println(remainingTime);
    driveForwardDistanceTimed(0.1, remainingTime);

  } else {
    driveForwardDistance(0.1);
  }
  Serial.print("FinishedTime:");
  Serial.print(millis() - startTime);
  Serial.println(",");
}

void loop() {
  if (imu.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  if (imu.getSensorEvent() == true && imu.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
    float yaw = imu.getYaw();
    Serial.print("Yaw:");
    Serial.print(yaw);
    Serial.print(",");
    Serial.println();
  }
  
  updateRobotState();
  purePursuit();

  // delay(10);
}