#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>  // Use the BNO08x Library
#include <AccelStepper.h>
#include <PID_v1.h>

// Pin definitions for ULN2003 and 28BYJ-48
#define LEFT_IN1 4
#define LEFT_IN2 5
#define LEFT_IN3 19
#define LEFT_IN4 20
#define RIGHT_IN1 23
#define RIGHT_IN2 8
#define RIGHT_IN3 10
#define RIGHT_IN4 22

// IMU Constants
#define BNO08X_INT A4
#define BNO08X_RST A5
#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B

// Define stepper objects in FULL4WIRE mode for ULN2003
AccelStepper leftStepper(AccelStepper::FULL4WIRE, LEFT_IN1, LEFT_IN3, LEFT_IN2, LEFT_IN4);
AccelStepper rightStepper(AccelStepper::FULL4WIRE, RIGHT_IN1, RIGHT_IN3, RIGHT_IN2, RIGHT_IN4);

// BNO08x IMU object
BNO08x imu;

// PID variables
double input90, output90, setPoint90;
double kP1 = 500, kI1 = 50, kD1 = 0;
PID turn90PID(&input90, &output90, &setPoint90, kP1, kI1, kD1, DIRECT);

double maxSpeed = 500; // Steps per second
double wheelDiameter = 7.6;
double wheelCircumference = wheelDiameter * PI;

float currentAngle = 0.0;
long startTime;               // Start time of the program
long desiredTotalDuration = 50000;  // Total desired duration in milliseconds (e.g., 20 seconds)


bool isValidYaw(float &yaw) {
  if (imu.getSensorEvent() == true && imu.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
    yaw = imu.getYaw();
    return !isnan(yaw);
  }
  return false;
}

float getCurrentAngle() {
  if (imu.getSensorEvent() == true && imu.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
    float yaw = imu.getYaw();
    if (!isnan(yaw)) {
      currentAngle = yaw;
    }
  }
  return currentAngle;
}

float getCurrentAngleWait() {
  float yaw = currentAngle;
  while (!isValidYaw(yaw)) {}
  currentAngle = yaw;
  return currentAngle;
}

// Helper function to normalize angles to [-π, π)
float normalizeAngleToPi(float angle) {
  while (angle < -PI) {
    angle += 2 * PI;
  }
  while (angle >= PI) {
    angle -= 2 * PI;
  }
  return angle;
}

void rotateToAngle(float targetAngle) {
  float currentAngle = getCurrentAngleWait();
  float angularDifference = normalizeAngleToPi(targetAngle - currentAngle);

  setPoint90 = 0.0;

  Serial.print("CurrentAngle:");
  Serial.print(currentAngle);
  Serial.print(",");
  Serial.print("AngularDifference:");
  Serial.print(angularDifference);
  Serial.println(",");

  turn90PID.SetMode(AUTOMATIC);
  turn90PID.SetOutputLimits(-900, 900);  // Output limits for motor control (steps per second)

  while (abs(angularDifference) > 0.01) {  // Allowable error (tolerance) in radians
    currentAngle = getCurrentAngle();      // Update current angle
    angularDifference = normalizeAngleToPi(targetAngle - currentAngle);

    input90 = angularDifference;  // Update PID input
    turn90PID.Compute();     // Compute PID output

    leftStepper.setSpeed(-output90);
    rightStepper.setSpeed(-output90);

    leftStepper.runSpeed();
    rightStepper.runSpeed();

    Serial.print("currentAngle:");
    Serial.print(currentAngle);
    Serial.print(",AngularDifference:");
    Serial.print(angularDifference);
    Serial.print(",output90:");
    Serial.print(output90);
    Serial.println(",");

    delay(2);  // Small delay for PID stability
  }

  leftStepper.stop();
  rightStepper.stop();
}

void rotateRight() {
  float currentAngle = getCurrentAngleWait();
  float targetAngle = normalizeAngleToPi(currentAngle - PI / 2);  // Rotate 90 degrees right
  rotateToAngle(targetAngle);
}

void rotateLeft() {
  float currentAngle = getCurrentAngleWait();
  float targetAngle = normalizeAngleToPi(currentAngle + PI / 2);  // Rotate 90 degrees left
  rotateToAngle(targetAngle);
}

void rotateToSpecificAngle(float targetAngle) {
  rotateToAngle(normalizeAngleToPi(targetAngle));
}

void rotateToZero() {
  rotateToSpecificAngle(0);
}

void rotateTo90() {
  rotateToSpecificAngle(PI / 2);
}

void rotateTo180() {
  rotateToSpecificAngle(PI);
}

void rotateTo270() {
  rotateToSpecificAngle(-PI / 2);
}

void rotateLeftExact() {
  float currentAngle = getCurrentAngleWait();
  float targetAngle;

  // Determine which of the standard angles (0, 90, 180, 270) the current angle is closest to
  if (currentAngle > -PI / 4 && currentAngle <= PI / 4) {
    targetAngle = PI / 2;  // Closest to 0, turn to 90
  } else if (currentAngle > PI / 4 && currentAngle <= 3 * PI / 4) {
    targetAngle = PI;  // Closest to 90, turn to 180
  } else if (currentAngle > 3 * PI / 4 || currentAngle <= -3 * PI / 4) {
    targetAngle = -PI / 2;  // Closest to 180, turn to 270
  } else {
    targetAngle = 0;  // Closest to 270, turn to 0
  }

  rotateToSpecificAngle(targetAngle);
}

void rotateRightExact() {
  float currentAngle = getCurrentAngleWait();
  float targetAngle;

  // Determine which of the standard angles (0, 90, 180, 270) the current angle is closest to
  if (currentAngle > -PI / 4 && currentAngle <= PI / 4) {
    targetAngle = -PI / 2;  // Closest to 0, turn to 270
  } else if (currentAngle > PI / 4 && currentAngle <= 3 * PI / 4) {
    targetAngle = 0;  // Closest to 90, turn to 0
  } else if (currentAngle > 3 * PI / 4 || currentAngle <= -3 * PI / 4) {
    targetAngle = PI / 2;  // Closest to 180, turn to 90
  } else {
    targetAngle = PI;  // Closest to 270, turn to 180
  }

  Serial.print("CurrentAngle:");
  Serial.print(currentAngle);
  Serial.print(",");
  Serial.print("TargetAngle:");
  Serial.print(targetAngle);
  Serial.println(",");
  rotateToSpecificAngle(targetAngle);
}

void driveForwardRotations(int rotations) {
  int steps = rotations * 2048;  // Calculate total steps for the given number of rotations

  // Reverse the left stepper direction by negating the steps
  leftStepper.move(-steps);
  rightStepper.move(steps);

  // Run the steppers until they reach the target position
  while (leftStepper.distanceToGo() != 0 || rightStepper.distanceToGo() != 0) {
    leftStepper.run();
    rightStepper.run();
  }

  leftStepper.stop();
  rightStepper.stop();
}

void driveForwardTimed(int rotations, long timeToComplete) {
  int steps = rotations * 2048;  // Calculate total steps for the given number of rotations
  float speed = min((float)steps / ((timeToComplete - 800.0) / 1000.0), maxSpeed);  // Steps per second added 800 less milliseconds to account for ramp up and ramp down

  leftStepper.setMaxSpeed(speed);
  rightStepper.setMaxSpeed(speed);

  leftStepper.move(-steps);
  rightStepper.move(steps);

  // Run the steppers until they reach the target position
  while (leftStepper.distanceToGo() != 0 || rightStepper.distanceToGo() != 0) {
    leftStepper.run();
    rightStepper.run();
  }

  leftStepper.stop();
  rightStepper.stop();
}

void driveForwardDistance(int distance) {
  driveForwardRotations(distance / wheelCircumference);
}

void driveForwardDistanceTimed(int distance, long timeToComplete) {
  driveForwardTimed(distance / wheelCircumference, timeToComplete);
}


void setupIMU() {
  Wire.begin();
  Wire.flush();

  if (imu.begin(BNO08X_ADDR, Wire, -1, -1) == false) {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }
  Serial.println("BNO08x found!");

  setReports();
  Serial.println("Reading events");
  delay(100);
}

// Define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (imu.enableGyro() == true) {
    Serial.println("Gyro enabled");
  } else {
    Serial.println("Could not enable gyro");
  }
  if (imu.enableRotationVector() == true) {
    Serial.println("Rotation vector enabled");
    Serial.println("Output in form i, j, k, real, accuracy");
  } else {
    Serial.println("Could not enable rotation vector");
  }
}

void setup() {
  Serial.begin(115200);

  startTime = millis();

  // Initialize steppers
  leftStepper.setMaxSpeed(maxSpeed);
  leftStepper.setAcceleration(500);
  rightStepper.setMaxSpeed(maxSpeed);
  rightStepper.setAcceleration(500);

  setupIMU();

  driveForwardDistance(100);
  rotateLeftExact();
  driveForwardDistance(100);
  rotateLeftExact();
  // driveForwardDistance(100);
  // rotateRightExact();
  // driveForwardDistance(150);
  // rotateRightExact();
  // driveForwardDistance(200);
  // rotateRightExact();
  // driveForwardDistance(100);
  // driveForwardDistance(-100);
  // rotateRightExact();

  // Adjust the final move to fit within the desired total duration
  long elapsedTime = millis() - startTime;
  long remainingTime = desiredTotalDuration - elapsedTime;

  if (remainingTime > 0) {
    Serial.print("Adjusting final move to complete in remaining time: ");
    // Serial.println(remainingTime);
    // driveForwardTimed(1, remainingTime);  // Adjusts the last move
    driveForwardDistanceTimed(100, remainingTime);

  } else {
    // driveForwardRotations(1);
    driveForwardDistance(100);
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

  delay(10);
}