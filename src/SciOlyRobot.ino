// put your setup code here, to run once:
#include <Wire.h>
#include <ESP32Servo.h>                       // Use ESP32Servo instead of the default Servo library
#include <SparkFun_BNO08x_Arduino_Library.h>  // Use the BNO08x Library
#include <NewPing.h>                          // Include the NewPing library
#include <PID_v1.h>

// Pin definitions
const int ULTRASONIC_TRIG_PIN = 6;   // Trigger pin of the ultrasonic sensor
const int ULTRASONIC_ECHO_PIN = 18;  // Echo pin of the ultrasonic sensor
const int LEFT_SERVO_PIN = 2;        // Pin for left servo
const int RIGHT_SERVO_PIN = 21;      // Pin for right servo

// Constants
const int maxDistance = 30;         // Max distance in cm to stop the robot
const int maxSensorDistance = 200;  // Maximum sensor distance for NewPing (200 cm)
float speedCmPerSec = 27;

//IMU Constants
#define BNO08X_INT A4
#define BNO08X_RST A5
#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B

// Servo objects
Servo leftServo;
Servo rightServo;

float currentAngle = 0.0;

// NewPing object for ultrasonic sensor
NewPing sonar(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN, maxSensorDistance);

// BNO08x IMU object
BNO08x imu;  // Create an instance of the BNO08X IMU

double input90, output90, setPoint90;

double kP1 = 10, kI1 = 0, kD1 = 0;
PID turn90PID(&input90, &output90, &setPoint90, kP1, kI1, kD1, DIRECT);

double inputForward, outputForward, setPointForward;
double kP2 = 0.001, kI2 = 0, kD2 = 0;
PID forwardPID(&inputForward, &outputForward, &setPointForward, kP2, kI2, kD2, DIRECT);

// Function to move the robot forward
void moveForward() {
  leftServo.write(180);  // Full speed forward
  rightServo.write(0);   // Full speed forward
}

// Function to stop the robot
void stopRobot() {
  leftServo.write(90);   // Stop
  rightServo.write(90);  // Stop
}

void moveForwardDistance(float targetDistanceCm) {
  // Set up the target distance in centimeters
  setPointForward = targetDistanceCm;

  // Initialize PID
  forwardPID.SetMode(AUTOMATIC);
  forwardPID.SetOutputLimits(-90, 90);  // Output limits for motor control

  // Reset distance measurement variables
  unsigned int initialDistance = sonar.ping_cm();
  if (initialDistance == 0) {
    Serial.println("Ultrasonic sensor failed to get initial distance.");
    return;
  }

  inputForward = 0;  // Start with zero distance traveled

  // Loop until the target distance is reached
  while (abs(inputForward - setPointForward) > 0.5) {  // Allowable error in cm
    // Measure the current distance traveled
    unsigned int currentDistance = sonar.ping_cm();
    if (currentDistance > 0) {
      inputForward = currentDistance - initialDistance;  // Update the distance traveled
    } else {
      Serial.println("Ultrasonic sensor error during distance measurement.");
    }

    forwardPID.Compute();  // Compute PID output

    // Adjust motor speeds based on PID output
    int leftSpeed = constrain(90 + outputForward, 0, 180);
    int rightSpeed = constrain(90 - outputForward, 0, 180);

    leftServo.write(leftSpeed);
    rightServo.write(rightSpeed);

    Serial.print("TargetDistance:");
    Serial.print(setPointForward);
    Serial.print(",Traveled:");
    Serial.print(inputForward);
    Serial.print(",Output:");
    Serial.println(outputForward);

    delay(10);  // Small delay for PID stability
  }

  stopRobot();  // Stop the robot after reaching the target distance
}

float getCurrentAngle() {
  if (imu.getSensorEvent() == false || imu.getSensorEventID() != SENSOR_REPORTID_ROTATION_VECTOR) {
    currentAngle = imu.getYaw();
  }
  return currentAngle;
}

void moveDistanceTime(float distanceCm) {
  // Calculate the time in milliseconds required to move the specified distance
  float timeMs = (distanceCm / speedCmPerSec) * 1000;

  // Move the robot forward at full speed for the calculated time
  leftServo.write(132);  // Full speed forward
  rightServo.write(45);   // Full speed forward

  // Delay for the calculated time
  delay(timeMs);

  // Stop the robot
  stopRobot();

  // Debugging information
  Serial.print("Moved ");
  Serial.print(distanceCm);
  Serial.print(" cm in ");
  Serial.print(timeMs);
  Serial.println(" ms.");
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
  // Normalize angles to [-π, π)

  float currentAngle = getCurrentAngle();
  float angularDifference = normalizeAngleToPi(targetAngle - currentAngle);

  // PID setup
  turn90PID.SetMode(AUTOMATIC);
  turn90PID.SetOutputLimits(-90, 90);  // Output limits for motor control

  // Rotate until the angular difference is within the tolerance
  while (abs(angularDifference) > 0.05) {  // Allowable error (tolerance) in radians
    currentAngle = getCurrentAngle();      // Update current angle
    angularDifference = normalizeAngleToPi(targetAngle - currentAngle);

    input90 = currentAngle;  // Update PID input
    turn90PID.Compute();     // Compute PID output

    // Adjust motor speeds based on PID output
    int leftSpeed = constrain(90 + output90, 0, 180);
    int rightSpeed = constrain(90 - output90, 0, 180);

    leftServo.write(leftSpeed);
    rightServo.write(rightSpeed);

    Serial.print("currentAngle:");
    Serial.print(currentAngle);
    Serial.print(",AngularDifference:");
    Serial.print(angularDifference);
    Serial.print(",output90:");
    Serial.print(output90);
    Serial.println(",");

    delay(10);  // Small delay for PID stability
  }

  stopRobot();  // Stop the robot after reaching the target angle
}

void rotateRight() {
  float currentAngle = getCurrentAngle();
  float targetAngle = normalizeAngleToPi(currentAngle - PI / 2);  // Rotate 90 degrees right
  rotateToAngle(targetAngle);
}

void rotateLeft() {
  float currentAngle = getCurrentAngle();
  float targetAngle = normalizeAngleToPi(currentAngle + PI / 2);  // Rotate 90 degrees left
  rotateToAngle(targetAngle);
}

void rotateRightTime(float time) {
  leftServo.write(180);
  rightServo.write(180);
  delay(time * 1000);
  stopRobot();
}

void rotateLeftTime(float time) {
  leftServo.write(0);
  rightServo.write(0);
  delay(time * 1000);
  stopRobot();
}

void setupIMU() {
  Wire.begin();
  Wire.flush();

  //if (imu.begin() == false) {  // Setup without INT/RST control (Not Recommended)
  if (imu.begin(BNO08X_ADDR, Wire, -1, -1) == false) {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1)
      ;
  }
  Serial.println("BNO08x found!");

  // Wire.setClock(400000); //Increase I2C data rate to 400kHz

  setReports();

  Serial.println("Reading events");
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
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
  // Set up serial communication
  Serial.begin(115200);

  // Attach servos
  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);

  // Initialize the BNO08X over I2C using Qwiic
  setupIMU();

  moveDistanceTime(40);
}

void loop() {
  // IMU stuff
  if (imu.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
  
  // Print the distance to the serial monitor
  // unsigned int distance = sonar.ping_cm();
  // Serial.print("Distance:" + String(distance) + ",");

  // Has a new event come in on the Sensor Hub Bus?
  if (imu.getSensorEvent() == true && imu.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
    float yaw = imu.getYaw();

    Serial.print("Yaw:");
    Serial.print(yaw);
    Serial.print(",");
  }

  Serial.println();

  delay(10);
}
