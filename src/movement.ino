// PID variables
double input90, output90, setPoint90;
double kP1 = 500, kI1 = 100, kD1 = 0;
PID turn90PID(&input90, &output90, &setPoint90, kP1, kI1, kD1, DIRECT);

// PID variables for distance control
double inputDistance, outputDistance, setPointDistance;
double kPDistance = 1.0, kIDistance = 0.1, kDDistance = 0.05;
PID distancePID(&inputDistance, &outputDistance, &setPointDistance, kPDistance, kIDistance, kDDistance, DIRECT);

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
  turn90PID.SetOutputLimits(-255, 255);  // Output limits for motor control (steps per second)

  while (abs(angularDifference) > 0.01) {  // Allowable error (tolerance) in radians
    currentAngle = getCurrentAngle();      // Update current angle
    angularDifference = normalizeAngleToPi(targetAngle - currentAngle);

    input90 = angularDifference;  // Update PID input
    turn90PID.Compute();     // Compute PID output\

    if (output90 > 0) {
      leftMotor.setSpeed(output90);
      rightMotor.setSpeed(output90);
      leftMotor.run(L298N::BACKWARD);
      rightMotor.run(L298N::FORWARD);
    } else {
      leftMotor.setSpeed(-output90);
      rightMotor.setSpeed(-output90);
      leftMotor.run(L298N::FORWARD);
      rightMotor.run(L298N::BACKWARD);
    }

    Serial.print("currentAngle:");
    Serial.print(currentAngle);
    Serial.print(",AngularDifference:");
    Serial.print(angularDifference);
    Serial.print(",output90:");
    Serial.print(output90);
    Serial.println(",");

    delay(1);  // Small delay for PID stability
  }

  leftMotor.stop();
  rightMotor.stop();
  delay(50);
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

// void driveForwardDistance(double distance) {
//   int steps = distance * 2048;  // Calculate total steps for the given number of rotations

//   // Reverse the left stepper direction by negating the steps
//   leftMotor.move(-steps);
//   rightMotor.move(steps);

//   // Run the steppers until they reach the target position
//   while (leftMotor.distanceToGo() != 0 || rightMotor.distanceToGo() != 0) {
//     leftMotor.run();
//     rightMotor.run();
//   }

//   leftMotor.stop();
//   rightMotor.stop();
// }

void driveForwardDistance(double distance) {
  // Convert distance to encoder ticks
  double targetTicks = (distance / wheelCircumference) * 1050;  // Assuming 360 ticks per revolution
  leftEncoder.write(0);  // Reset encoder counts
  rightEncoder.write(0);

  setPointDistance = targetTicks;  // Target encoder count
  distancePID.SetMode(AUTOMATIC);  // Enable PID
  distancePID.SetOutputLimits(-255, 255);  // Limit PID output to valid motor speeds

  while (true) {
    // Read encoder values
    double leftTicks = abs(leftEncoder.read());
    double rightTicks = abs(rightEncoder.read());
    double averageTicks = (leftTicks + rightTicks) / 2.0;  // Average of both encoders

    inputDistance = averageTicks;  // Update PID input
    distancePID.Compute();         // Compute PID output

    // Control motor speeds based on PID output
    if (outputDistance > 0) {
      leftMotor.setSpeed(outputDistance);
      rightMotor.setSpeed(outputDistance);
      leftMotor.run(L298N::FORWARD);
      rightMotor.run(L298N::FORWARD);
    } else {
      leftMotor.setSpeed(-outputDistance);
      rightMotor.setSpeed(-outputDistance);
      leftMotor.run(L298N::BACKWARD);
      rightMotor.run(L298N::BACKWARD);
    }

    // Break the loop when the target distance is reached
    if (abs(averageTicks - targetTicks) < 1) {  // Tolerance of 1 tick
      break;
    }

    delay(1);  // Small delay for PID stability
  }

  // Stop motors after reaching the target
  leftMotor.stop();
  rightMotor.stop();

  delay(10);
}

void driveForwardDistanceTimed(double distance, long timeToComplete) {
  // Convert distance to encoder ticks
  double targetTicks = (distance / wheelCircumference) * 360;  // Assuming 360 ticks per revolution
  leftEncoder.write(0);  // Reset encoder counts
  rightEncoder.write(0);

  // Calculate the average ticks per second required to complete the distance in time
  double ticksPerSecond = targetTicks / (timeToComplete / 1000.0);

  setPointDistance = ticksPerSecond;  // Target ticks per second
  distancePID.SetMode(AUTOMATIC);    // Enable PID
  distancePID.SetOutputLimits(-255, 255);  // Limit PID output to valid motor speeds

  unsigned long startTime = millis();
  while (millis() - startTime < timeToComplete) {
    // Read encoder values
    double leftTicks = abs(leftEncoder.read());
    double rightTicks = abs(rightEncoder.read());
    double averageTicks = (leftTicks + rightTicks) / 2.0;  // Average of both encoders

    // Calculate the current speed (ticks per second)
    double elapsedTime = (millis() - startTime) / 1000.0;  // Elapsed time in seconds
    double currentTicksPerSecond = averageTicks / elapsedTime;

    inputDistance = currentTicksPerSecond;  // Update PID input
    distancePID.Compute();                  // Compute PID output

    // Control motor speeds based on PID output
    if (outputDistance > 0) {
      leftMotor.setSpeed(outputDistance);
      rightMotor.setSpeed(outputDistance);
      leftMotor.run(L298N::FORWARD);
      rightMotor.run(L298N::FORWARD);
    } else {
      leftMotor.setSpeed(-outputDistance);
      rightMotor.setSpeed(-outputDistance);
      leftMotor.run(L298N::BACKWARD);
      rightMotor.run(L298N::BACKWARD);
    }

    // Break the loop if the target ticks are reached before time ends
    if (averageTicks >= targetTicks) {
      break;
    }

    delay(10);  // Small delay for PID stability
  }

  // Stop motors after reaching the target or time limit
  leftMotor.stop();
  rightMotor.stop();
}