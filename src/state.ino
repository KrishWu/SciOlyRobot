// Robot state
float robotX = startX; // Current x position
float robotY = startY; // Current y position
float robotTheta = 0.0; // Current orientation (radians)

// Wheel encoder readings
long previousLeftEncoder = 0;  // Previous left encoder count
long previousRightEncoder = 0; // Previous right encoder count

void updateRobotState() {
    // Get the current IMU yaw angle for robotTheta
    robotTheta = getCurrentAngle();  // Ensure IMU yaw is normalized

    // Read the current encoder counts
    long currentLeftEncoder = leftEncoder.read();
    long currentRightEncoder = rightEncoder.read();

    // Calculate the distance traveled by each wheel
    float leftWheelDistance = (currentLeftEncoder - previousLeftEncoder) * (wheelCircumference / 360.0);  // Convert ticks to distance
    float rightWheelDistance = (currentRightEncoder - previousRightEncoder) * (wheelCircumference / 360.0);

    // Update the previous encoder counts for the next calculation
    previousLeftEncoder = currentLeftEncoder;
    previousRightEncoder = currentRightEncoder;

    // Calculate the average distance traveled
    float distanceTravelled = (leftWheelDistance + rightWheelDistance) / 2.0;

    // Update the robot's global position
    robotX += distanceTravelled * cos(robotTheta);
    robotY += distanceTravelled * sin(robotTheta);

    // Print debug information
    Serial.print("RobotX: "); Serial.print(robotX);
    Serial.print(", RobotY: "); Serial.print(robotY);
    Serial.print(", RobotTheta: "); Serial.println(robotTheta);
}

void updateRobotStateOdometryOnly() {
    // Read the current encoder counts
    long currentLeftEncoder = leftEncoder.read();
    long currentRightEncoder = rightEncoder.read();

    // Calculate the distance traveled by each wheel
    float leftWheelDistance = (currentLeftEncoder - previousLeftEncoder) * (wheelCircumference / 360.0);  // Convert ticks to distance
    float rightWheelDistance = (currentRightEncoder - previousRightEncoder) * (wheelCircumference / 360.0);

    // Update the previous encoder counts for the next calculation
    previousLeftEncoder = currentLeftEncoder;
    previousRightEncoder = currentRightEncoder;

    // Calculate the change in orientation (Δθ)
    float deltaTheta = (rightWheelDistance - leftWheelDistance) / wheelBase;

    // Update the robot's orientation (robotTheta)
    robotTheta += deltaTheta;

    // Normalize robotTheta to the range [-π, π)
    robotTheta = fmod(robotTheta + PI, 2 * PI) - PI;

    // Calculate the average traveled distance
    float distanceTravelled = (leftWheelDistance + rightWheelDistance) / 2.0;

    // Update the robot's global position
    robotX += distanceTravelled * cos(robotTheta);
    robotY += distanceTravelled * sin(robotTheta);

    // Print debug information
    Serial.print("RobotX: "); Serial.print(robotX);
    Serial.print(", RobotY: "); Serial.print(robotY);
    Serial.print(", RobotTheta: "); Serial.println(robotTheta);
}