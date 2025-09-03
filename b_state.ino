// Robot state
float robotX = startX; // Current x position
float robotY = startY; // Current y position
float robotTheta = 0.0; // Current orientation (radians)

// Wheel encoder readings
long previousLeftEncoder = 0;  // Previous left encoder count
long previousRightEncoder = 0; // Previous right encoder count

void updateRobotState() {
    // Get the current IMU yaw angle for robotTheta
    robotTheta = getCurrentAngle() + PI / 2;  // Ensure IMU yaw is normalized

    // Read the current encoder counts
    long currentLeftEncoder = leftEncoder.read() * -1;
    long currentRightEncoder = rightEncoder.read() * -1;

    // Calculate the distance traveled by each wheel
    float leftWheelDistance = (currentLeftEncoder - previousLeftEncoder) * wheelCircumference / (150 * 7 * 4);  // Convert encoder rotation ticks to distance
    float rightWheelDistance = (currentRightEncoder - previousRightEncoder) * wheelCircumference / (150 * 7 * 4);

    // Update the previous encoder counts for the next calculation
    previousLeftEncoder = currentLeftEncoder;
    previousRightEncoder = currentRightEncoder;

    // Calculate the average distance traveled
    float distanceTravelled = (leftWheelDistance + rightWheelDistance) / 2.0;

    // Update the robot's global position
    robotX += distanceTravelled * cos(robotTheta);
    robotY += distanceTravelled * sin(robotTheta);

    robotTheta = normalizeAngleToPi(robotTheta);

    // robotTheta = fmod(robotTheta + PI, 2 * PI) - PI;

    // Print debug information
    // Serial.print("RobotX: "); Serial.print(robotX);
    // Serial.print(", RobotY: "); Serial.print(robotY);
    // Serial.print(", RobotTheta: "); Serial.println(robotTheta);
}

void updateRobotStateOdometryOnly() {
    // Read the current encoder counts
    long currentLeftEncoder = leftEncoder.read() * -1;
    long currentRightEncoder = rightEncoder.read() * -1;

    // Calculate the distance traveled by each wheel
    float leftWheelDistance = (currentLeftEncoder - previousLeftEncoder) * wheelCircumference / (150 * 7 * 4);  // Convert encoder rotation ticks to distance
    float rightWheelDistance = (currentRightEncoder - previousRightEncoder) * wheelCircumference / (150 * 7 * 4);

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
    // Serial.print("RobotX: "); Serial.print(robotX);
    // Serial.print(", RobotY: "); Serial.print(robotY);
    // Serial.print(", RobotTheta: "); Serial.println(robotTheta);
    Serial.print(leftEncoder.read());
    Serial.print(" ");
    Serial.println(rightEncoder.read());
}

// void updateRobotStateOdometryOnly() {
//     // Read the current encoder counts
//     long currentLeftEncoder = leftEncoder.read() * -1;
//     long currentRightEncoder = rightEncoder.read() * -1;

//     // Calculate the distance traveled by each wheel
//     float leftWheelDistance = (currentLeftEncoder - previousLeftEncoder) * wheelCircumference / (150 * 7 * 4);  // Convert encoder ticks to distance
//     float rightWheelDistance = (currentRightEncoder - previousRightEncoder) * wheelCircumference / (150 * 7 * 4);

//     // Update previous encoder counts for next iteration
//     previousLeftEncoder = currentLeftEncoder;
//     previousRightEncoder = currentRightEncoder;

//     // Calculate the change in orientation (Δθ) based on differential drive kinematics
//     float deltaTheta = (rightWheelDistance - leftWheelDistance) / wheelBase;

//     // Handle small movements to avoid division by zero errors
//     if (fabs(deltaTheta) < 1e-6) {
//         // If the robot is moving in a straight line
//         float distanceTravelled = (leftWheelDistance + rightWheelDistance) / 2.0;
//         robotX += distanceTravelled * cos(robotTheta);
//         robotY += distanceTravelled * sin(robotTheta);
//     } else {
//         // Compute radius of curvature R
//         float R = (wheelBase / 2.0) * ((leftWheelDistance + rightWheelDistance) / (rightWheelDistance - leftWheelDistance));

//         // Compute the new center of rotation
//         float ICC_x = robotX - R * sin(robotTheta);
//         float ICC_y = robotY + R * cos(robotTheta);

//         // Update robot's position using rotation about ICC
//         robotX = cos(deltaTheta) * (robotX - ICC_x) - sin(deltaTheta) * (robotY - ICC_y) + ICC_x;
//         robotY = sin(deltaTheta) * (robotX - ICC_x) + cos(deltaTheta) * (robotY - ICC_y) + ICC_y;
//     }

//     // Update the robot's orientation (robotTheta)
//     robotTheta += deltaTheta;

//     // Normalize robotTheta to the range [-π, π)
//     robotTheta = fmod(robotTheta + PI, 2 * PI) - PI;

//     // Print debug information
//     // Serial.print("RobotX: "); Serial.print(robotX);
//     // Serial.print(", RobotY: "); Serial.print(robotY);
//     // Serial.print(", RobotTheta: "); Serial.println(robotTheta);
//     // Serial.print(leftEncoder.read());
//     // Serial.print(" ");
//     // Serial.println(rightEncoder.read());
// }
