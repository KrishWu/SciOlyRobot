//Pure pursuit
// Waypoints (x, y coordinates in meters)
const float waypoints[][2] = {
    {0.0, 0.0},
    {1.0, 1.0},
    {1.0, 1.5},
    {0.0, 2.0}
};
const int numWaypoints = sizeof(waypoints) / sizeof(waypoints[0]);

// Function to compute the Euclidean distance
float distance(float x1, float y1, float x2, float y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// Pure pursuit control for DC motors
void purePursuit() {
    // Find the closest point on the path within the lookahead distance
    int targetIndex = -1;
    float minDistance = 9999.0;

    for (int i = 0; i < numWaypoints; i++) {
        float dist = distance(robotX, robotY, waypoints[i][0], waypoints[i][1]);

        if (dist > lookaheadDistance) {
            if (dist < minDistance) {
                minDistance = dist;
                targetIndex = i;
            }
        }
    }

    // If no valid target found, stop the robot
    if (targetIndex == -1) {
        leftMotor.stop();
        rightMotor.stop();
        return;
    }

    // Get the target waypoint
    float targetX = waypoints[targetIndex][0];
    float targetY = waypoints[targetIndex][1];

    // Compute the target point in robot's local frame
    float dx = targetX - robotX;
    float dy = targetY - robotY;

    float localX = dx * cos(-robotTheta) - dy * sin(-robotTheta);
    float localY = dx * sin(-robotTheta) + dy * cos(-robotTheta);

    // Compute curvature
    float curvature = 2 * localY / (pow(localX, 2) + pow(localY, 2));

    // Compute wheel speeds
    float leftSpeed = maxSpeed * (1 - (curvature * wheelBase) / 2);
    float rightSpeed = maxSpeed * (1 + (curvature * wheelBase) / 2);

    leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
    rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

    // // Map speeds to PWM values
    // int leftPWM = map(constrain(leftSpeed, 0, maxSpeed), 0, maxSpeed, 0, 255);
    // int rightPWM = map(constrain(rightSpeed, 0, maxSpeed), 0, maxSpeed, 0, 255);

    // // Set motor speeds
    // analogWrite(leftMotorPin, leftPWM);
    // analogWrite(rightMotorPin, rightPWM);

    // Set motor speeds and directions
    if (leftSpeed >= 0) {
        leftMotor.setSpeed(leftSpeed);
        leftMotor.run(L298N::FORWARD);
    } else {
        leftMotor.setSpeed(-leftSpeed);
        leftMotor.run(L298N::BACKWARD);
    }

    if (rightSpeed >= 0) {
        rightMotor.setSpeed(rightSpeed);
        rightMotor.run(L298N::FORWARD);
    } else {
        rightMotor.setSpeed(-rightSpeed);
        rightMotor.run(L298N::BACKWARD);
    }

    // ** Send data for visualization **
    Serial.print("RobotX:"); Serial.print(robotX);
    Serial.print(",RobotY:"); Serial.print(robotY);
    Serial.print(",TargetX:"); Serial.print(targetX);
    Serial.print(",TargetY:"); Serial.print(targetY);
    Serial.print(",Theta:"); Serial.print(robotTheta);
    Serial.print(",LeftSpeed:"); Serial.print(leftSpeed);
    Serial.print(",RightSpeed:"); Serial.println(rightSpeed);
}