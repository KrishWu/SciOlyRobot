//Pure pursuit
// Waypoints (x, y coordinates in meters)
const float waypoints[][2] = {
    {0.0, 0.0},
    {0.0, 1.0},
    {0.5, 1.0},
    {0.0, 1.0},
    {0.0, 0.0},
    {1.0, 0.0},
    {1.0, 1.0},
    {0.0, 1.0},
    {1.0, 1.0}
    // {0.0, 2.0}
};
const int numWaypoints = sizeof(waypoints) / sizeof(waypoints[0]);

int totalDistanceToTravel = 0;

int currentWaypoint = 0;

double inputPurePursuit, outputPurePursuit, setPointPurePursuit;
double kPPurePursuit = 0.20, kIPurePursuit = 0.0, kDPurPursuit = 0.0;
PID purePursuitPID(&inputPurePursuit, &outputPurePursuit, &setPointPurePursuit, kPPurePursuit, kIPurePursuit, kDPurPursuit, DIRECT);

long lastSegmentStartTime;
double lastSegmentDistance;

// Function to compute the Euclidean distance
float calculateDistance(float x1, float y1, float x2, float y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

void calculateTotalDistance() {
  for (int i = 0; i < numWaypoints - 1; i++) {
    totalDistanceToTravel += calculateDistance(waypoints[i][0], waypoints[i][1], waypoints[i+1][0], waypoints[i+1][0]);
  }
}

double calculateDistanceRemaining() {
  double distanceRemaining = calculateDistance(robotX, robotY, waypoints[currentWaypoint][0], waypoints[currentWaypoint][1]);
  for (int i = currentWaypoint; i < numWaypoints - 1; i++) {
    distanceRemaining += calculateDistance(waypoints[i][0], waypoints[i][1], waypoints[i+1][0], waypoints[i+1][0]);
  }
  return distanceRemaining;
}

// Function to find the closest waypoint in Pure Pursuit
int findLookaheadPoint() {
  for (int i = currentWaypoint; i < numWaypoints - 1; i++) {
    float dx = waypoints[i][0] - robotX;
    float dy = waypoints[i][1] - robotY;
    float distance = sqrt(dx * dx + dy * dy);
    if (distance > lookaheadDistance) {
      return i;
    }
    currentWaypoint++;
    if (currentWaypoint == numWaypoints - 1 && millis() - startTime < desiredTotalDuration) {
      lastSegmentStartTime == millis();
      lastSegmentDistance = calculateDistance(robotX, robotY, waypoints[numWaypoints - 1][0], waypoints[numWaypoints - 1][1]) / wheelCircumference * 150 * 7 * 4; //Distance in ticks
      setPointPurePursuit = lastSegmentDistance / ((desiredTotalDuration - (millis() - startTime)) / 1000.0);

      purePursuitPID.SetMode(AUTOMATIC);
      purePursuitPID.SetOutputLimits(0, 255);
    }
  }
  return numWaypoints - 1; // Last waypoint
}

// Function to find the intersection of lookahead circle with the path line
bool findLookaheadIntersection(float x1, float y1, float x2, float y2, float &targetX, float &targetY) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    float d = sqrt(dx * dx + dy * dy);

    if (d < 0.0001) return false; // Prevent division by zero

    // Normalize direction vector
    float unitX = dx / d;
    float unitY = dy / d;

    // Compute the projection of the robot onto the path
    float projX = x1 + unitX * ((robotX - x1) * unitX + (robotY - y1) * unitY);
    float projY = y1 + unitY * ((robotX - x1) * unitX + (robotY - y1) * unitY);

    // Clamp projection to the segment
    float t = ((projX - x1) * dx + (projY - y1) * dy) / (dx * dx + dy * dy);
    if (t < 0) {
        projX = x1;
        projY = y1;
    } else if (t > 1) {
        projX = x2;
        projY = y2;
    }

    // Compute perpendicular distance from robot to path
    float perpDist = sqrt(pow(projX - robotX, 2) + pow(projY - robotY, 2));

    if (perpDist > lookaheadDistance) {
        // If no valid intersection, set target to the closest point on the line
        targetX = projX;
        targetY = projY;
        return false;
    }

    // Compute intersection points
    float offset = sqrt(pow(lookaheadDistance, 2) - pow(perpDist, 2));

    // Two possible intersection points
    float intX1 = projX + unitX * offset;
    float intY1 = projY + unitY * offset;
    float intX2 = projX - unitX * offset;
    float intY2 = projY - unitY * offset;

    // Choose the intersection closer to the current waypoint
    float dist1 = sqrt(pow(intX1 - x2, 2) + pow(intY1 - y2, 2));
    float dist2 = sqrt(pow(intX2 - x2, 2) + pow(intY2 - y2, 2));

    if (dist1 < dist2) {
        targetX = intX1;
        targetY = intY1;
    } else {
        targetX = intX2;
        targetY = intY2;
    }

    return true;
}

// Function to execute Pure Pursuit path following
void purePursuit() {
  int targetIndex = findLookaheadPoint();

  // Get the target waypoint
  float waypointX = waypoints[targetIndex][0];
  float waypointY = waypoints[targetIndex][1];

  float targetX = waypointX;
  float targetY = waypointY;

  if (targetIndex == 0) {
    targetX = waypointX;
    targetY = waypointY;
  } else {
    findLookaheadIntersection(waypoints[targetIndex - 1][0], waypoints[targetIndex - 1][1], waypointX, waypointY, targetX, targetY);
  }

  // Compute the target point in robot's local frame
  float dx = targetX - robotX;
  float dy = targetY - robotY;

  // Stop the robot if close enough
  float distance = sqrt(pow(waypointX - robotX, 2) + pow(waypointY - robotY, 2));
  if (currentWaypoint >= numWaypoints) {
    leftMotor.stop();
    rightMotor.stop();
    Serial.println("Path Completed!");
    Serial.println("Path Completed!");
    Serial.println(distance);
    Serial.println(lastSegmentStartTime);
    Serial.println(pow(waypointX - robotX, 2));
    return;
  } else if (distance < 0.10 && currentWaypoint == numWaypoints - 1) { // Reached waypoint
    currentWaypoint++;
    leftMotor.stop();
    rightMotor.stop();
    Serial.println("Path Completed!");
    Serial.println(distance);
    lastSegmentStartTime = millis() - lastSegmentStartTime;
    Serial.println(pow(waypointX - robotX, 2));
    return;
  }

  if (abs(normalizeAngleToPi(robotTheta - atan2(dy, dx))) > PI * 3 / 4) { //BRING BACK LATER WHEN GYRO IS FIXED
    rotateToAngleLessPrecise(atan2(dy, dx));
    return;
  }

  // if (distance > lookaheadDistance) {
  //   dx = dx / distance * lookaheadDistance;
  //   dy = dy / distance * lookaheadDistance;
  // }
  
  float localX = dx * cos(-robotTheta) - dy * sin(-robotTheta);
  float localY = dx * sin(-robotTheta) + dy * cos(-robotTheta);

  // Compute curvature
  float curvature = 2 * localY / (pow(localX, 2) + pow(localY, 2));

  // Compute wheel speeds
  float leftSpeed = maxSpeed * (1 - (curvature * wheelBase) / 2);
  float rightSpeed = maxSpeed * (1 + (curvature * wheelBase) / 2);

  // leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  // rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  float maxAllowedSpeed = max(max(abs(leftSpeed), abs(rightSpeed)), (float)maxSpeed);
  leftSpeed = leftSpeed / maxAllowedSpeed * maxSpeed;
  rightSpeed = rightSpeed / maxAllowedSpeed * maxSpeed;

  if (currentWaypoint == numWaypoints - 1 && millis() - startTime < desiredTotalDuration) {
    double distanceRemaining = calculateDistance(robotX, robotY, waypoints[numWaypoints - 1][0], waypoints[numWaypoints - 1][1]);
    double distanceTraveled = (lastSegmentDistance - distanceRemaining / wheelCircumference * (150 * 7 * 4));
    double timeElapsed = millis() - lastSegmentStartTime;
    inputPurePursuit = distanceTraveled / (timeElapsed / 1000.0);
    purePursuitPID.Compute();

    double multiplier = outputPurePursuit;
    Serial.print("Input:"); Serial.print(inputPurePursuit);
    Serial.print(",SetPoint:"); Serial.print(setPointPurePursuit);
    Serial.print(",Multiplier:"); Serial.print(multiplier);
    Serial.print(",DistanceTraveled:"); Serial.print(distanceTraveled);
    Serial.print(",TimeElapsed:"); Serial.println(timeElapsed);
    leftSpeed = leftSpeed / maxSpeed * multiplier;
    rightSpeed = rightSpeed / maxSpeed * multiplier;
  }

  // Apply motor commands
  leftMotor.setSpeed((int)abs(leftSpeed));
  rightMotor.setSpeed((int)abs(rightSpeed));

  if (leftSpeed >= 0) {
    leftMotor.run(L298N::FORWARD);
  } else {
    leftMotor.run(L298N::BACKWARD);
  }
  if (rightSpeed >= 0) {
    rightMotor.run(L298N::FORWARD);
  } else {
    rightMotor.run(L298N::BACKWARD);
  }

  // ** Send data for visualization **
  Serial.print("RobotX:"); Serial.print(robotX);
  Serial.print(",RobotY:"); Serial.print(robotY);
  Serial.print(",TargetX:"); Serial.print(waypointX);
  Serial.print(",TargetY:"); Serial.print(waypointY);
  Serial.print(",ActualTargetX:"); Serial.print(robotX + dx);
  Serial.print(",ActualTargetY:"); Serial.print(robotY + dy);
  Serial.print(",LocalX:"); Serial.print(localX);
  Serial.print(",LocalY:"); Serial.print(localY);
  Serial.print(",Theta:"); Serial.print(robotTheta);
  Serial.print(",AngleDifference:"); Serial.print(normalizeAngleToPi(robotTheta - atan2(dy, dx)));
  Serial.print(",LeftSpeed:"); Serial.print(leftSpeed);
  Serial.print(",RightSpeed:"); Serial.print(rightSpeed);
  // Serial.print(",LeftMotorSpeed:"); Serial.print(leftMotor.getSpeed());
  // Serial.print(",RightMotorSpeed:"); Serial.print(rightMotor.getSpeed());
  Serial.print(",Distance:"); Serial.print(distance);
  Serial.print(",CurrentWaypoint:"); Serial.print(currentWaypoint);
  Serial.print(",LeftEncoder:"); Serial.print(leftEncoder.read());
  Serial.print(",RightEncoder:"); Serial.println(rightEncoder.read());
}