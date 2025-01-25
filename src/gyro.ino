// IMU Constants
#define BNO08X_INT A4
#define BNO08X_RST A5
#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B

float currentAngle = 0.0;

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