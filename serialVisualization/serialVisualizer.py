import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

# Attempt to connect to the serial port
connected = False
while not connected:
    try:
        ser = serial.Serial('/dev/cu.usbmodem101', 115200)  # Replace 'COM3' with your Arduino's port
        connected = True
    except serial.SerialException:
        print("Failed to connect to serial port. Retrying...")

# Initialize plotting
fig, ax = plt.subplots()
robot_path_x = []  # To store the robot's x positions
robot_path_y = []  # To store the robot's y positions
robot_position, = ax.plot([], [], 'b-', label="Robot Path")  # Blue line for robot path
current_position, = ax.plot([], [], 'bo', label="Robot Position")  # Blue circle for robot
waypoints, = ax.plot([], [], 'ro-', label="Waypoints")  # Red line for waypoints
target_point, = ax.plot([], [], 'go', label="Target Point")  # Green circle for target
actual_target_point, = ax.plot([], [], 'yo', label="Actual Target Point")  # Green circle for target

# Define waypoints (same as in the Arduino code)
waypoint_x = [0.0, 0.0, 0.5, -0.5, -0.5, -1.0, -1.0, -1.0, -0.5, -0.5, -1.0, -0.5, -0.5, 0.0, 0.0, 0.0, -1.5, -1.5]
waypoint_y = [0.0, 0.5, 0.6, 0.5, 1.0, 1.0, 0.5, 1.0, 1.0, 1.5, 1.5, 1.5, 1.0, 1.0, 1.5, 1.0, 1.0, 1.5]


# Set axis limits
ax.set_xlim(-2, 1)
ax.set_ylim(-0.5, 2)
ax.set_aspect('equal')
# ax.legend()

# Update function for animation
def update(frame):
    ser.flushInput()  # Clear the serial input buffer
    time.sleep(0.001)
    line = ser.readline().decode('utf-8').strip()
    print("Serial Data:", line)  # Debugging print
    line = ser.readline().decode('utf-8').strip()
    print("Serial Data:", line)  # Debugging print

    if not line.startswith("RobotX"):
        return

    # Parse the incoming data
    data = {key: float(value) for key, value in [pair.split(":") for pair in line.split(",")]}
    robot_x = data["RobotX"]
    robot_y = data["RobotY"]
    target_x = data["TargetX"]
    target_y = data["TargetY"]
    actual_target_x = data["ActualTargetX"]
    actual_target_y = data["ActualTargetY"]

    print("Robot Position:", robot_x, robot_y)  # Debugging print

    # Update the robot path and position
    robot_position.set_data(robot_path_x, robot_path_y)
    current_position.set_data([robot_x], [robot_y])  # Current position is a single point

    # Update waypoints and target point
    waypoints.set_data(waypoint_x, waypoint_y)
    target_point.set_data([target_x], [target_y])
    actual_target_point.set_data([actual_target_x], [actual_target_y])

    # Append the current position to the path
    robot_path_x.append(robot_x)
    robot_path_y.append(robot_y)

# Animation
ani = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
plt.show()