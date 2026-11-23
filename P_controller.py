from controller import Robot

robot = Robot()

TIME_STEP = int(robot.getBasicTimeStep())

# Motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

# Velocity control mode
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# GPS
gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

# Controller parameters
Kp = 2.0
MAX_SPEED = 6.28

# Target position (meters)
target_position = 1.0

# Tolerance
tolerance = 0.02

while robot.step(TIME_STEP) != -1:

    # Read current position
    current_position = gps.getValues()[0]  # X coordinate

    # Calculate error
    error = target_position - current_position

    # P controller
    command = Kp * error

    # Limit command
    command = max(min(command, MAX_SPEED), -MAX_SPEED)

    # Stop when close enough
    if abs(error) < tolerance:
        command = 0.0

    # Drive both wheels
    left_motor.setVelocity(command)
    right_motor.setVelocity(command)

    print("Position:", current_position,
          "Target:", target_position,
          "Error:", error,
          "Command:", command)
