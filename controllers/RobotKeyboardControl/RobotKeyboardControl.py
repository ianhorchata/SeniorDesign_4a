"""Drive the ME480 robot with the keyboard.

Click in the Webots 3D view first so that it receives the keys.

    W or Up     forward         A or Left    turn left
    S or Down   backward        D or Right   turn right

Keys can be combined (W + A drives a left arc). Releasing all keys stops the robot.

The controller works in two places:
  * As the controller of the ME480Robot itself it drives the wheel motors
    directly. This needs no ROS.
  * In the ROS 2 launch the robot's controller is webots_ros2_driver, and a robot
    can only have one controller. There it runs on a separate body-less Robot
    node, like the Ros2Supervisor, and publishes /cmd_vel for the driver.
"""

from controller import Keyboard, Robot

LINEAR_SPEED = 0.3    # m/s while a forward/backward key is held
ANGULAR_SPEED = 1.5   # rad/s while a turn key is held

WHEEL_RADIUS = 0.04   # m, matches ME480Robot.proto
HALF_TRACK = 0.06     # m, lateral distance from body center to wheel center
LEFT_WHEELS = ('wheel1', 'wheel3')
RIGHT_WHEELS = ('wheel2', 'wheel4')

FORWARD_KEYS = {ord('W'), Keyboard.UP}
BACKWARD_KEYS = {ord('S'), Keyboard.DOWN}
LEFT_KEYS = {ord('A'), Keyboard.LEFT}
RIGHT_KEYS = {ord('D'), Keyboard.RIGHT}


def read_command(keyboard):
    """Return (linear m/s, angular rad/s) for the keys that are held right now."""
    pressed = set()
    key = keyboard.getKey()
    while key != -1:
        pressed.add(key & Keyboard.KEY)   # drop the Shift/Ctrl/Alt flags
        key = keyboard.getKey()

    def held(keys):
        return 1.0 if pressed & keys else 0.0

    linear = (held(FORWARD_KEYS) - held(BACKWARD_KEYS)) * LINEAR_SPEED
    angular = (held(LEFT_KEYS) - held(RIGHT_KEYS)) * ANGULAR_SPEED
    return linear, angular


class WheelDrive:
    """Turns the command into wheel speeds on the robot this controller runs on."""

    def __init__(self, robot):
        self.__left = [robot.getDevice(name) for name in LEFT_WHEELS]
        self.__right = [robot.getDevice(name) for name in RIGHT_WHEELS]
        for wheel in self.__left + self.__right:
            wheel.setPosition(float('inf'))
            wheel.setVelocity(0.0)
        self.__max_speed = self.__left[0].getMaxVelocity()

    def send(self, linear, angular):
        left = (linear - angular * HALF_TRACK) / WHEEL_RADIUS
        right = (linear + angular * HALF_TRACK) / WHEEL_RADIUS
        # Scale both sides together so the turning radius is kept when saturating.
        scale = max(1.0, abs(left) / self.__max_speed, abs(right) / self.__max_speed)
        for wheel in self.__left:
            wheel.setVelocity(left / scale)
        for wheel in self.__right:
            wheel.setVelocity(right / scale)


class CmdVelPublisher:
    """Publishes the command on /cmd_vel for the robot's ROS 2 driver."""

    def __init__(self):
        # Imported here so the controller also runs in Webots without ROS.
        import rclpy
        from geometry_msgs.msg import Twist

        self.__Twist = Twist
        rclpy.init(args=None)
        self.__node = rclpy.create_node('robot_keyboard_control')
        self.__publisher = self.__node.create_publisher(Twist, 'cmd_vel', 1)
        self.__was_moving = False

    def send(self, linear, angular):
        moving = linear != 0.0 or angular != 0.0
        # Publish while a key is held and once more to stop. Staying silent
        # otherwise leaves /cmd_vel free for other nodes, e.g. a path follower.
        if moving or self.__was_moving:
            twist = self.__Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.__publisher.publish(twist)
        self.__was_moving = moving


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    keyboard = robot.getKeyboard()
    keyboard.enable(timestep)

    devices = {robot.getDeviceByIndex(i).getName() for i in range(robot.getNumberOfDevices())}
    if devices.issuperset(LEFT_WHEELS + RIGHT_WHEELS):
        output = WheelDrive(robot)
        target = 'the wheel motors'
    else:
        output = CmdVelPublisher()
        target = '/cmd_vel'
    print(f'RobotKeyboardControl: driving {target}. Click in the 3D view, then use '
          'W/A/S/D or the arrow keys.')

    try:
        while robot.step(timestep) != -1:
            output.send(*read_command(keyboard))
    except KeyboardInterrupt:
        pass


main()
