"""webots_ros2_driver plugin for the ME480 4-wheel skid-steer robot.

Publishes
  /PointCloud  sensor_msgs/PointCloud    raw lidar hits in the lidar frame
  /gps         geometry_msgs/PointStamped robot position in the world (map) frame
  /imu         sensor_msgs/Imu           robot orientation in the world (map) frame
Subscribes
  /cmd_vel     geometry_msgs/Twist       linear.x [m/s], angular.z [rad/s]
"""

import math

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from controller.wb import wb
from geometry_msgs.msg import Point32, PointStamped, Twist
from sensor_msgs.msg import Imu, PointCloud

WHEEL_RADIUS = 0.04       # m, matches ME480Robot.proto
HALF_TRACK = 0.06         # m, lateral distance from body center to wheel center
LIDAR_PERIOD_MS = 100     # 10 Hz scans
# Lidar hits closer than this are the robot's own body/wheels.
SELF_FILTER_RADIUS = 0.15  # m

# Memory layout of WbLidarPoint in the Webots C API.
LIDAR_POINT_DTYPE = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('layer', '<i4'), ('time', '<f4'),
])


def stamp_from_seconds(seconds):
    sec = int(seconds)
    return Time(sec=sec, nanosec=int(round((seconds - sec) * 1e9)))


class ME480Driver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        timestep = int(self.__robot.getBasicTimeStep())

        self.__lidar = self.__robot.getDevice('lidar')
        lidar_period = max(timestep, round(LIDAR_PERIOD_MS / timestep) * timestep)
        self.__lidar.enable(lidar_period)
        self.__lidar.enablePointCloud()
        self.__lidar_period = lidar_period / 1000.0
        self.__last_scan_time = -math.inf

        self.__gps = self.__robot.getDevice('gps')
        self.__gps.enable(timestep)
        self.__imu = self.__robot.getDevice('imu')
        self.__imu.enable(timestep)

        self.__left_wheels = [self.__robot.getDevice('wheel1'), self.__robot.getDevice('wheel3')]
        self.__right_wheels = [self.__robot.getDevice('wheel2'), self.__robot.getDevice('wheel4')]
        for wheel in self.__left_wheels + self.__right_wheels:
            wheel.setPosition(float('inf'))
            wheel.setVelocity(0.0)
        self.__max_wheel_speed = self.__left_wheels[0].getMaxVelocity()

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('me480_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__cloud_pub = self.__node.create_publisher(PointCloud, 'PointCloud', 1)
        self.__gps_pub = self.__node.create_publisher(PointStamped, 'gps', 10)
        self.__imu_pub = self.__node.create_publisher(Imu, 'imu', 10)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def __drive(self):
        v = self.__target_twist.linear.x
        w = self.__target_twist.angular.z
        left = (v - w * HALF_TRACK) / WHEEL_RADIUS
        right = (v + w * HALF_TRACK) / WHEEL_RADIUS
        # Scale both sides together so turning radius is kept when saturating.
        scale = max(1.0, abs(left) / self.__max_wheel_speed, abs(right) / self.__max_wheel_speed)
        for wheel in self.__left_wheels:
            wheel.setVelocity(left / scale)
        for wheel in self.__right_wheels:
            wheel.setVelocity(right / scale)

    def __publish_pose(self, stamp):
        x, y, z = self.__gps.getValues()
        if any(math.isnan(value) for value in (x, y, z)):
            return
        gps_msg = PointStamped()
        gps_msg.header.stamp = stamp
        gps_msg.header.frame_id = 'map'
        gps_msg.point.x, gps_msg.point.y, gps_msg.point.z = x, y, z
        self.__gps_pub.publish(gps_msg)

        qx, qy, qz, qw = self.__imu.getQuaternion()
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = 'base_link'
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        # The InertialUnit provides no rates or accelerations.
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0
        self.__imu_pub.publish(imu_msg)

    def __publish_cloud(self, stamp):
        count = self.__lidar.getNumberOfPoints()
        raw = wb.wb_lidar_get_point_cloud(self.__lidar._tag)
        if count == 0 or not raw:
            return
        points = np.frombuffer(bytes(raw[:count * LIDAR_POINT_DTYPE.itemsize]), dtype=LIDAR_POINT_DTYPE)
        xyz = np.stack((points['x'], points['y'], points['z']), axis=1)
        # Rays that hit nothing come back as +inf.
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        xyz = xyz[np.einsum('ij,ij->i', xyz, xyz) > SELF_FILTER_RADIUS ** 2]

        msg = PointCloud()
        msg.header.stamp = stamp
        msg.header.frame_id = 'lidar_link'
        msg.points = [Point32(x=x, y=y, z=z) for x, y, z in xyz.tolist()]
        self.__cloud_pub.publish(msg)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        self.__drive()

        now = self.__robot.getTime()
        stamp = stamp_from_seconds(now)
        self.__publish_pose(stamp)
        if now - self.__last_scan_time >= self.__lidar_period - 1e-6:
            self.__last_scan_time = now
            self.__publish_cloud(stamp)
