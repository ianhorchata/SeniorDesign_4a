"""Broadcasts map -> base_link from the simulated GPS position and IMU orientation."""

import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        self.__map_frame = self.declare_parameter('map_frame', 'map').value
        self.__base_frame = self.declare_parameter('base_frame', 'base_link').value

        self.__position = None
        self.__broadcaster = TransformBroadcaster(self)
        self.create_subscription(PointStamped, 'gps', self.__gps_callback, 10)
        self.create_subscription(Imu, 'imu', self.__imu_callback, 10)

    def __gps_callback(self, msg):
        self.__position = msg.point

    def __imu_callback(self, msg):
        if self.__position is None:
            return
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.__map_frame
        transform.child_frame_id = self.__base_frame
        transform.transform.translation.x = self.__position.x
        transform.transform.translation.y = self.__position.y
        transform.transform.translation.z = self.__position.z
        transform.transform.rotation = msg.orientation
        self.__broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
