"""Plans paths for the robot over the floors of the OctoMap.

Subscribes
  /octomap_point_cloud_centers  sensor_msgs/PointCloud2     occupied voxels of the map
  /goal_pose                    geometry_msgs/PoseStamped   goal, from the RViz tool "2D Goal Pose".
                                                            That tool always clicks at z = 0.
  /clicked_point                geometry_msgs/PointStamped  goal, from the RViz tool "Publish Point".
                                                            It gives the 3D point under the mouse,
                                                            so it also works on upper floors.
Publishes
  /plan            nav_msgs/Path            path from the robot to the goal, in the map frame
  /drivable_cells  sensor_msgs/PointCloud2  where the planner can drive (x, y, z, clearance)
Service
  /plan_path       nav_msgs/srv/GetPlan     path between any two points. A start with an empty
                                            frame_id means the current position of the robot.

A goal is planned from where the robot is now (tf map -> base_link). Start and goal may be
up to snap_distance away from drivable floor; the path then starts or ends on the nearest
drivable cell. The planning itself is in surface_planner.py.
"""

import dataclasses
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from bringup.surface_planner import (PlannerConfig, PlanningError, SurfacePlanner,
                                     voxels_from_centers)

DRIVABLE_FIELDS = [
    PointField(name=name, offset=4 * i, datatype=PointField.FLOAT32, count=1)
    for i, name in enumerate(('x', 'y', 'z', 'clearance'))
]


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        # Every field of PlannerConfig is a parameter of the same name.
        self.__config = PlannerConfig(**{
            field.name: self.declare_parameter(field.name, field.default).value
            for field in dataclasses.fields(PlannerConfig)
        })
        self.__map_frame = self.declare_parameter('map_frame', 'map').value
        self.__base_frame = self.declare_parameter('base_frame', 'base_link').value
        # The path is published this far above the floor: at the height of base_link,
        # so it can be compared with the robot position and shows above the floor in RViz.
        self.__path_height = self.declare_parameter('path_height', 0.065).value
        # How often the drivable cells are refreshed for RViz while nothing is planned. 0 = never.
        drivable_period = self.declare_parameter('drivable_update_period', 3.0).value

        self.__cloud = None             # latest map
        self.__planner = None           # surface graph ...
        self.__planner_cloud = None     # ... and the map it was built from
        self.__tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.__path_pub = self.create_publisher(Path, 'plan', latched)
        self.__drivable_pub = self.create_publisher(PointCloud2, 'drivable_cells', latched)
        self.create_subscription(PointCloud2, 'octomap_point_cloud_centers', self.__cloud_callback, 1)
        self.create_subscription(PoseStamped, 'goal_pose', self.__goal_pose_callback, 1)
        self.create_subscription(PointStamped, 'clicked_point', self.__clicked_point_callback, 1)
        self.create_service(GetPlan, 'plan_path', self.__plan_path_callback)
        if drivable_period > 0:
            self.create_timer(drivable_period, self.__refresh_drivable_cells)

    # ------------------------------------------------------------------ inputs

    def __cloud_callback(self, msg):
        # Only kept here. The surface graph is built when a path is asked for.
        self.__cloud = msg

    def __goal_pose_callback(self, msg):
        self.__plan_from_robot(msg.header.frame_id, msg.pose.position)

    def __clicked_point_callback(self, msg):
        self.__plan_from_robot(msg.header.frame_id, msg.point)

    def __plan_from_robot(self, frame_id, goal):
        if not self.__in_map_frame(frame_id, 'goal'):
            return
        start = self.__robot_position()
        if start is not None:
            self.__plan(start, (goal.x, goal.y, goal.z))

    def __plan_path_callback(self, request, response):
        response.plan.header.frame_id = self.__map_frame
        if not (self.__in_map_frame(request.start.header.frame_id, 'start')
                and self.__in_map_frame(request.goal.header.frame_id, 'goal')):
            return response
        if request.start.header.frame_id:
            position = request.start.pose.position
            start = (position.x, position.y, position.z)
        else:
            start = self.__robot_position()
        goal = request.goal.pose.position
        if start is not None:
            path = self.__plan(start, (goal.x, goal.y, goal.z))
            if path is not None:
                response.plan = path
        return response

    def __in_map_frame(self, frame_id, what):
        if frame_id in ('', self.__map_frame):
            return True
        self.get_logger().warn(
            f'Ignoring a {what} in frame "{frame_id}": it must be in "{self.__map_frame}".')
        return False

    def __robot_position(self):
        try:
            transform = self.__tf_buffer.lookup_transform(self.__map_frame, self.__base_frame, Time())
        except TransformException as error:
            self.get_logger().warn(f'Cannot plan, the robot position is unknown: {error}')
            return None
        translation = transform.transform.translation
        return (translation.x, translation.y, translation.z)

    # ------------------------------------------------------------------ planning

    def __surface(self):
        """The surface graph of the latest map, rebuilt if the map has changed. None without map."""
        if self.__cloud is None:
            self.get_logger().warn(
                'Cannot plan, no map yet. Is octomap_server running and has the robot seen anything?')
            return None
        if self.__planner_cloud is not self.__cloud:
            cloud = self.__cloud
            centers = point_cloud2.read_points_numpy(cloud, field_names=('x', 'y', 'z'))
            self.__planner_cloud = cloud
            try:
                self.__planner = SurfacePlanner(
                    voxels_from_centers(centers, self.__config.resolution), self.__config)
            except PlanningError as error:
                self.__planner = None
                self.get_logger().warn(f'Cannot plan: {error}')
                return None
            self.__publish_drivable_cells()
        return self.__planner

    def __plan(self, start, goal):
        """Plan, publish and return the path, or None if there is none."""
        started = time.monotonic()
        planner = self.__surface()
        if planner is None:
            return None
        try:
            plan = planner.plan(start, goal)
        except PlanningError as error:
            self.get_logger().warn(f'No path: {error}')
            return None
        path = self.__to_path(plan.points)
        self.__path_pub.publish(path)
        self.get_logger().info(
            f'Planned {plan.length:.2f} m to ({goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f}) with '
            f'{len(plan.waypoints)} waypoints in {time.monotonic() - started:.2f} s.')
        return path

    def __to_path(self, points):
        path = Path()
        path.header.frame_id = self.__map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        yaw = 0.0
        for i, (x, y, z) in enumerate(points.tolist()):
            if i + 1 < len(points):
                yaw = math.atan2(points[i + 1][1] - y, points[i + 1][0] - x)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z + self.__path_height
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            path.poses.append(pose)
        return path

    # ------------------------------------------------------------------ RViz

    def __refresh_drivable_cells(self):
        """Keep the drivable cells in RViz up to date while the map grows."""
        if self.__cloud is not None and self.__drivable_pub.get_subscription_count() > 0:
            self.__surface()

    def __publish_drivable_cells(self):
        planner = self.__planner
        cells = np.column_stack([planner.positions, planner.clearances]).astype(np.float32)
        header = Header(frame_id=self.__map_frame, stamp=self.get_clock().now().to_msg())
        self.__drivable_pub.publish(point_cloud2.create_cloud(header, DRIVABLE_FIELDS, cells))


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
