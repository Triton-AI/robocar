import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from f110_msgs.msg import Wpnt, WpntArray

def hypot(x, y):
    return np.sqrt(x**2 + y**2)

def circleSegmentIntersection(start, end, radius):
    start_np = np.array([start.x, start.y])
    end_np = np.array([end.x, end.y])
    d = end_np - start_np
    f = start_np

    a = np.dot(d, d)
    b = 2 * np.dot(f, d)
    c = np.dot(f, f) - radius**2

    discriminant = b**2 - 4 * a * c

    if discriminant < 0:
        return None
    else:
        discriminant = np.sqrt(discriminant)
        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)

        t = t1 if 0 <= t1 <= 1 else t2
        intersection = start_np + t * d

        intersection_point = Point()
        intersection_point.x = intersection[0]
        intersection_point.y = intersection[1]
        return intersection_point

class WaypointDetection(Node):

    def __init__(self):
        super().__init__('Waypoint_Detection_Node', allow_undeclared_parameters=True)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_frame', 'map'),
                ('lookahead_radius_m', 1.0),
                ('pose_topic', '/amcl_pose'),
                ('drive_topic', '/ego_racecar/drive'),
                ('goal_topic', 'goal_pose'),
                ('shortest_path_enable', False),
                ('Ts', 1.0)]
        )

        # self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.lookahead_radius = self.get_parameter('lookahead_radius_m').value
        self.drive_topic = self.get_parameter('drive_topic').value

        self.track_bounds = None
        self.glb_wpnts = WpntArray()
        self.glb_wpnts_sub_ = self.create_subscription(WpntArray, '/global_waypoints', self.glb_wpnts_cb, 10)
        self.glb_wpnts_sub_

        self.glb_sp_wpnts = WpntArray()
        self.glb_sp_wpnts_sub_ = self.create_subscription(WpntArray, '/global_waypoints/shortest_path', self.glb_sp_wpnts_cb, 10)
        self.glb_sp_wpnts_sub_

        self.pose = PoseWithCovarianceStamped()
        self.pose_sub_ = self.create_subscription(PoseWithCovarianceStamped, self.get_parameter('pose_topic').value, self.pose_cb, 10)
        self.pose_sub_

        self.goal_update_pub_ = self.create_publisher(PoseStamped, self.get_parameter('goal_topic').value, 10)

        # Start a timer
        self.Ts = self.get_parameter('Ts').value
        self.timer = self.create_timer(self.Ts, self.periodic)

    def glb_wpnts_cb(self, msg):
        self.glb_wpnts = msg
        track_length = msg.wpnts[-1].s_m
        gb_len = rclpy.Parameter('track_length', rclpy.Parameter.Type.DOUBLE, track_length)
        self.set_parameters([gb_len])

    def glb_sp_wpnts_cb(self, msg):
        self.glb_sp_wpnts = msg

    def pose_cb(self, msg):
        self.pose = msg

    def periodic(self):
        # TODO: waypoint detection logic
        car_pos = np.array((self.pose.pose.pose.position.x, self.pose.pose.pose.position.y))
        self.get_logger().info(f'Car Position: {car_pos}')
        
        if self.get_parameter('shortest_path_enable').value:
            wpnts = self.glb_sp_wpnts.wpnts
        else:
            wpnts = self.glb_wpnts.wpnts
        lookahead_wpnt = self.get_lookahead_point(car_pos, self.lookahead_radius, wpnts, interpolate_after_goal=True)
        
        # TODO: include velocity constraints logic
        if lookahead_wpnt:
            goal_id = lookahead_wpnt.id
            goal_x = lookahead_wpnt.x_m
            goal_y = lookahead_wpnt.y_m
            goal_theta = lookahead_wpnt.psi_rad
            self.get_logger().info('Recieved Data:\n ID: %d \n X : %f \n Y : %f \n Theta : %f' % (goal_id, goal_x, goal_y, goal_theta))
            self.send_goal(goal_x, goal_y, goal_theta)

    def send_goal(self, x ,y, theta):
        self.get_logger().info('Sending goal to GoalUpdater')
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.get_parameter('map_frame').value
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        quat = quaternion_from_euler(0.0, 0.0, theta)
        goal_pose.pose.orientation.x = quat[0]
        goal_pose.pose.orientation.y = quat[1]
        goal_pose.pose.orientation.z = quat[2]
        goal_pose.pose.orientation.w = quat[3]
        
        self.goal_update_pub_.publish(goal_pose)
        
        self.get_logger().info('Goal sent')

    def transform_waypoints(self, car_pos_, wpnts):
        closest_wpnt_index = np.argmin([hypot(wpnt.x_m - car_pos_[0], wpnt.y_m - car_pos_[1]) for wpnt in wpnts])
        transformed_wpnts = []
        for wpnt in wpnts[closest_wpnt_index:]:
            transformed_wpnt = Wpnt()
            transformed_wpnt.x_m = wpnt.x_m - car_pos_[0]
            transformed_wpnt.y_m = wpnt.y_m - car_pos_[1]
            transformed_wpnt.psi_rad = wpnt.psi_rad
            transformed_wpnt.id = wpnt.id
            transformed_wpnts.append(transformed_wpnt)
        return transformed_wpnts

    def get_lookahead_point(self, car_pos_, lookahead_dist, global_wpnts, interpolate_after_goal):
        self.get_logger().info(f'Looking for waypoint with lookahead distance: {lookahead_dist}')
        transformed_path_wpnts = self.transform_waypoints(car_pos_, global_wpnts)
        
        goal_wpnt_it = next(
            (wpnt for wpnt in transformed_path_wpnts if hypot(wpnt.x_m, wpnt.y_m) >= lookahead_dist), 
            None
        )

        if goal_wpnt_it is None:
            self.get_logger().info('No waypoint found within lookahead distance')
            if interpolate_after_goal:
                last_wpnt_it = transformed_path_wpnts[-1]
                prev_last_wpnt_it = transformed_path_wpnts[-2]

                end_path_orientation = np.arctan2(
                    last_wpnt_it.y_m - prev_last_wpnt_it.y_m,
                    last_wpnt_it.x_m - prev_last_wpnt_it.x_m
                )

                last_wpnt_position = Point()
                last_wpnt_position.x = last_wpnt_it.x_m
                last_wpnt_position.y = last_wpnt_it.y_m

                projected_position = Point()
                projected_position.x = last_wpnt_position.x + np.cos(end_path_orientation) * lookahead_dist
                projected_position.y = last_wpnt_position.y + np.sin(end_path_orientation) * lookahead_dist

                interpolated_position = circleSegmentIntersection(
                    last_wpnt_position, projected_position, lookahead_dist
                )

                if interpolated_position is None:
                    self.get_logger().warn('Interpolation failed, using last waypoint')
                    last_wpnt_it.x_m += car_pos_[0]
                    last_wpnt_it.y_m += car_pos_[1]
                    return last_wpnt_it

                interpolated_wpnt = Wpnt()
                interpolated_wpnt.x_m = interpolated_position.x + car_pos_[0]
                interpolated_wpnt.y_m = interpolated_position.y + car_pos_[1]
                interpolated_wpnt.psi_rad = last_wpnt_it.psi_rad
                interpolated_wpnt.id = last_wpnt_it.id
                self.get_logger().info(f'Interpolated waypoint: {interpolated_wpnt.x_m}, {interpolated_wpnt.y_m}')
                return interpolated_wpnt
            else:
                goal_wpnt_it = transformed_path_wpnts[-1]
        else:
            goal_wpnt_index = transformed_path_wpnts.index(goal_wpnt_it)
            self.get_logger().info(f'Found waypoint within lookahead distance at index: {goal_wpnt_index}')
            if goal_wpnt_index != 0:
                prev_wpnt_it = transformed_path_wpnts[goal_wpnt_index - 1]
                prev_wpnt_position = Point()
                prev_wpnt_position.x = prev_wpnt_it.x_m
                prev_wpnt_position.y = prev_wpnt_it.y_m
                goal_wpnt_position = Point()
                goal_wpnt_position.x = goal_wpnt_it.x_m
                goal_wpnt_position.y = goal_wpnt_it.y_m

                point = circleSegmentIntersection(
                    prev_wpnt_position, goal_wpnt_position, lookahead_dist
                )

                if point is None:
                    self.get_logger().warn('Circle-segment intersection failed, using goal waypoint')
                    goal_wpnt_it.x_m += car_pos_[0]
                    goal_wpnt_it.y_m += car_pos_[1]
                    return goal_wpnt_it

                wpnt = Wpnt()
                wpnt.x_m = point.x + car_pos_[0]
                wpnt.y_m = point.y + car_pos_[1]
                wpnt.psi_rad = goal_wpnt_it.psi_rad
                wpnt.id = goal_wpnt_it.id
                self.get_logger().info(f'Computed waypoint: {wpnt.x_m}, {wpnt.y_m}')
                return wpnt
            
        goal_wpnt_it.x_m += car_pos_[0]
        goal_wpnt_it.y_m += car_pos_[1]
        self.get_logger().info(f'Goal waypoint: {goal_wpnt_it.x_m}, {goal_wpnt_it.y_m}')
        return goal_wpnt_it
        
def main(args=None):
    rclpy.init(args=args)

    node = WaypointDetection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()