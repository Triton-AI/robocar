import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from f110_msgs.msg import Wpnt, WpntArray

class WaypointDetection(Node):

    def __init__(self):
        super().__init__('Waypoint_Detection_Node', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_frame', 'map'),
                ('search_radius_m', 1.0),
                ('pose_topic', '/amcl_pose'),
                ('drive_topic', '/ego_racecar/drive'),
                ('goal_topic', 'goal_pose'),
                ('Ts', 1.0)]
        )

        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.search_radius = self.get_parameter('search_radius_m').value
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
        wpnts = self.glb_wpnts.wpnts
        dist_to_wpnts = []
        car_point = np.array((self.pose.pose.pose.position.x, self.pose.pose.pose.position.y))
        for wpnt in wpnts:
            waypoint = np.array((wpnt.x_m, wpnt.y_m))
            dist_to_wpnts.append(np.abs(np.linalg.norm(waypoint - car_point)))
        closest_wpnt = wpnts[np.argmin(np.array(dist_to_wpnts))]

        # TODO: include velocity constraints logic
        
        goal_id = closest_wpnt.id
        goal_x = closest_wpnt.x_m
        goal_y = closest_wpnt.y_m
        goal_theta = closest_wpnt.psi_rad
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

        # self.get_logger().info('waiting for action server')
        # self._action_client.wait_for_server()
        # self.get_logger().info('action server detected')

        # self._send_goal_future = self._action_client.send_goal_async(
        #     goal_pose,
        #     feedback_callback=self.feedback_callback)
        
        self.goal_update_pub_.publish(goal_pose)
        
        self.get_logger().info('goal sent')
        
    # def goal_response_callback(self, future):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().info('Goal rejected :(')
    #         return
    #     self.get_logger().info('Goal accepted :)')
    #     self._get_result_future = goal_handle.get_result_async()

    #     self._get_result_future.add_done_callback(self.get_result_callback)

    # def get_result_callback(self, future):
    #     result = future.result().result
    #     self.get_logger().info('Result: {0}' + str(result))
    #     rclpy.shutdown()

    # def feedback_callback(self, feedback_msg):
    #     feedback = feedback_msg.feedback
    #     self.get_logger().info('FEEDBACK:' + str(feedback) )

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