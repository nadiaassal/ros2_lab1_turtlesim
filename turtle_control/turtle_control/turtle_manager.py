import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen
from std_srvs.srv import Empty

from turtle_interfaces.msg import TargetPose
from turtle_interfaces.srv import SetSpeedLimits
from turtle_interfaces.action import GoToPose, DrawSquare

class TurtleManagerNode(Node):
    def __init__(self):
        super().__init__('turtle_manager')

        # 1. Parameters (Speed Limits)
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)

        # Subscribers and Publishers
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.current_pose = None

        # 2. Speed Service
        self.speed_srv = self.create_service(
            SetSpeedLimits, 'set_speed_limits', self.set_speed_limits_callback)

        # 3. Turtlesim internal service clients
        self.clear_cli = self.create_client(Empty, '/clear')
        self.teleport_cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.pen_cli = self.create_client(SetPen, '/turtle1/set_pen')

        # Custom Reset Service
        self.reset_srv = self.create_service(Empty, 'reset_turtle', self.reset_callback)

        # 4. Action Server: goToPose
        self.goto_action_server = ActionServer(
            self, GoToPose, 'go_to_pose', self.execute_goto_callback)

        # 5. Action Server & Client for Square
        self.square_action_server = ActionServer(
            self, DrawSquare, 'draw_square', self.execute_square_callback)
        self.goto_action_client = ActionClient(self, GoToPose, 'go_to_pose')

        self.get_logger().info('Turtle Manager Node has been started.')

    def pose_callback(self, msg):
        self.current_pose = msg

    def set_speed_limits_callback(self, request, response):
        if request.max_linear_speed <= 0.0 or request.max_angular_speed <= 0.0:
            response.success = False
            response.message = "Speed limits must be strictly positive!"
            return response

        self.set_parameters([
            rclpy.parameter.Parameter('max_linear_speed', rclpy.Parameter.Type.DOUBLE, request.max_linear_speed),
            rclpy.parameter.Parameter('max_angular_speed', rclpy.Parameter.Type.DOUBLE, request.max_angular_speed)
        ])
        response.success = True
        response.message = f"Limits updated: linear={request.max_linear_speed}, angular={request.max_angular_speed}"
        return response

    def reset_callback(self, request, response):
        # 1. Turn pen off
        if self.pen_cli.wait_for_service(timeout_sec=2.0):
            pen_req = SetPen.Request()
            pen_req.off = 1
            self.pen_cli.call_async(pen_req)

        # 2. Teleport to center (5.54, 5.54, 0.0)
        if self.teleport_cli.wait_for_service(timeout_sec=2.0):
            tp_req = TeleportAbsolute.Request()
            tp_req.x = 5.544445
            tp_req.y = 5.544445
            tp_req.theta = 0.0
            self.teleport_cli.call_async(tp_req)

        # 3. Clear lines
        if self.clear_cli.wait_for_service(timeout_sec=2.0):
            self.clear_cli.call_async(Empty.Request())

        time.sleep(0.1)

        # 4. Turn pen back on
        if self.pen_cli.wait_for_service(timeout_sec=2.0):
            pen_req = SetPen.Request()
            pen_req.off = 0
            self.pen_cli.call_async(pen_req)

        return response

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def execute_goto_callback(self, goal_handle):
        target = goal_handle.request.target
        feedback_msg = GoToPose.Feedback()
        rate = 0.02

        while self.current_pose is None:
            time.sleep(0.01)

        max_ang = self.get_parameter('max_angular_speed').value
        max_lin = self.get_parameter('max_linear_speed').value

        dx = target.x - self.current_pose.x
        dy = target.y - self.current_pose.y
        dist = math.hypot(dx, dy)

        # Step 1: Rotate in-place towards waypoint
        if dist > 0.05:
            target_angle = math.atan2(dy, dx)
            while True:
                max_ang = self.get_parameter('max_angular_speed').value
                angle_error = self.normalize_angle(target_angle - self.current_pose.theta)
                if abs(angle_error) < 0.02:
                    break

                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = max(-max_ang, min(max_ang, 2.5 * angle_error))
                self.cmd_vel_pub.publish(twist)
                time.sleep(rate)

            # Momentary stop
            self.cmd_vel_pub.publish(Twist())
            time.sleep(0.05)

        # Step 2: Drive straight along line (Pure translation with small orientation correction)
        while True:
            max_lin = self.get_parameter('max_linear_speed').value
            max_ang = self.get_parameter('max_angular_speed').value

            dx = target.x - self.current_pose.x
            dy = target.y - self.current_pose.y
            dist = math.hypot(dx, dy)

            feedback_msg.distance_to_goal = dist
            goal_handle.publish_feedback(feedback_msg)

            if dist < 0.04:
                break

            target_angle = math.atan2(dy, dx)
            angle_error = self.normalize_angle(target_angle - self.current_pose.theta)

            twist = Twist()
            # If drifted, correct orientation while driving slowly
            twist.linear.x = max(0.1, min(max_lin, 1.8 * dist))
            twist.angular.z = max(-max_ang, min(max_ang, 2.0 * angle_error))
            self.cmd_vel_pub.publish(twist)
            time.sleep(rate)

        # Stop at corner
        self.cmd_vel_pub.publish(Twist())
        time.sleep(0.05)

        # Step 3: Align to final requested yaw
        while True:
            max_ang = self.get_parameter('max_angular_speed').value
            angle_error = self.normalize_angle(target.yaw - self.current_pose.theta)

            if abs(angle_error) < 0.02:
                break

            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = max(-max_ang, min(max_ang, 2.5 * angle_error))
            self.cmd_vel_pub.publish(twist)
            time.sleep(rate)

        # Full Stop
        self.cmd_vel_pub.publish(Twist())

        goal_handle.succeed()
        result = GoToPose.Result()
        result.success = True
        result.final_distance = 0.0
        return result

    def execute_square_callback(self, goal_handle):
        side = goal_handle.request.side_length
        if side <= 0.0:
            side = 3.0

        # 1. Reset first to clean board
        self.reset_callback(Empty.Request(), Empty.Response())
        time.sleep(0.5)

        # Center start position
        x0, y0 = 5.544445, 5.544445

        # Precise 4 corners with corresponding heading:
        # Corner 1: Walk East -> face North (1.5708)
        # Corner 2: Walk North -> face West (3.14159)
        # Corner 3: Walk West -> face South (-1.5708)
        # Corner 4: Walk South -> face East (0.0)
        corners = [
            (x0 + side, y0, 1.5708),
            (x0 + side, y0 + side, 3.14159),
            (x0, y0 + side, -1.5708),
            (x0, y0, 0.0)
        ]

        feedback = DrawSquare.Feedback()

        for idx, (cx, cy, cyaw) in enumerate(corners):
            feedback.current_side = idx + 1
            goal_handle.publish_feedback(feedback)

            g = GoToPose.Goal()
            g.target.x = cx
            g.target.y = cy
            g.target.yaw = cyaw

            self.goto_action_client.wait_for_server()
            future = self.goto_action_client.send_goal_async(g)

            while not future.done():
                time.sleep(0.02)

            sub_handle = future.result()
            res_future = sub_handle.get_result_async()
            while not res_future.done():
                time.sleep(0.02)

        goal_handle.succeed()
        result = DrawSquare.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    node = TurtleManagerNode()
    try:
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
