from math import hypot, cos, sin, atan2, pi, inf
from enum import Enum
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
from rclpy.time import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int64MultiArray
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class State(Enum):
    END = 0
    GO_TO_SIGN_1 = 1,
    GO_TO_SIGN_2 = 2,
    GO_TO_SIGN_3 = 3,
    GO_TO_END = 4,

    CAPTURE = 10,
    GO_TO_NEXT_SIGN = 11,

    LEFT = 20
    RIGHT = 21

    SPIN_LEFT_A = 31,
    SPIN_LEFT_B = 32,
    SPIN_RIGHT_A = 33,
    SPIN_RIGHT_B = 34,

class Behavior(Node):

    def __init__(self, node_name="behavior"):
        # Node Constructor =============================================================
        super().__init__(node_name)

        # Parameters (hardcoded)
        self._plan_period = Duration(seconds=0.5)

        # Tf2
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Path Subscriber
        self._sub_path = self.create_subscription(Path, "path", self._callback_sub_path, 2)

        # YOLO Result Subscriber
        self._sub_yolo_result = self.create_subscription(Int64MultiArray, "yolo/result", self._callback_yolo_result, 2)

        # Path request publisher
        self._pub_path_request = self.create_publisher(Path, "path_request", 2)

        # Publish lookahead
        self._pub_lookahead = self.create_publisher(PoseStamped, "lookahead", 2)

        # Handles: Timers
        self._timer = self.create_timer(0.05, self.finite_state_machine)

        # Other Instance Variables
        self._last_plan_time = self.get_clock().now()
        self._path = []
        
        self.rbt_x = 0.0
        self.rbt_y = 0.0
        self.rbt_ang = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.final_ang = 0.0
        self.state = None
        self.at_sign = 0
        
        # constants
        self.FINAL_ANGLE_THRESHOLD = np.pi / 180.0 * 2.0 # 2 degree
        self.DISTANCE_THRESHOLD = 0.05 # 5 cm
        
        # Transition to the first sign
        self.transition_state_to(State.GO_TO_SIGN_1)


    def transition_state_to(self, new_state: State):
        '''
        Called every time a state completes and is ready to transition to the next state. 
        The goal point for the next state should be tuned in this function.
        
        :param new_state: The state to transition to.
        :type new_state: State
        '''
        # overwrite the current state
        self.state = new_state
        self.get_logger().info(f"Switching to [{self.state.name}]")

        # set the goal point before transition to the new state
        if new_state == State.GO_TO_SIGN_1:
            self.goal_x = 1.25
            self.goal_y = -0.15
            self.final_ang = - pi / 180.0 * 2.0
        elif new_state == State.GO_TO_SIGN_2:
            self.goal_x = 0.3
            self.goal_y = -0.9
            self.final_ang = -pi / 180 * 175
        elif new_state == State.GO_TO_SIGN_3:
            self.goal_x = 1.05
            self.goal_y = -0.45
            self.final_ang = pi/2
        elif new_state == State.GO_TO_END:
            self.goal_x = 1.4
            self.goal_y = -0.9
            self.final_ang = 0.0
        elif new_state == State.LEFT:
            self.goal_x = self.rbt_x
            self.goal_y = self.rbt_y
            self.final_ang = self.rbt_ang + pi
        elif new_state == State.RIGHT:
            self.goal_x = self.rbt_x
            self.goal_y = self.rbt_y
            self.final_ang = self.rbt_ang - pi
        elif new_state == State.SPIN_LEFT_A:
            self.goal_x = self.rbt_x
            self.goal_y = self.rbt_y
            self.final_ang = self.rbt_ang - pi * 0.75 # 135 deg right
        elif new_state == State.SPIN_LEFT_B:
            self.goal_x = self.rbt_x
            self.goal_y = self.rbt_y
            self.final_ang = self.rbt_ang - pi * 0.75 # 135 deg right
        elif new_state == State.SPIN_RIGHT_A:
            self.goal_x = self.rbt_x
            self.goal_y = self.rbt_y
            self.final_ang = self.rbt_ang + pi * 0.75 # 135 deg right
        elif new_state == State.SPIN_RIGHT_B:
            self.goal_x = self.rbt_x
            self.goal_y = self.rbt_y
            self.final_ang = self.rbt_ang + pi * 0.75 # 135 deg right
        elif new_state == State.CAPTURE:
            pass
        elif new_state == State.GO_TO_NEXT_SIGN:
            pass
        elif new_state == State.END:
            pass


    def finite_state_machine(self):
        '''
        The callback implementing the state machine. The callback should complete quickly and within 0.05s.
        The callback checks if the terminating condition for each state is reached, before calling `transition_state_to()` to transition to the next state.
        '''

        # Get the smoothly interpolated robot frame (and thus its position and heading)
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except:
            self.get_logger().info("Waiting for robot tf to become available...")
            return
        self.rbt_x = t.transform.translation.x
        self.rbt_y = t.transform.translation.y
        q = t.transform.rotation
        sy_cp = 2.0 * (q.w * q.z + q.x * q.y) # x and y are zero
        cy_cp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z) # y is zero
        self.rbt_ang = atan2(sy_cp, cy_cp)
        
        # Request a path at regular intervals (by default, a plan is requested every 1s)
        self._request_plan(self.rbt_x, self.rbt_y, self.goal_x, self.goal_y)
        
        # Gets the lookahead point.
        lookahead_x, lookahead_y = self._get_lookahead_point(0.1)
        self._send_lookahead_to_controller(lookahead_x, lookahead_y, self.final_ang)

        if self.state == State.GO_TO_SIGN_1:
            self.get_logger().info(f"[{self.state.name}] Waiting to reach Goal (and do final turn).")
            if self.rbt_close_to_goal(self.DISTANCE_THRESHOLD) and self.rbt_ang_close_to_final_ang(self.FINAL_ANGLE_THRESHOLD):
                self.at_sign = 1
                self.transition_state_to(State.CAPTURE)
        
        elif self.state == State.GO_TO_SIGN_2:
            self.get_logger().info(f"[{self.state.name}] Waiting to reach Goal (and do final turn).")
            if self.rbt_close_to_goal(self.DISTANCE_THRESHOLD) and self.rbt_ang_close_to_final_ang(self.FINAL_ANGLE_THRESHOLD):
                self.at_sign = 2
                self.transition_state_to(State.CAPTURE)

        elif self.state == State.GO_TO_SIGN_3:
            self.get_logger().info(f"[{self.state.name}] Waiting to reach Goal (and do final turn).")
            if self.rbt_close_to_goal(self.DISTANCE_THRESHOLD) and self.rbt_ang_close_to_final_ang(self.FINAL_ANGLE_THRESHOLD):
                self.at_sign = 3
                self.transition_state_to(State.CAPTURE)

        elif self.state == State.GO_TO_END:
            self.get_logger().info(f"[{self.state.name}] Waiting to reach Goal (and do final turn).")
            if self.rbt_close_to_goal(self.DISTANCE_THRESHOLD) and self.rbt_ang_close_to_final_ang(self.FINAL_ANGLE_THRESHOLD):
                self.transition_state_to(State.END)

        elif self.state == State.CAPTURE:
            if self.image_class == -1: # no class seen
                self.get_logger().info(f"Saw Image Class={self.image_class}")
                self.transition_state_to(State.GO_TO_NEXT_SIGN)
            elif self.image_class == 0: # class 0 seen
                self.get_logger().info(f"Saw Image Class={self.image_class}")
                self.transition_state_to(State.SPIN_LEFT_A)
            elif self.image_class == 1: # class 1 seen
                self.get_logger().info(f"Saw Image Class={self.image_class}")
                self.transition_state_to(State.SPIN_RIGHT_A)
            elif self.image_class == 2: # qr code seen
                self.get_logger().info(f"Saw QR with message={self.qr_message}")
                if self.qr_message == "spin left":
                    self.transition_state_to(State.SPIN_LEFT_A)
                elif self.qr_message == "spin right":
                    self.transition_state_to(State.SPIN_RIGHT_A)
                else: # no message? go to the next sign
                    self.transition_state_to(State.GO_TO_NEXT_SIGN)
        
        elif self.state == State.GO_TO_NEXT_SIGN:
            self.get_logger().info(f"[{self.state.name}] Switching state...")
            if self.at_sign == 1:
                self.transition_state_to(State.GO_TO_SIGN_2)
            elif self.at_sign == 2:
                self.transition_state_to(State.GO_TO_SIGN_3)
            else:
                self.transition_state_to(State.GO_TO_END)

        elif self.state == State.LEFT:
            self.get_logger().info(f"[{self.state.name}] Performing action.")
            if self.rbt_ang_close_to_final_ang(self.FINAL_ANGLE_THRESHOLD):
                self.transition_state_to(State.GO_TO_NEXT_SIGN)

        elif self.state == State.RIGHT:
            self.get_logger().info(f"[{self.state.name}] Performing action.")
            if self.rbt_ang_close_to_final_ang(self.FINAL_ANGLE_THRESHOLD):
                self.transition_state_to(State.GO_TO_NEXT_SIGN)

        elif self.state == State.SPIN_LEFT_A:
            self.get_logger().info(f"[{self.state.name}] Performing action.")
            if self.rbt_ang_close_to_final_ang(self.FINAL_ANGLE_THRESHOLD):
                self.transition_state_to(State.SPIN_LEFT_B)

        elif self.state == State.SPIN_LEFT_B:
            self.get_logger().info(f"[{self.state.name}] Performing action.")
            if self.rbt_ang_close_to_final_ang(self.FINAL_ANGLE_THRESHOLD):
                self.transition_state_to(State.GO_TO_NEXT_SIGN)

        elif self.state == State.SPIN_RIGHT_A:
            self.get_logger().info(f"[{self.state.name}] Performing action.")
            if self.rbt_ang_close_to_final_ang(self.FINAL_ANGLE_THRESHOLD):
                self.transition_state_to(State.SPIN_RIGHT_B)

        elif self.state == State.SPIN_RIGHT_B:
            self.get_logger().info(f"[{self.state.name}] Performing action.")
            if self.rbt_ang_close_to_final_ang(self.FINAL_ANGLE_THRESHOLD):
                self.transition_state_to(State.GO_TO_NEXT_SIGN)

        elif self.state == State.END:
            self.get_logger().info("COMPLETE!!!")
            self.destroy_timer(self._timer)

        else:
            self.get_logger().warn("STATE IS NOT CODED! NOTHING DONE!")
            
    def rbt_close_to_goal(self, threshold=0.05):
        '''
        Returns true if the robot is at the goal point.
        
        :param threshold: If the robot is within this threshold distance (meters) from the goal point, the robot is considered to be on the goal point.
        '''
        return hypot(self.goal_x - self.rbt_x, self.goal_y - self.rbt_y) < threshold

    def rbt_ang_close_to_final_ang(self, threshold=0.034):
        '''
        Returns true if the robot is facing the same direction as the final goal angle.
        
        :param threshold: If the robot's heading is within a threshold angle (radians) from the final goal angle, the robot is considered to be facing the same direction.
        '''
        return abs(ang_diff(self.final_ang, self.rbt_ang)) < threshold
    
    def _get_lookahead_point(self, lookahead_distance: float):
        '''
        Gets the lookahead point that is a lookahead distance away from the robot. This point is in the direction of the goal along the path, from the closest point to the robot.
        If the goal (last point on the path) is closer than the lookahead distance, the goal point is returned.
        If the path has not yet been received, the robot's coordinates are returned.
        
        :param lookahead_distance: The lookahead distance in meters.
        :type lookahead_distance: float
        '''
        if not self._path:
            return (self.rbt_x, self.rbt_y)
        
        # Get closest
        min_dist = inf
        min_idx = len(self._path) - 1
        for i, pose in enumerate(self._path):
            dx = pose.pose.position.x - self.rbt_x
            dy = pose.pose.position.y - self.rbt_y
            dist = hypot(dx, dy)
            if min_dist > dist:
                min_dist = dist
                min_idx = i

        # Get lookahead
        for i in range(min_idx, len(self._path)):
            pos = self._path[i].pose.position
            dx = pos.x - self.rbt_x
            dy = pos.y - self.rbt_y
            dist = hypot(dx, dy)
            if dist > lookahead_distance:
                return (pos.x, pos.y)
            
        goal_pose = self._path[-1]
        return (goal_pose.pose.position.x, goal_pose.pose.position.y)

    def _request_plan(self, rbt_x: float, rbt_y: float, goal_x: float, goal_y: float):
        '''
        Requests for a path at regular intervals, determined by self._plan_period. The path found accounts for inflation zones.
        
        :param rbt_x: The x-coordinate (meters) of the starting point in the world frame.
        :type rbt_x: float
        :param rbt_y: The y-coordinate (meters) of the starting point in the world frame.
        :type rbt_y: float
        :param goal_x: The x-coordinate (meters) of the goal point in the world frame.
        :type goal_x: float
        :param goal_y: The y-coordinate (meters) of the goal point in the world frame.
        :type goal_y: float
        '''
        # silently return if none of the coords are received from the subscribers
        
        # return if not time to replan
        if self.get_clock().now() - self._last_plan_time < self._plan_period:
            return
        
        # set next time to plan
        while self._last_plan_time < self.get_clock().now():
            self._last_plan_time += self._plan_period

        # create a new message for publishing
        msg_path_request = Path()
        msg_path_request.header.stamp = self.get_clock().now().to_msg()
        msg_path_request.header.frame_id = "map"
        rbt_pose = PoseStamped()
        rbt_pose.pose.position.x = rbt_x
        rbt_pose.pose.position.y = rbt_y
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        msg_path_request.poses = [rbt_pose, goal_pose]

        # publish the message
        # self.get_logger().info(f"Sending Path Planning Request from Rbt @ ({rbt_x:7.3f}, {rbt_y:7.3f}) to Goal @ ({goal_x:7.3f}, {goal_y:7.3f})")
        self._pub_path_request.publish(msg_path_request)

    def _send_lookahead_to_controller(self, lookahead_x: float, lookahead_y: float, final_ang: float):
        '''
        Publishes the lookahead point and the a "final" angle for the robot to turn to.
        If the robot is able to reach the coordinates before another lookahead is published, the controller will turn the robot towards the final angle.
        
        :param lookahead_x: The x-coordinate (meters) of the lookahead point in the world frame.
        :type lookahead_x: float
        :param lookahead_y: The y-coordinate (meters) of the lookahead point in the world frame.
        :type lookahead_y: float
        :param final_ang: The final angle (radians) to face in the world frame
        :type final_ang: float
        '''
        lookahead_pose = PoseStamped()
        lookahead_pose.header.frame_id = "map"
        lookahead_pose.header.stamp = self.get_clock().now().to_msg()
        lookahead_pose.pose.position.x = lookahead_x
        lookahead_pose.pose.position.y = lookahead_y
        final_ang = (final_ang + pi) % (2*pi) - pi
        q = lookahead_pose.pose.orientation
        q.w = cos(final_ang * 0.5)
        q.z = sin(final_ang * 0.5)
        self._pub_lookahead.publish(lookahead_pose)

    def _callback_sub_path(self, msg: Path):
        '''
        Subscriber callback that is called everytime a path is received from the path planner.
        
        :param msg: Description
        :type msg: Path
        '''
        self._path = msg.poses

    def _callback_yolo_result(self, msg: Int64MultiArray):
        '''
        Subscriber callback for YOLO sign classes and QR messages.
        '''

        self.image_class = msg.data[0]
        self.qr_message = ""
        if msg.data[1] == 0:
            self.qr_message = "spin left"
        elif msg.data[1] == 1:
            self.qr_message = "spin right"
        elif msg.data[1] == 2:
            self.qr_message = "dance left"
        elif msg.data[1] == 3:
            self.qr_message = "dance right"


def ang_diff(final_angle: float, initial_angle: float):
    '''
    Calculates the angular difference (radians) between the two angles. It is given by `final_angle - initial_angle`, and with additional processing to ensure that the difference never exceeds -pi and pi radians.
    This has the effect of finding the shortest angular difference between the two angles.
    **This function should be used every time angles are subtracted**.

    :param final_angle: An angle.
    :param initial_angle: The angle to subtract from `final_angle`.
    '''
    diff = float(final_angle) - float(initial_angle)
    return (diff + pi) % (2*pi) - pi

# Main Boiler Plate =============================================================
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Behavior())
    rclpy.shutdown()


if __name__ == "__main__":
    main()