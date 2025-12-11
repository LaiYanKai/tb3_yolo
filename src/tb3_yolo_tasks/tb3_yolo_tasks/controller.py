from math import hypot, atan2, inf, cos, sin, nan, isnan, pi

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class Controller(Node):

    def __init__(self, node_name="controller"):
        # Node Constructor =============================================================
        super().__init__(node_name)

        # Lookahead subscriber
        self._sub_lookahead = self.create_subscription(PoseStamped, "lookahead", self._callback_sub_lookahead, 2)

        # Tf2
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # cmd vel publisher
        self._pub_cmd_vel = self.create_publisher(TwistStamped, "cmd_vel", 2)

        # controller
        self._timer = self.create_timer(0.05, self.controller)

        # States
        self.lookahead_x = 0.1
        self.lookahead_y = 0.0
        self.final_ang = 0.0
        self.rbt_x = 0.0
        self.rbt_y = 0.0
        self.rbt_ang = 0.0

        # constants
        self.ANGLE_THRESHOLD = pi / 180.0 * 15.0 # if the robot is facing the lookahead point (within this angle threshold), the robot will begin to move forward towards it.
        self.DISTANCE_LO_THRESHOLD = 0.02 # this distance should be smaller or equal to DISTANCE_THRESHOLD in behavior.
        self.DISTANCE_HI_THRESHOLD = 0.10 # this distance must be smaller than the lookahead distance in behavior (see _get_lookahead_point()).
        self.FINAL_ANGLE_THRESHOLD = pi / 180.0 * 2.0 # this distance should be the same as the the FINAL_ANGLE_THRESHOLD in behavior.

        # initialize the distance threshold
        self.distance_threshold = self.DISTANCE_LO_THRESHOLD


    def controller(self):
        '''
        The motion controller. Three general states:
        1. If the robot is not facing the lookahead point, 
            rotate towards the point until it is roughly facing the point 
            (determined by ANGLE_THRESHOLD). Control only the angular velocity.
        2. If the robot is roughly facing the lookahead point, move towards it. 
            Control the linear and angular velocities.
        3. If the robot is very close to the lookahead point 
            (i.e. reaches the goal point), rotate towards the final angle. 
            Control the angular velocity.
        '''

        # Get the smoothly interpolated robot frame (and thus its position and heading)
        try:
            t = self._tf_buffer.lookup_transform('map','base_link', rclpy.time.Time())
        except:
            self.get_logger().info("Waiting for tf")
            self.move(0.1, 0) # apply a small motion so that amcl starts publishing
            return
        self.rbt_x = t.transform.translation.x
        self.rbt_y = t.transform.translation.y
        q = t.transform.rotation
        sy_cp = 2.0 * (q.w * q.z + q.x * q.y) # x and y are zero
        cy_cp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z) # y is zero
        self.rbt_ang = atan2(sy_cp, cy_cp)

        # Get angular error: the difference in angle between the robot and the lookahead
        dx = self.lookahead_x - self.rbt_x
        dy = self.lookahead_y - self.rbt_y
        ang_err = ang_diff(atan2(dy, dx), self.rbt_ang)

        # Get distance error: the distance from the robot to the lookahead
        distance_err = hypot(dx, dy)

        if distance_err > self.distance_threshold:
            self.distance_threshold = self.DISTANCE_LO_THRESHOLD

            ang_vel = 1.0 * ang_err
            ang_vel = clamp(ang_vel, -1.0, 1.0)

            lin_vel = 1.0 * distance_err
            lin_vel = clamp(lin_vel, -0.22, 0.22)

            # Adjust lin vel if robot is not facing lookahead point. Improve this with a smooth function!
            if abs(ang_err) > self.ANGLE_THRESHOLD:
                lin_vel = 0

        else: # Robot is very close (distance wise) to the goal point.
            # Temporarily set the distance threshold higher (hysteresis) so it doesn't trigger the previous if statement if it momentarily moves out of the preivous distance threshold
            self.distance_threshold = self.DISTANCE_HI_THRESHOLD

            # Rotate towards the final_ang (the angle specified by the lookahead) if close to the lookahead
            final_ang_err = ang_diff(self.final_ang, self.rbt_ang)
            ang_vel = 1.0 * final_ang_err
            ang_vel = clamp(ang_vel, -1.0, 1.0)

            lin_vel = 0

        self.move(lin_vel, ang_vel)

    def move(self, lin_vel: float, ang_vel: float):
        '''
        Moves the robot at a linear velocity and angular velocity
        
        :param lin_vel: Meters per second. Positive to move forward, negative to move back.
        :type lin_vel: float
        :param ang_vel: Radians per second. Positive to rotate left, negative to rotate right.
        :type ang_vel: float
        '''
        msg_cmd_vel = TwistStamped()
        msg_cmd_vel.header.stamp = self.get_clock().now().to_msg()
        msg_cmd_vel.twist.linear.x = float(lin_vel)
        msg_cmd_vel.twist.angular.z = float(ang_vel)
        self._pub_cmd_vel.publish(msg_cmd_vel)

    def _callback_sub_lookahead(self, msg: PoseStamped):
        '''
        Copies the lookahead coordinates and the final angle to the controller everytime they are received from behavior.
        
        :param msg: The received message.
        :type msg: PoseStamped
        '''
        self.lookahead_x = msg.pose.position.x
        self.lookahead_y = msg.pose.position.y

        q = msg.pose.orientation
        sy_cp = 2.0 * q.w * q.z # x and y are zero
        cy_cp = 1.0 - 2.0 * q.z * q.z # y is zero
        self.final_ang = atan2(sy_cp, cy_cp)


def clamp(value: float | int, low: float | int, high: float | int):
    '''
    This function can be used to constrain `value` to be within `low` and `high` inclusive.
    If `value` is smaller than `low`, `low` is returned.
    If `value` is larger than `high`, `high` is returned.
    Otherwise, `value` is returned.

    :example:
    ```
    constrained_value = clamp(0.5, -0.3, 0.4) # 0.4 is returned.
    ```
    
    :param value: The value to constrain.
    :type value: float | int
    :param low: The lower bound, inclusive.
    :type low: float | int
    :param high: The higher bound, inclusive.
    :type high: float | int
    '''
    return max(min(value, high), low)

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
    rclpy.spin(Controller())
    rclpy.shutdown()


if __name__ == "__main__":
    main()