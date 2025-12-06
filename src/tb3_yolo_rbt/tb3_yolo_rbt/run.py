import os
import numpy as np
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import cv_bridge
from ultralytics import YOLO
from cv2 import QRCodeDetector

from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped

class State(Enum):
    TURN_LEFT = 0
    TURN_RIGHT = 1
    MOVE_FORWARD = 2
    FOLLOW_WALL = 10
    DETECT_SIGN = 11
    DETECT_WALLS = 12
    SPIN_LEFT = 20
    SPIN_RIGHT = 21
    DANCE_LEFT = 22
    DANCE_RIGHT = 23

class Run(Node):

    def __init__(self):
        super().__init__('run')
        
        # Handles
        self._cv_bridge = cv_bridge.CvBridge()
        self._sub_image = self.create_subscription(
            CompressedImage, 'camera/image_raw/compressed', self._callback_sub_image, 2)
        self._sub_scan = self.create_subscription(
            LaserScan, 'scan', self._callback_sub_scan, 2)
        self._sub_imu = self.create_subscription(
            Imu, 'imu', self._callback_sub_imu, 2)
        self._pub_yolo_img = self.create_publisher(
            CompressedImage, 'yolo/image/compressed', 2)
        self._pub_cmd_vel = self.create_publisher(
            TwistStamped, 'cmd_vel', 2)
        self._timer = self.create_timer(
            0.05, self.run_)
        
        # YOLO Model
        path_to_model = os.path.join(os.path.expanduser("~"), "tb3_yolo/", "best.pt")
        print(f"Loading {path_to_model}...")
        self.model_ = YOLO(path_to_model)

        # QRCode Detector (don't actually have to use YOLO)
        self.qr_ = QRCodeDetector()
        
        # Other states / variables
        self.best_class = None
        self.best_confidence = 0.0
        self.qr_message = ""
        self.scan_ranges = None
        self.gyro_heading = None
        self._target_heading = 0
        self._target_time = 0
        
        # initial state
        self.state = State.FOLLOW_WALL
        print(f"Begin run")

    def _run(self):

        # !RETURN UNTIL ALL DATA IS RECEIVED =======================
        if self.best_class is None or self.scan_ranges is None or self.gyro_heading is None:
            return
        # !=============================================================

        # Print statements to terminal
        print(f"[{self.state.name}] ---- ")
        print(f"YOLO: Best class={self.best_class}), Best confidence value={self.best_confidence}")
        print(f"Gyro: {self.gyro_heading} rad, or {self.gyro_heading / np.pi * 180} deg")

        # get lidar scan ranges
        front_right_range = self.scan_ranges[270]
        front_range = self.scan_ranges[360]
        front_left_range = self.scan_ranges[450]
        left_range = self.scan_ranges[540]
        right_range = self.scan_ranges[180]
        print(f"LIDAR: L={left_range:.2f}, FL={front_left_range:.2f}, F={front_range:.2f}, FR={front_right_range:.2f}, {right_range:.2f}")

        # state machine

        if self.state == State.TURN_LEFT:
            # Turn until the robot faces 90 deg left from the original position.
            self.move(0, 0.1)

            # The target heading must be set immediately before this state
            if self.reached_target_heading():
                self.state = State.FOLLOW_WALL

        elif self.state == State.TURN_RIGHT:
            # Turn until the robot faces 90 deg right from the original position
            self.move(0, -0.1)

            # The target heading must be set immediately before this state
            if self.reached_target_heading():
                self.state = State.FOLLOW_WALL

        elif self.state == State.MOVE_FORWARD:
            self.move(0.05, 0)

            # The target time must be set immediately before switching to this state
            if self.reached_target_time():
                self.state = State.DETECT_WALLS

        elif self.state == State.FOLLOW_WALL:
            
            if (front_right_range != np.inf): # lidar gives inf values if it cannot reliably detect the range

                # P control for angular velocity
                ang_error = 0.2 - front_right_range
                kp = 5.0
                ang_vel = kp * ang_error
                self.move(0.03, ang_vel)

            # Try to detect a sign if a front wall is detected one cell away
            if front_range < 0.4:
                self.state = State.DETECT_SIGN

        elif self.state == State.DETECT_WALLS:
            self.move(0, 0)

            left_blocked = left_range < 0.2
            right_blocked = right_range < 0.2

            if left_blocked and right_blocked:
                # DEADEND: U Turn
                self.set_target_heading_(np.pi)
                self.state = State.TURN_LEFT
            elif left_blocked and not right_blocked:
                # Turn Right
                self.set_target_heading_(0, -np.pi / 2)
                self.state = State.TURN_RIGHT
            elif not left_blocked and right_blocked:
                # Turn Left
                self.set_target_heading_(0, np.pi / 2)
                self.state = State.TURN_LEFT
            else:
                # Both free. Just turn left
                self.set_target_heading_(0, np.pi / 2)
                self.state = State.TURN_LEFT


        elif self.state == State.DETECT_SIGN:
            self.move(0, 0)

            # Try to see if there is any sign
            if self.best_class == 0: # left class
                self.state = State.TURN_LEFT
                self.set_target_heading(np.pi / 2)

            elif self.best_class == 1: # right class
                self.state = State.TURN_RIGHT
                self.set_target_heading(-np.pi / 2)

            elif self.best_class == 2: # qr class
                if self.qr_message == "spin left":
                    self.state = State.SPIN_LEFT
                elif self.qr_message == "spin right":
                    self.state = State.SPIN_RIGHT
                elif self.qr_message == "dance left":
                    self.state = State.DANCE_LEFT
                elif self.qr_message == "dance right":
                    self.state = State.DANCE_RIGHT

            else: # No sign - move forward into the cell to detect left and right walls
                self.state = State.MOVE_FORWARD

        elif self.state == State.SPIN_LEFT:
            # Spin 270 degrees to the right (clockwise from top view) until the robot is actually facing left

            "The Lady of the Lake, her arm clad in the purest shimmering samite, held aloft Excalibur from the bosom of the water," 

        elif self.state == State.SPIN_RIGHT:
            # Spin 270 degrees to the left (anticlockwise from top view) until the robot is actually facing right
            
            "signifying by divine providence that I, Arthur, was to carry Excalibur. That is why I am your king."

        elif self.state == State.DANCE_LEFT:
            # Do some special dance for about 1s until the robot faces left.

            "Listen. Strange women lying in ponds distributing swords is no basis for a system of government."

        elif self.state == State.DANCE_RIGHT:
            # Do some special dance for about 1s until the robot faces right.

            "Supreme executive power derives from a mandate from the masses, not from some farcical aquatic ceremony."

    def set_target_heading(self, heading_change: float):
        '''
        Sets the target heading. Use self.reached_target_heading() to check if target heading has been reached.
        
        :param heading_change: The angle to add to the current heading, in radians
        :type heading_change: float
        '''

        self._target_heading = self.gyro_heading + heading_change
        # constrain heading to -pi and pi
        self._target_heading = ((self._target_heading + np.pi) % (2 * np.pi)) - np.pi
        print(f"Set target heading to {self.target_heading / np.pi * 180} deg")

    def reached_target_heading(self, tolerance: float = np.pi/36):
        '''
        If the current heading is close to the target heading, return true. Otherwise return false.
        
        :param tolerance: The angle from the target heading with which the current heading is considered to be close. In radians, and positive. Defaults to 5 degrees.
        :type tolerance: float
        '''

        heading_difference = self._target_heading - self.gyro_heading
        heading_difference = ((heading_difference + np.pi) % (2* np.pi)) - np.pi
        return np.abs(heading_difference) < tolerance

    def set_target_time(self, time_change: float = 0.0):
        '''
        Sets the target time. Use self.reached_target_time() to check if the target time has been reached.
        
        :param time_change: The amount of time in seconds from now.
        :type time_change: float
        '''
        self._target_time = self.get_clock().now() + Duration(seconds=time_change)
        print(f"Set target time to {(self._target_time.seconds_nanoseconds)}")

    def reached_target_time(self):
        '''
        Returns true if the target time has been reached. False otherwise.
        '''
        return self.get_clock().now() >= self._target_time
        
    def move(self, lin_vel: float, ang_vel: float):
        '''
        Publishes the specified velocities to move the robot. There can be slight deviations from the specified velocities.
        
        :param lin_vel: The forward velocity in m/s. Set it to negative to reverse.
        :type lin_vel: float
        :param ang_vel: The rotational velocity in rad/s. Set it to positive to turn left. Set it to negative to turn right.
        :type ang_vel: float
        '''

        msg_cmd_vel = TwistStamped()
        msg_cmd_vel.header.stamp = self.get_clock().now().to_msg()
        msg_cmd_vel.twist.linear.x = float(lin_vel)
        msg_cmd_vel.twist.angular.z = float(ang_vel)
        self._pub_cmd_vel.publish(msg_cmd_vel)

    def _callback_sub_image(self, msg: CompressedImage):
        '''
        Subscriber callback for camera image.

        - Processes the current camera video frame and uses the YOLO model to predict classes on the frame.
        - The predictions are subsequently annotated onto an image, and the image is published into the `yolo/image/compressed` topic.
        - Identifies the image classes and decodes the QR message if it is a QR message.
        '''
        # Predict classes
        image = self._cv_bridge.compressed_imgmsg_to_cv2(msg, 'bgr8') # decompresses the received image
        results = self.model_.predict(image, verbose=False) # there's only one result because only one image is used for prediction
        
        # Publish annotated images
        annotated_img = results[0].plot() # gets the annotated image in numpy form
        msg_yolo_img = self._cv_bridge.cv2_to_compressed_imgmsg(annotated_img) # converts it into compressed image for publishing.
        self._pub_yolo_img.publish(msg_yolo_img) # publish annotated image.

        # Get the predicted classes and their confidence values
        classes = results[0].boxes.cls.numpy()
        confidence_values = results[0].boxes.conf.numpy()    

        # Return if no class identified
        if len(classes) == 0:
            self.best_class = -1
            self.best_confidence = 0.0
            self.qr_message = ""
            return

        # If at least one class is identified, get the class that has the highest confidence value
        index_of_best_confidence = np.argmax(confidence_values)
        self.best_class = classes[index_of_best_confidence] 
        self.best_confidence = confidence_values[index_of_best_confidence]

        # if it is a QR code, decode it
        if self.best_class == 2: 
            retval, decoded_info, points, straight_qrcode = self.qr_.detectAndDecodeMulti(img=image)
            if len(decoded_info) > 0:
                self.qr_message = decoded_info[0] # just take the first decoded message since there is only one QR code most of the time
            else:
                self.qr_message = "" # false positive
        else:
            self.qr_message = ""

    def _callback_sub_scan(self, msg: LaserScan):
        '''
        Subscriber callback for LIDAR Scans. Stores the LIDAR readings into `self.scan_ranges`

        Example:
            ```
            self.scan_ranges[360] # front
            self.scan_ranges[540] # left
            self.scan_ranges[0]   # back
            self.scan_ranges[180] # right
            ```
        '''

        self.scan_ranges = msg.ranges
    
    def _callback_sub_imu(self, msg: Imu):
        '''
        Subscriber callback for IMU readings. Stores the gyro z-axis rotation into `self.gyro_heading`.
        '''
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.gyro_heading = np.arctan2(siny_cosp, cosy_cosp)

    # def _sleep(self, seconds: float):
    #     self.get_clock().sleep_for(Duration(seconds=seconds))
    #     print(f"Sleeping for {seconds}s")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Run())
    rclpy.shutdown()


if __name__ == '__main__':
    main()