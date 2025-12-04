import os
import numpy as np

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

class Run(Node):

    def __init__(self):
        super().__init__('run')
        
        # Handles
        self.cv_bridge_ = cv_bridge.CvBridge()
        self.sub_image_ = self.create_subscription(
            CompressedImage, 'camera/image_raw/compressed', self.callback_sub_image_, 2)
        self.sub_scan_ = self.create_subscription(
            LaserScan, 'scan', self.callback_sub_scan_, 2)
        self.sub_imu_ = self.create_subscription(
            Imu, 'imu', self.callback_sub_imu_, 2)
        self.pub_yolo_img_ = self.create_publisher(
            CompressedImage, 'yolo/image/compressed', 2)
        self.pub_cmd_vel_ = self.create_publisher(
            TwistStamped, 'cmd_vel', 2)
        self.timer_ = self.create_timer(
            0.05, self.run_)
        
        # YOLO Model
        path_to_model = os.path.join(os.path.expanduser("~"), "tb3_yolo/", "best.pt")
        print(path_to_model)
        self.model_ = YOLO(path_to_model)

        # QRCode Detector (don't actually have to use YOLO)
        self.qr_ = QRCodeDetector()
        
        # Other states / variables
        self.best_class_ = None
        self.best_confidence_ = 0.0
        self.qr_message_ = ""
        self.scan_ranges_ = None
        self.gyro_heading_ = None
        self.target_heading_ = 0

        self.STATE_TURN_LEFT = 1
        self.STATE_TURN_RIGHT = 2
        self.STATE_FOLLOW_WALL = 3
        self.STATE_CAPTURE = 4
        self.STATE_SPIN_LEFT = 5
        self.STATE_SPIN_RIGHT = 6
        self.STATE_DANCE_LEFT = 7
        self.STATE_DANCE_RIGHT = 8
        
        # initial state
        self.state = self.STATE_FOLLOW_WALL

    def run_(self):

        # !RETURN UNTIL ALL DATA IS RECEIVED =======================
        if self.best_class_ is None or self.scan_ranges_ is None or self.gyro_heading_ is None:
            return
        # !=============================================================

        # Print statements to terminal
        print(f"YOLO: Best class={self.best_class_}), Best confidence value={self.best_confidence_}")
        print(f"Gyro: {self.gyro_heading_} rad, or {self.gyro_heading_ / np.pi * 180} deg")

        # get lidar scan ranges
        front_right_range = self.scan_ranges_[270]
        front_range = self.scan_ranges_[360]
        front_left_range = self.scan_ranges_[450]
        left_range = self.scan_ranges_[540]
        right_range = self.scan_ranges_[180]
        print(f"LIDAR: L={left_range:.2f}, FL={front_left_range:.2f}, F={front_range:.2f}, FR={front_right_range:.2f}, {right_range:.2f}")

        # begin state machine
        if self.state == self.STATE_FOLLOW_WALL:
            
            if (front_right_range != np.inf): # lidar gives inf values if it cannot reliably detect the range

                # P control for angular velocity
                ang_error = 0.2 - front_right_range
                kp = 5.0
                ang_vel = kp * ang_error
                self.move_(0.03, ang_vel)

            # Change state to "capture" if a wall is one cell in front of the robot
            if front_range < 0.4:
                self.state = self.STATE_CAPTURE
                print("Switching from FOLLOW_WALL to CAPTURE")

        elif self.state == self.STATE_CAPTURE:
            # Stop the robot
            self.move_(0, 0)

            # Do some capture
            if self.best_class_ == 0: # left class
                print("Switching from CAPTURE to TURN_LEFT")
                self.state = self.STATE_TURN_LEFT
                self.target_heading_ = self.gyro_heading_ + np.pi / 2

            elif self.best_class_ == 1: # right class
                print("Switching from CAPTURE to TURN_RIGHT")
                self.state = self.STATE_TURN_RIGHT
                self.target_heading_ = self.gyro_heading_ - np.pi / 2

            elif self.best_class_ == 2: # qr class
                if self.qr_message_ == "spin left":
                    print("Switching from CAPTURE to SPIN_LEFT")
                    self.state = self.STATE_SPIN_LEFT
                elif self.qr_message_ == "spin right":
                    print("Switching from CAPTURE to SPIN_RIGHT")
                    self.state = self.STATE_SPIN_RIGHT
                elif self.qr_message_ == "dance left":
                    print("Switching from CAPTURE to DANCE_LEFT")
                    self.state = self.STATE_DANCE_LEFT
                elif self.qr_message_ == "dance right":
                    print("Switching from CAPTURE to DANCE_RIGHT")
                    self.state = self.STATE_DANCE_RIGHT

        elif self.state == self.STATE_TURN_LEFT:
            # Turn until the robot faces 90 deg left from the original position.
            self.move_(0, 0.1)

            # The target heading is set immediately before the state becomes this
            if np.abs(self.gyro_heading_ - self.target_heading_) < 0.08726646259: # 5 degrees
                print("Switching from TURN_LEFT to FOLLOW_WALL")
                self.state = self.STATE_FOLLOW_WALL

        elif self.state == self.STATE_TURN_RIGHT:
            # Turn until the robot faces 90 deg right from the original position
            self.move_(0, -0.1)

            # The target heading is set immediately before the state becomes this
            if np.abs(self.gyro_heading_ - self.target_heading_) < 0.08726646259: # 5 degrees
                print("Switching from TURN_RIGHT to FOLLOW_WALL")
                self.state = self.STATE_FOLLOW_WALL

        elif self.state == self.STATE_SPIN_LEFT:
            # Spin 270 degrees to the right (clockwise from top view) until the robot is actually facing left

            print("Switching from SPIN_LEFT to FOLLOW_WALL")
            self.state = self.STATE_FOLLOW_WALL

            # "The Lady of the Lake, her arm clad in the purest shimmering samite, held aloft Excalibur from the bosom of the water," 

        elif self.state == self.STATE_SPIN_RIGHT:
            # Spin 270 degrees to the left (anticlockwise from top view) until the robot is actually facing right
            
            print("Switching from SPIN_RIGHT to FOLLOW_WALL")
            self.state = self.STATE_FOLLOW_WALL

            # "signifying by divine providence that I, Arthur, was to carry Excalibur. That is why I am your king."

        elif self.state == self.STATE_DANCE_LEFT:
            # Do some special dance for about 1s until the robot faces left.

            print("Switching from DANCE_LEFT to FOLLOW_WALL")
            self.state = self.STATE_FOLLOW_WALL

            # "Listen. Strange women lying in ponds distributing swords is no basis for a system of government."

        elif self.state == self.STATE_DANCE_RIGHT:
            # Do some special dance for about 1s until the robot faces right.

            print("Switching from DANCE_RIGHT to FOLLOW_WALL")
            self.state = self.STATE_FOLLOW_WALL

            # "Supreme executive power derives from a mandate from the masses, not from some farcical aquatic ceremony."


    def callback_sub_image_(self, msg: CompressedImage):
        '''
        Subscriber callback for camera image.

        - Processes the current camera video frame and uses the YOLO model to predict classes on the frame.
        - The predictions are subsequently annotated onto an image, and the image is published into the `yolo/image/compressed` topic.
        - Identifies the image classes and decodes the QR message if it is a QR message.
        '''
        # Predict classes
        image = self.cv_bridge_.compressed_imgmsg_to_cv2(msg, 'bgr8') # decompresses the received image
        results = self.model_.predict(image, verbose=False) # there's only one result because only one image is used for prediction
        
        # Publish annotated images
        annotated_img = results[0].plot() # gets the annotated image in numpy form
        msg_yolo_img = self.cv_bridge_.cv2_to_compressed_imgmsg(annotated_img) # converts it into compressed image for publishing.
        self.pub_yolo_img_.publish(msg_yolo_img) # publish annotated image.

        # Get the predicted classes and their confidence values
        classes = results[0].boxes.cls.numpy()
        confidence_values = results[0].boxes.conf.numpy()    

        # Return if no class identified
        if len(classes) == 0:
            self.best_class_ = -1
            self.best_confidence_ = 0.0
            self.qr_message_ = ""
            return

        # If at least one class is identified, get the class that has the highest confidence value
        index_of_best_confidence = np.argmax(confidence_values)
        self.best_class_ = classes[index_of_best_confidence] 
        self.best_confidence_ = confidence_values[index_of_best_confidence]

        # if it is a QR code, decode it
        if self.best_class_ == 2: 
            retval, decoded_info, points, straight_qrcode = self.qr_.detectAndDecodeMulti(img=image)
            if len(decoded_info) > 0:
                self.qr_message_ = decoded_info[0] # just take the first decoded message since there is only one QR code most of the time
            else:
                self.qr_message_ = "" # false positive
        else:
            self.qr_message_ = ""

    def callback_sub_scan_(self, msg: LaserScan):
        '''
        Subscriber callback for LIDAR Scans. Stores the LIDAR readings into `self.scan_ranges_`

        Example:
            ```
            self.scan_ranges_[360] # front
            self.scan_ranges_[540] # left
            self.scan_ranges_[0]   # back
            self.scan_ranges_[180] # right
            ```
        '''

        self.scan_ranges_ = msg.ranges
    
    def callback_sub_imu_(self, msg: Imu):
        '''
        Subscriber callback for IMU readings. Stores the gyro z-axis rotation into `self.gyro_heading_`.
        '''
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.gyro_heading_ = np.arctan2(siny_cosp, cosy_cosp)
        
    def move_(self, lin_vel: float, ang_vel: float):
        '''
        Publishes the specified velocities to move the robot. There can be slight deviations from the specified velocities.
        
        Args:
            lin_vel (float): The forward velocity in m/s. Set it to negative to reverse.
            ang_vel (float): The rotational velocity in rad/s. Set it to positive to turn left. Set it to negative to turn right.
        '''

        msg_cmd_vel = TwistStamped()
        msg_cmd_vel.header.stamp = self.get_clock().now().to_msg()
        msg_cmd_vel.twist.linear.x = float(lin_vel)
        msg_cmd_vel.twist.angular.z = float(ang_vel)
        self.pub_cmd_vel_.publish(msg_cmd_vel)

    def sleep_(self, seconds: float):
        self.get_clock().sleep_for(Duration(seconds=seconds))
        print(f"Sleeping for {seconds}s")


        

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Run())
    rclpy.shutdown()


if __name__ == '__main__':
    main()