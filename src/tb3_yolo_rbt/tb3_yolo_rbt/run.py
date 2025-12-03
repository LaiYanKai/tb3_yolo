import os
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import cv_bridge
from ultralytics import YOLO

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
        
        # Other states / variables
        self.yolo_classes_ = None
        self.yolo_confs_ = None
        self.scan_ranges_ = None
        self.gyro_ = None

    def callback_sub_image_(self, msg: CompressedImage):
        '''
        Subscriber callback for camera image.

        - Processes the current camera video frame and uses the YOLO model to predict classes on the frame.
        - The predictions are subsequently annotated onto an image, and the image is published into the `yolo/image/compressed` topic.
        - The predicted classes are stored as a numpy array into `self.yolo_classes_`. 
        - The corresponding confidences are stored as a numpy array into `yolo_confs_`.
        '''
        # Predict classes
        image = self.cv_bridge_.compressed_imgmsg_to_cv2(msg, 'bgr8') # decompresses the received image
        results = self.model_.predict(image, verbose=False) # there's only one result because only one image is used for prediction
        
        # Publish annotated images
        annotated_img = results[0].plot() # gets the annotated image in numpy form
        msg_yolo_img = self.cv_bridge_.cv2_to_compressed_imgmsg(annotated_img) # converts it into compressed image for publishing.
        self.pub_yolo_img_.publish(msg_yolo_img) # publish annotated image.

        # Store the predicted classes
        self.yolo_classes_ = results[0].boxes.cls.numpy()
        self.yolo_confs_ = results[0].boxes.conf.numpy()        

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
        Subscriber callback for IMU readings. Stores the gyro z-axis rotation into `self.gyro_`.
        '''
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.gyro_ = np.arctan2(siny_cosp, cosy_cosp)
        
    def move_(self, lin_vel: float, ang_vel: float):
        '''
        Publishes the specified velocities to move the robot. There can be slight deviations from the specified velocities.
        
        Args:
            lin_vel (float): The forward velocity in m/s. Set it to negative to reverse.
            ang_vel (float): The rotational velocity in rad/s. Set it to positive to turn left. Set it to negative to turn right.
        '''

        msg_cmd_vel = TwistStamped()
        msg_cmd_vel.header.stamp = self.get_clock().now()
        msg_cmd_vel.twist.linear.x = lin_vel
        msg_cmd_vel.twist.angular.z = ang_vel
        self.pub_cmd_vel_.publish(msg_cmd_vel)

    def sleep_(self, seconds: float):
        self.get_clock().sleep_for(Duration(seconds=seconds))
        print(f"Sleeping for {seconds}s")

    def run_(self):

        # !RETURN UNTIL ALL DATA IS RECEIVED =======================
        if self.yolo_classes_ is None or self.scan_ranges_ is None or self.gyro_ is None:
            return
        # !=============================================================

        self.sleep_(0.5)
        print(f"YOLO: Classes{self.yolo_classes_}, Confs{self.yolo_confs_}")
        print(f"LIDAR: F{self.scan_ranges_[360]:.2f}, L{self.scan_ranges_[540]:.2f}, B{self.scan_ranges_[0]:.2f}, R{self.scan_ranges_[180]:.2f}")
        print(f"Gyro: {self.gyro_} rad / {self.gyro_ / np.pi * 180}")
        

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Run())
    rclpy.shutdown()


if __name__ == '__main__':
    main()