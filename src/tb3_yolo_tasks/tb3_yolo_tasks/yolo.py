# TODO: Merge to tb3_yolo_camera

import os
import numpy as np

import rclpy
from rclpy.node import Node
import cv_bridge
from ultralytics import YOLO
from cv2 import QRCodeDetector

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int64MultiArray


class YOLOAndQR(Node):

    def __init__(self):
        super().__init__('run')
        
        # Handles
        self._cv_bridge = cv_bridge.CvBridge()
        self._sub_image = self.create_subscription(
            CompressedImage, 'camera/image_raw/compressed', self._callback_sub_image, 2)
        self._pub_yolo_img = self.create_publisher(
            CompressedImage, 'yolo/image/compressed', 2)
        self._pub_yolo_result = self.create_publisher(
            Int64MultiArray, 'yolo/result', 2)
        
        # YOLO Model
        path_to_model = os.path.join(os.path.expanduser("~"), "tb3_yolo", "best.pt")
        print(f"Loading {path_to_model}...")
        self.model_ = YOLO(path_to_model)

        # QRCode Detector (don't actually have to use YOLO)
        self.qr_ = QRCodeDetector()
        
        # Other states / variables
        self.result = Int64MultiArray()
        self.result.data = [-1, -1]
        
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
            self.result.data = [-1, -1]
        else:

            # If at least one class is identified, get the class that has the highest confidence value
            index_of_best_confidence = np.argmax(confidence_values)
            best_class = int(classes[index_of_best_confidence])
            
            # if it is a QR code, decode it
            if best_class == 2: 
                retval, decoded_info, points, straight_qrcode = self.qr_.detectAndDecodeMulti(img=image)
                if len(decoded_info) > 0:
                    if decoded_info[0] == 'spin left': # just take the first decoded message since there is only one QR code most of the time
                        self.result.data[1] = 0
                    elif decoded_info[0] == 'spin right':
                        self.result.data[1] = 1
                    elif decoded_info[0] == 'dance left':
                        self.result.data[1] = 2
                    elif decoded_info[0] == 'dance right':
                        self.result.data[1] = 3
                    else:
                        self.result.data[1] = -1
                else:
                    self.result.data[1] = -1 # false positive / occluded
            else:
                self.result.data[1] = -1

            self.result.data[0] = best_class
        
        self._pub_yolo_result.publish(self.result)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(YOLOAndQR())
    rclpy.shutdown()


if __name__ == '__main__':
    main()