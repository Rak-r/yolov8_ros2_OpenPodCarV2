# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


'''

1.) To operate the OpenPodCar2 (https://github.com/Rak-r/OpenPodCar_V2) in real world as well as simualtion with full SLAM and NAV2 stack.

2.) It was observed that with dynamic obstacles like moving Pedestrians, cars the vehicle was struggling with localization jumps and NAV2 path following issues.

3.) To handle the scenario for miving object environment, the detectd and tracked information from the YOLOV8 was extracted and maksed out from the original 
incoming depth image from the RGBD sensor, and publihsed on a seperate topic named /masked_depth. This newly publihsed topic then subscribed by RTABMAP's
rgbd odometry and rtabmap slam instead of original /depth topic.

4.) Test were conducted by moving heavily infront of the camera and it is observed that the localization jumps were handled and vehicle was able to 
navigate to desired goal locations.

5.) To maintain the simplicity the mentioned approach is integrated in the 2D detection node.

'''


from typing import List, Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from cv_bridge import CvBridge

from ultralytics import YOLO
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from ultralytics.engine.results import Masks
from ultralytics.engine.results import Keypoints

from sensor_msgs.msg import Image
from yolov8_msgs.msg import Point2D
from yolov8_msgs.msg import BoundingBox2D
from yolov8_msgs.msg import Mask
from yolov8_msgs.msg import KeyPoint2D
from yolov8_msgs.msg import KeyPoint2DArray
from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray
from std_srvs.srv import SetBool
import message_filters

import numpy as np
class Yolov8Node(Node):

    def __init__(self) -> None:
        super().__init__("yolov8_node")

        
        # params
        self.declare_parameter("model", "yolov8m.pt")
        model = self.get_parameter(
            "model").get_parameter_value().string_value
        
        self.declare_parameter("classes", list(range(1)))                             # add the person class id to detect and generate the BBox messages
        self.classes = self.get_parameter(
            "classes").get_parameter_value().integer_array_value

        self.declare_parameter("device", "cuda:0")
        self.device = self.get_parameter(
            "device").get_parameter_value().string_value

        self.declare_parameter("threshold", 0.5)
        self.threshold = self.get_parameter(
            "threshold").get_parameter_value().double_value

        self.declare_parameter("enable", True)
        self.enable = self.get_parameter(
            "enable").get_parameter_value().bool_value

        self.declare_parameter("image_reliability",
                               QoSReliabilityPolicy.BEST_EFFORT)
        image_qos_profile = QoSProfile(
            reliability=self.get_parameter(
                "image_reliability").get_parameter_value().integer_value,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.cv_bridge = CvBridge()
        self.yolo = YOLO(model)
        self.yolo.fuse()

        # pubs
        self._pub = self.create_publisher(DetectionArray, "detections", 10)
        self.masked_depth_pub = self.create_publisher(Image, '/masked_depth', 10)           # add another publisher for /masked depth topic
        # subs
        self._sub = self.create_subscription(
            Image, "image_raw", self.image_cb,
            image_qos_profile
        )
        
        self.depth_sub = self.create_subscription(Image, '/depth', self.depth_callback, 10)  # subscribe to original depth topic
        # services
        self._srv = self.create_service(SetBool, "enable", self.enable_cb)

        # depth image related attributes
        self.latest_depth_image = None
        self.latest_depth_image_header =  None

        
    def enable_cb(
        self,
        req: SetBool.Request,
        res: SetBool.Response
    ) -> SetBool.Response:
        self.enable = req.data
        res.success = True
        return res

    def parse_hypothesis(self, results: Results) -> List[Dict]:

        hypothesis_list = []

        box_data: Boxes
        for box_data in results.boxes:
            class_id = int(box_data.cls)
            class_name = self.yolo.names[class_id]
            hypothesis = {
                "class_id": class_id,
                "class_name": class_name,
                "score": float(box_data.conf)
            }
            hypothesis_list.append(hypothesis)

        return hypothesis_list

    def parse_boxes(self, results: Results) -> List[BoundingBox2D]:

        boxes_list = []

        box_data: Boxes
        for box_data in results.boxes:
            

            msg = BoundingBox2D()

            # get boxes values
            box = box_data.xywh[0]
            msg.center.position.x = float(box[0])
            msg.center.position.y = float(box[1])
            msg.size.x = float(box[2])
            msg.size.y = float(box[3])

            # append msg
            boxes_list.append(msg)

        return boxes_list

    def parse_masks(self, results: Results) -> List[Mask]:

        masks_list = []

        def create_point2d(x: float, y: float) -> Point2D:
            p = Point2D()
            p.x = x
            p.y = y
            return p

        mask: Masks
        for mask in results.masks:

            msg = Mask()

            msg.data = [create_point2d(float(ele[0]), float(ele[1]))
                        for ele in mask.xy[0].tolist()]
            msg.height = results.orig_img.shape[0]
            msg.width = results.orig_img.shape[1]

            masks_list.append(msg)

        return masks_list

    def parse_keypoints(self, results: Results) -> List[KeyPoint2DArray]:

        keypoints_list = []

        points: Keypoints
        for points in results.keypoints:

            msg_array = KeyPoint2DArray()

            if points.conf is None:
                continue

            for kp_id, (p, conf) in enumerate(zip(points.xy[0], points.conf[0])):

                if conf >= self.threshold:
                    msg = KeyPoint2D()

                    msg.id = kp_id + 1
                    msg.point.x = float(p[0])
                    msg.point.y = float(p[1])
                    msg.score = float(conf)

                    msg_array.data.append(msg)

            keypoints_list.append(msg_array)

        return keypoints_list
    

    '''
    callback to subscribe to depth image from rgbd sensor
    '''
    def depth_callback(self, msg):
        self.latest_depth_image_header = msg.header
        self.latest_depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')


    '''
    Function to mask out tt=he depth image with detected person's bbox dimensions
    '''

    def masked_depth_image(self, depth_image, detection_array: DetectionArray, padding: int = 0):
        masked_depth_image = depth_image.copy()

        image_height, image_width = depth_image.shape[:2]  # Get the dimensions of the image

        for detection in detection_array.detections:
            # Convert from xywh to x1, y1, x2, y2
            x_center = detection.bbox.center.position.x
            y_center = detection.bbox.center.position.y
            bbox_width = detection.bbox.size.x
            bbox_height = detection.bbox.size.y

            # Ensure padding is an integer, added to handle some edges which can be visible while person is moving 
            if padding is None:
                padding = 0

            x_min = int((x_center - bbox_width / 2) - padding)
            x_max = int((x_center + bbox_width / 2) + padding)
            y_min = int((y_center - bbox_height / 2) - padding)
            y_max = int((y_center + bbox_height / 2) + padding)

            # Ensure bbox coordinates are within image bounds
            x_min = max(0, x_min)
            x_max = min(image_width, x_max)   # Use image_width instead of depth_image[1]
            y_min = max(0, y_min)
            y_max = min(image_height, y_max)  # Use image_height instead of depth_image[0]

            # Mask the detected person's bounding box in the depth image
            masked_depth_image[y_min:y_max, x_min:x_max] = 0  # Set masked area to zero depth

        return masked_depth_image

    def image_cb(self, msg: Image) -> None:
        masked_depth = Image()
        # self.get_logger().info('No person detected')
        if self.enable:

            # convert image + predict
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
            results = self.yolo.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                classes = self.classes,                                   # just detect the person class, can detect all classes as well as per application requirement.
                conf=self.threshold,
                device=self.device
            )
            results: Results = results[0].cpu()

            if results.boxes:
                hypothesis = self.parse_hypothesis(results)
                boxes = self.parse_boxes(results)

            if results.masks:
                masks = self.parse_masks(results)

            if results.keypoints:
                keypoints = self.parse_keypoints(results)

            # create detection msgs
            detections_msg = DetectionArray()

            for i in range(len(results)):
              
                aux_msg = Detection()
                
                if results.boxes:
                    aux_msg.class_id = hypothesis[i]["class_id"]
                    aux_msg.class_name = hypothesis[i]["class_name"]
                    aux_msg.score = hypothesis[i]["score"]
                  
                    aux_msg.bbox = boxes[i]

                if results.masks:
                    aux_msg.mask = masks[i]

                if results.keypoints:
                    aux_msg.keypoints = keypoints[i]

                detections_msg.detections.append(aux_msg)

            # publish detections
            # Mask the depth image
            detections_msg.header = msg.header
            if self.latest_depth_image is not None:
                masked_image_cv2 = self.masked_depth_image(self.latest_depth_image, detections_msg, 10)
                masked_image_msg = self.cv_bridge.cv2_to_imgmsg(masked_image_cv2, encoding='passthrough')
                masked_image_msg.header = self.latest_depth_image_header # Set the header for the masked depth image
                masked_depth = masked_image_msg
            self.masked_depth_pub.publish(masked_depth)

            self._pub.publish(detections_msg)

def main():
    rclpy.init()
    node = Yolov8Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
