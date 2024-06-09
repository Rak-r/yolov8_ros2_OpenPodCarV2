import numpy as np
import cv2
import math
import rclpy
from rclpy.node import Node
from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point


class KalmanFilter(Node):
    """Linear Kalman Filter for an autonomous point mass system, assuming constant velocity"""

    def __init__(self, sPos=None, sVel=None, sMeasurement=None):
        super().__init__("estimate_node")

        """
        input matrices must be numpy arrays

        :param A: state transition matrix
        :param B: state control matrix
        :param C: measurement matrix
        :param Q: covariance of the Gaussian error in state transition
        :param R: covariance of the Gaussain error in measurement
        """
        self.dt = 0.0

        self.X =  np.array(4)
        # matrices of state transition and measurement
        self.A = np.array([[1, dt, 0, 0], [0, 1, 0, 0], [0, 0, 1, dt], [0, 0, 0, 1]])
        self.B = np.array([[0, 0], [dt, 0], [0, 0], [0, dt]])
        self.C = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])
        # default noise covariance
        if (sPos is None) and (sVel is None) and (sMeasurement is None):
            # sPos = 0.5 * 5 * dt ** 2  # assume 5m/s2 as maximum acceleration
            # sVel = 5.0 * dt  # assume 8.8m/s2 as maximum acceleration
            sPos = 1.3*self.dt   # assume 5m/s2 as maximum acceleration
            sVel = 4*self.dt  # assume 8.8m/s2 as maximum acceleration
            sMeasurement = 0.2 # 68% of the measurement is within [-sMeasurement, sMeasurement]

        # state transition noise
        self.Q = np.diag([sPos ** 2, sVel ** 2, sPos ** 2, sVel ** 2])
        # measurement noise
        self.R = np.diag([sMeasurement ** 2, sMeasurement ** 2])


        self.previous_detections = {}

         # publishers
        self._pub = self.create_publisher(DetectionArray, "detections_speed", 10)
        self.trajectory_pub = self.create_publisher(PoseStamped, '/object_trajectory', 10)                                 # publish the detected and tracked object's poses / trajectories
        
        #subscribers
        self._sub = self.create_subscription(
            DetectionArray, "detections_3d", self.detections_cb, 10)

    def detections_cb(self, detection_msg: DetectionArray):

        cur_time = detection_msg.header.stamp.sec + \
            detection_msg.header.stamp.nanosec / 1e9
         
        detection: Detection
        for detection in detection_msg.detections:
            cur_pos = detection.bbox3d.center

            if detection.id in self.previous_detections:

                self.dt = cur_time - \
                    self.previous_detections[detection.id]["timestamp"]
    
    def predict(self):



    

    



