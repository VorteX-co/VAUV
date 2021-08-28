"""
Node for publishing objects.

Node for publishing objects attributes and their point cloud found in a zed2
left camera frame detected by a custom yolov5 model;
custom model will be added soon.
this Node uses message_filters to synchronize ros msgs instead of flags.
"""
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from custom_msgs.msg import ObjectList
from custom_msgs.msg import Object
import message_filters
import ros2_numpy
import torch
import math
# import cv2


class Detection:
    """
    Stores attributes of a detect objects resulted from yolov5 custom model.

    Args
    ----
        label: label of detected object (int)
        threshold: threshold of detected object (float) between 0->1
        coordinates: bbox of detected object in camera frame and point cloud of
        object's midpoint with respect

    Attributes
    ----------
        __object_label: object's label
        __object_threshold": object's threshold
        __object_xmin: top left x of object's bbox
        __object_xmax: bottom right x of object's bbox
        __object_ymin: top left y of object's bbox
        __object_ymax: bottom right y of object's bbox
        __object_cloud_x: real x value of the midpoint of object's bbox with
        respect to zed2 left camera, maybe NAN, INF and -INF
        __object_cloud_y: real y value of the midpoint of object's bbox with
        respect to zed2 left camera, maybe NAN, INF and -INF
        __object_cloud_z: real z value of the midpoint of object's bbox with
        respect to zed2 left camera, maybe NAN, INF and -INF

    """

    def __init__(self, label, threshold, coordinates):
        """
        Constructor.

        :param label: label of detected object (int).
        :param threshold: threshold of detected object (float) between 0->1
        :param coordinates: bbox of detected object in camera frame and point
        cloud of object's midpoint with respect to the left camera
        """
        self.__object_label = label  # object label
        self.__object_threshold = threshold  # object threshold
        # coordinates of detected object in camera frame
        self.__object_xmin = coordinates[0]
        self.__object_xmax = coordinates[2]
        self.__object_ymin = coordinates[1]
        self.__object_ymax = coordinates[3]
        # point cloud of object's mid point with respect to the left camera
        self.__object_cloud_x = coordinates[4]
        self.__object_cloud_y = coordinates[5]
        self.__object_cloud_z = coordinates[6]

    def get_label(self):
        """:return: object's label."""
        return self.__object_label

    def get_name(self):
        """
        :return: object's name.

        may return unknown if label isn't defined in dictionary.
        """
        objects_names = {0: "Gate", 1: "Path"}
        if self.__object_label in objects_names.keys():
            return objects_names[self.__object_label]
        else:
            return "unknown"

    def get_threshold(self):
        """:return: object's threshold."""
        return self.__object_threshold

    def get_xmin(self):
        """:return: top left x of object's bbox."""
        return self.__object_xmin

    def get_xmax(self):
        """:return: bottom right x of object's bbox."""
        return self.__object_xmax

    def get_ymin(self):
        """:return: top left y of object's bbox."""
        return self.__object_ymin

    def get_ymax(self):
        """:return: bottom right y of object's bbox."""
        return self.__object_ymax

    def get_cloud_x(self):
        """
        :return: real x value.

        value of the midpoint of object's bbox with respect to zed2 left
        camera, maybe NAN, INF and -INF
        """
        return self.__object_cloud_x

    def get_cloud_y(self):
        """
        :return: real y value.

        value of the midpoint of object's bbox with respect to zed2 left
        camera, maybe NAN, INF and -INF
        """
        return self.__object_cloud_y

    def get_cloud_z(self):
        """
        :return: real z value.

        value of the midpoint of object's bbox with respect to zed2 left
        camera, maybe NAN, INF and -INF
        """
        return self.__object_cloud_z


class ZedFrame:
    """
    Stores ZED2 left camera frame.

    Args
    ----
        ros_msg: subscribed msg from topic:
        /zed2/zed_node/left/image_rect_color

    Attributes
    ----------
        __frame: ZED2 left camera frame
        __nano: timestamp (nano sec)
        __second: timestamp (second)

    """

    def __init__(self, ros_msg):
        """
        :param ros_msg: subscribed msg.

        from topic:/zed2/zed_node/left/image_rect_color
        """
        # ZED2 left camera frame
        self.__frame = CvBridge().imgmsg_to_cv2(ros_msg)
        # timestamp
        self.__nano = ros_msg.header.stamp.nanosec
        self.__second = ros_msg.header.stamp.sec

    def get_frame(self):
        """:return: return ZED2 left camera frame."""
        return self.__frame

    def get_nanosec(self):
        """:return: return timestamp (nano sec)."""
        return self.__nano

    def get_second(self):
        """:return: return timestamp (second)."""
        return self.__second


class ZedCLoud:
    """
    Stores ZED2 left camera point cloud.

    Args
    ----
        ros_msg: subscribed msg from topic:
        '/zed2/zed_node/point_cloud/cloud_registered'

    Attributes
    ----------
        __cloud: ZED2 left camera point cloud
        __nano: timestamp (nano sec)
        __second: timestamp (second)

    """

    def __init__(self, ros_msg):
        """
        :param ros_msg: subscribed msg.

        from topic:'/zed2/zed_node/point_cloud/cloud_registered'
        """
        # ZED2 left camera point cloud
        self.__cloud = ros2_numpy.numpify(ros_msg)
        # timestamp
        self.__nano = ros_msg.header.stamp.nanosec
        self.__second = ros_msg.header.stamp.sec

    def get_cloud(self):
        """:return: ZED2 left camera point cloud."""
        return self.__cloud

    def get_nanosec(self):
        """:return: timestamp (nano sec)."""
        return self.__nano

    def get_second(self):
        """:return: timestamp (second)."""
        return self.__second


class MinimalSubscriber(Node):
    """
    Class for ROS2 Node.

    Attributes
    ----------
        self.publisher_: publisher for detected objects
        self.subscription_cloud: create subscriber for point cloud
        self.subscription_frame: create subscriber for frame
        self.synchronizer: create TimeSynchronizer to sync ros msgs
        self.model: yolov5 model

    """

    def __init__(self):
        """Constructor."""
        super().__init__('object_publisher_msg_filter_zed2')
        # publisher for detected objects
        self.publisher_ = self.create_publisher(ObjectList,
                                                'zed_objects_list', 10)
        # create subscriber for frame
        self.subscription_frame = message_filters.Subscriber(self, Image,
                                                             '/zed2/zed_node/'
                                                             'left/'
                                                             'image_rect_color'
                                                             '')
        # create subscriber for point cloud
        self.subscription_cloud = message_filters.Subscriber(self, PointCloud2,
                                                             '/zed2/zed_node/'
                                                             'point_cloud/'
                                                             'cloud_registered'
                                                             '')
        # create TimeSynchronizer to sync ros msgs
        self.synchronizer = message_filters.TimeSynchronizer(
            [self.subscription_frame,
             self.subscription_cloud], 10)
        # create TimeSynchronizer callback function
        self.synchronizer.registerCallback(self.listener_callback)
        # yolov5 model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

    def listener_callback(self, frame_msg, cloud_msg):
        """
        Call when msg is published.

        :param frame_msg: subscribed msg from topic:
        '/zed2/zed_node/point_cloud/cloud_registered'
        :param cloud_msg: subscribed msg from topic:
        '/zed2/zed_node/point_cloud/cloud_registered'
        """
        # frame obj object
        frame_obj = ZedFrame(frame_msg)
        # point cloud object
        cloud_obj = ZedCLoud(cloud_msg)
        # get predictions
        predictions = self.model(frame_obj.get_frame())
        # use ObjectList msg; custom msg for detected objects
        objects_list = ObjectList()
        # loop through predictions and find object attributes and
        # real coordinates
        for i in range(len(predictions.xyxy[0])):
            objects_list.detected_objects.append(
                self.get_object_msg(prediction=predictions.xyxy[0][i],
                                    width=frame_obj.get_frame().shape[1],
                                    height=frame_obj.get_frame().shape[0],
                                    point_cloud=cloud_obj.get_cloud()))

        # publish ObjectList
        self.publisher_.publish(objects_list)
        self.get_logger().info('Publishing Objects')

    @staticmethod
    def cloud_state(current_cloud):
        """
        Use to check if point cloud is calculated.

        it may be NAN, INF and-INF if not calculated
        :param current_cloud: real coordinates of object's bbox midpoint
        """
        x, y, z = current_cloud[0], current_cloud[1], current_cloud[2]
        # check if x, y and z are NAN
        if math.isnan(x) or math.isnan(y) or math.isnan(z):
            # return float NAN; ros msg has float
            return [float("nan"), float("nan"), float("nan")]
        # check if x, y and z are INF OR - INF
        if math.isinf(x) or math.isinf(y) or math.isinf(z):
            # check if x, y and z are -INF
            if x < 0 or y < 0 or z < 0:
                # return float -INF; ros msg has float
                return [float("-inf"), float("-inf"), float("-inf")]
            else:
                # return float INF; ros msg has float
                return [float("inf"), float("inf"), float("inf")]
        # x, y and z are calculated; return their values
        return [x, y, z]

    def convert_yolo_coordinates(self, coordinates, width, height,
                                 point_cloud):
        """
        Use to format coordinates.

        :param coordinates: yolov5 bbox coordinates
        :param width: with of frame
        :param height: height of frame
        :param point_cloud: point cloud of frame
        :return: Detection object
        """
        # get data from GPU and convert them to numpy
        coordinates = coordinates.cpu().numpy()
        # check if coordinates are within frame dimensions
        x1, y1, x2, y2 = coordinates[0:4]
        x1, y1 = int(min(x1, x2)), int(min(y1, y2))
        x2, y2 = int(max(x1, x2)), int(max(y1, y2))
        x1 = x1 if x1 >= 0 else 0
        y1 = y1 if y1 >= 0 else 0
        x2 = x2 if x2 < width else width - 1
        y2 = y2 if y2 < height else height - 1
        # get real coordinates of bbox midpoint
        object_cloud = self.cloud_state(point_cloud[(y1 + y2) // 2][(x1 + x2) // 2])
        # object label
        object_label = int(coordinates[-1])
        # object threshold
        object_threshold = coordinates[-2]
        return Detection(object_label, object_threshold,
                         [x1, y1, x2, y2, float(object_cloud[0]),
                          float(object_cloud[1]), float(object_cloud[2])])

    def get_object_msg(self, prediction, width, height, point_cloud):
        """
        Use to get Object msg.

        :param prediction: yolov5 bbox coordinates
        :param width: with of frame
        :param height: height of frame
        :param point_cloud: point cloud of frame
        :return: Object ros msg; custom message for detected objects
        """
        # Detection object
        detection_class = self.convert_yolo_coordinates(
            prediction=prediction,
            width=width,
            height=height,
            point_cloud=point_cloud)
        # get Object msg; custom msg for detected objects
        current_object = Object()
        current_object.label = int(detection_class.get_label())
        current_object.name = detection_class.get_name()
        current_object.threshold = float(detection_class.get_threshold())
        current_object.xmin = detection_class.get_xmin()
        current_object.xmax = detection_class.get_xmax()
        current_object.ymin = detection_class.get_ymin()
        current_object.ymax = detection_class.get_ymax()
        current_object.cloud_x = detection_class.get_cloud_x()
        current_object.cloud_y = detection_class.get_cloud_y()
        current_object.cloud_z = detection_class.get_cloud_z()
        return current_object


def main(args=None):
    """main."""
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
