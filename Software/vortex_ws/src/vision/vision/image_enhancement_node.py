# Copyright 2020-2021 Vortex-co.
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
from sensor_msgs.msg import Image
import cv2
import numpy as np


class color_correction:
    def __init__(self, imag):
        self.image = imag

    def Histogram_stretching(self, channel, clip_hist_percent=1.5):
        # Calculate channel histogram
        hist = cv2.calcHist([channel], [0], None, [256], [0, 256])
        hist_size = len(hist)

        # Calculate cumulative distribution from the histogram
        accumulator = []
        accumulator.append(float(hist[0]))
        for index in range(1, hist_size):
            accumulator.append(accumulator[index - 1] + float(hist[index]))

        # Locate points to clip
        maximum = accumulator[-1]
        clip_hist_percent *= (maximum/100.0)
        clip_hist_percent /= 2.0

        # Locate left cut
        minimum_ch = 0
        while accumulator[minimum_ch] < clip_hist_percent:
            minimum_ch += 1

        # Locate right cut
        maximum_ch = hist_size - 1
        while accumulator[maximum_ch] >= (maximum - clip_hist_percent):
            maximum_ch -= 1
    # Calculate alpha and beta values
        alpha = 255 / (maximum_ch - minimum_ch)
        beta = -minimum_ch * alpha
        image_cs = cv2.convertScaleAbs(channel, alpha=alpha, beta=beta)
        return image_cs

    def channels_stretching(self):
        R, G, B = cv2.split(self.image)
        R_s = self.Histogram_stretching(R)
        G_s = self.Histogram_stretching(G)
        B_s = self.Histogram_stretching(B)
        rgb_image = cv2.merge([R_s, G_s, B_s])

        H, S, V = cv2.split(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV))
        S_s = self.Histogram_stretching(S)
        V_s = self.Histogram_stretching(V)

        hsv_image = cv2.cvtColor(cv2.merge([H, S_s, V_s]), cv2.COLOR_HSV2RGB)
        return hsv_image


class auto_contrast:
    def __init__(self, img):
        self.image = img

    def power_transformation(self):
        # Get the information of the incoming image type.
        info = np.iinfo(self.image.dtype)
        # normalize the data to 0 - 1
        normalized = self.image.stype(np.float64) / info.max
        powered_img = cv2.pow(normalized, 1.2)
        powered_img = 255 * powered_img   # Now scale by 255
        powered_img = powered_img.astype(np.uint8)
        return powered_img


class MinimalPublisher_Subscriber(Node):
    def __init__(self):
        super().__init__('minimal_Publisher_Subscriber')
        sub1_topic_name = '~/left_raw/image_rect_color'
        pub1_topic_name = 'enhanced_frame_z'

        sub2_topic_name = 'lowlight_camera'
        pub2_topic_name = 'enhanced_frame_l'

        self.ZEDsub = 0
        self.ZEDpub = 0
        self.LLsub = 0
        self.LLpub = 0

        self.w = 500
        self.h = 500
        self.bridge = CvBridge()
        self.initial_enhanced_fram = np.zeros((self.w, self.h, 3), np.uint8)
        self.enhanced_frame_Z = np.zeros((self.w, self.h, 3), np.uint8)
        self.enhanced_frame_L = np.zeros((self.w, self.h, 3), np.uint8)
        self.timer_period = 0.1
    # ********************* subscribe raw frame *****************************
        self.subscribe(sub1_topic_name, self.listener_callback_Z)
        self.subscribe(sub2_topic_name, self.listener_callback_L)
    # *********************  puplish enhanced fram ***************************
        self.publisher1_ = self.create_publisher(Image, pub1_topic_name, 10)
        self.timer1 = self.create_timer(self.timer_period,
                                        self.timer_callback_Z)

        self.publisher2_ = self.create_publisher(Image, pub2_topic_name, 10)
        self.timer2 = self.create_timer(self.timer_period,
                                        self.timer_callback_L)

    def timer_callback_Z(self):
        self.publisher1_.publish(
            self.bridge.cv2_to_imgmsg(np.array(self.enhanced_frame_Z), "bgr8"))
        self.get_logger().info(
            'Publishing ZED camera frame: {}'.format(self.ZEDpub))
        self.ZEDpub += 1

    def timer_callback_L(self):
        self.publisher2_.publish(
            self.bridge.cv2_to_imgmsg(np.array(self.enhanced_frame_L), "bgr8"))
        self.get_logger().info(
            'Publishing low light camera frame: {}'.format(self.LLpub))
        self.LLpub += 1

    def enhance(self, cv_img):
        """
        Uncomment the following 3 code lines
        and comment the fourth one to see the effect of the power
        """
        # power_adjustment = auto_contrast(frame)
        # powered_img = power_adjustment.power_transformation()
        # color_adjustment = color_correction(powered_img)
        color_adjustment = color_correction(cv_img)
        enhanced_frame = color_adjustment. channels_stretching()

        return enhanced_frame

    def listener_callback_Z(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.enhanced_frame_Z = self.enhance(cv_image)
        self.get_logger().info(
            'subscribing ZED camera  {}'.format(self.ZEDsub))
        self.ZEDsub += 1

    def listener_callback_L(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.enhanced_frame_L = self.enhance(cv_image)
        self.get_logger().info(
            'subscribing low light camera{}'.format(self.LLsub))
        self.LLsub += 1

    def subscribe(self, sub_topic_name, listener_callback):
        self.subscription = self.create_subscription(
            Image, sub_topic_name, listener_callback, 10)
        self.subscription


def main(args=None):
    rclpy.init(args=args)

    minimal_Publisher_Subscriber = MinimalPublisher_Subscriber()

    rclpy.spin(minimal_Publisher_Subscriber)

    minimal_Publisher_Subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
