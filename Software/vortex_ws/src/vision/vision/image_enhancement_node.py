"""
This is an image enhancement publisher_subscriber node.

It contains two classes for underwater color correction and conrast correction
by using power function  which enhance the subscribed frame
and publish the result.
"""
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
from sensor_msgs.msg import Image
import cv2
import numpy as np


class color_correction:
    """
    A class to represent some functions applied to the input image.

    it correct the color absorption and attenuation due to under water depth.

    ...

    Attributes
    ----------
    imag : numpy.ndarray
        input image

    Methods
    -------
    stretch_Histogram(channel, clip_hist_percent=1.5):
        Return image channel after stretching it's histogram.
    ImgChannels_correction():
        Apply stretch_Histogram function at image channels
        to correct the image color and intensity.
    """

    def __init__(self, imag):
        """
        Construct all the necessary attributes for the color correction object.

        Parameters
        ----------
            imag : 3D array
        """
        self.image = imag

    def stretch_Histogram(self, channel, clip_hist_percent=1.5):
        """
        Return image channel after stretching it's histogram.

        Parameters
        ----------
        channel : numpy.ndarray
            image channel 2D array
        clip_hist_percent : decimal number
            cumulative curve threshold  of cutting

        Returns
        -------
         channel_HS (numpy.ndarray): 2D array
            after applying the histogram stretching
        """
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
        channel_HS = cv2.convertScaleAbs(channel, alpha=alpha, beta=beta)
        return channel_HS

    def ImgChannels_correction(self):
        """
        Apply stretch_Histogram function at image channels.

        it correct the image color and intensity.

        This function splits the image to R,G,B channels and apply
        stretch_Histogram function at each channel,then it merge the output
        to construct the color enhanced image.This image also splitted into
        H,S,V channels to apply stretch_Histogram function at S and V channels
        and finally merge the output and return it.

        Parameters
        ----------
        None

        Returns
        -------
         hsv_image : numpy.ndarray
             image after applying the histogram stretching at some channels
        """
        R, G, B = cv2.split(self.image)
        R_s = self.stretch_Histogram(R)
        G_s = self.stretch_Histogram(G)
        B_s = self.stretch_Histogram(B)
        rgb_image = cv2.merge([R_s, G_s, B_s])

        H, S, V = cv2.split(cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV))
        S_s = self.stretch_Histogram(S)
        V_s = self.stretch_Histogram(V)

        hsv_image = cv2.cvtColor(cv2.merge([H, S_s, V_s]), cv2.COLOR_HSV2RGB)
        return hsv_image


class auto_contrast:
    """
    A class to represent a function to adjust the image power.

    ...

    Attributes
    ----------
    img : numpy.ndarray
        image after color correction

    Methods
    -------
    power_transformation():
         Apply opencv power  function to enhance the image contrast.
    """

    def __init__(self, img):
        """
        Construct all the necessary attributes for the auto_contrast object.

        Parameters
        ----------
            imag : 3D array
        """
        self.image = img

    def power_transformation(self):
        """
        Apply opencv power  function to enhance the image contrast.

        Parameters
        ----------
        None

        Returns
        -------
         powered_img (numpy.ndarray): 3D array
            after enhance the image power by specific factor=1.2
        """
        # Get the information of the incoming image type.
        info = np.iinfo(self.image.dtype)
        # normalize the data to 0 - 1
        normalized = self.image.stype(np.float64) / info.max
        powered_img = cv2.pow(normalized, 1.2)
        powered_img = 255 * powered_img   # Now scale by 255
        powered_img = powered_img.astype(np.uint8)
        return powered_img


class MinimalPublisher_Subscriber(Node):
    """
    A class to represent publisher_subscriber node.

    ...

    Attributes
    ----------
    sub1_topic_name : str
        topic name at which ZED camera published the raw frames
    pub1_topic_name : str
        topic name at which image enhancement node publish
        the enhanced frames of ZED camera
    sub2_topic_name : str
        topic name at which low light camera published the raw frames
    pub2_topic_name : str
        topic name at which image enhancement node publish
        the enhanced frames of low light camera camera
    ZEDsub : int
        counter for ZED camera subscribed frame
    ZEDpub : int
        counter for ZED camera subscribed frame
    LLsub : int
        counter for low light camera subscribed frame
    LLpub : int
        counter for low light camera published frame
    w : int
        image initial width
    h : int
        image initial hight
    enhanced_frame_Z : 3D array
        blank image to overwrite the frame
        of ZED camera after enhancement
    enhanced_frame_L : 3D array
        blank image to overwrite the frame
        of low light camera after enhancement
    timer_period : int
        publish frames every certain time period

    Methods
    -------
    z_timer_callback():
        publish enhanced frame of ZED camera
    l_timer_callback():
        publish enhanced frame of low light camera
    enhance(cv_img):
        Apply color correction class functions
        and auto contrast functions to enhance the input image.
    z_listener_callback(data):
        receive the data from ZED camera publisher node.
    l_listener_callback(data):
        receive the data from low light camera publisher node.
    subscribe(sub_topic_name, listener_callback):
        take the topic name which subscribes the raw frame from it
        and the callback function
    """

    def __init__(self):
        """
        Construct all the necessary attributes for MinimalPublisher_Subscriber.

        Parameters
        ----------
            None
        """
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
        self.enhanced_frame_Z = np.zeros((self.w, self.h, 3), np.uint8)
        self.enhanced_frame_L = np.zeros((self.w, self.h, 3), np.uint8)
        self.timer_period = 0.1
    # ********************* subscribe raw frame *****************************
        self.subscribe(sub1_topic_name, self.z_listener_callback)
        self.subscribe(sub2_topic_name, self.l_listener_callback)
    # *********************  puplish enhanced fram ***************************
        self.publisher1_ = self.create_publisher(Image, pub1_topic_name, 10)
        self.timer1 = self.create_timer(self.timer_period,
                                        self.z_timer_callback)

        self.publisher2_ = self.create_publisher(Image, pub2_topic_name, 10)
        self.timer2 = self.create_timer(self.timer_period,
                                        self.l_timer_callback)

    def np2ros(self, image):
        """
        convert image to Image ros message.

        Parameters
        ----------
        image : 3D array

        Returns
        -------
        msg : 1D array
        """
        msg = Image()
        msg.height, msg.width, msg.step = image.shape
        msg.data = image.reshape(-1).tolist()
        return msg

    def z_timer_callback(self):
        """
        publish enhanced_frame_Z after overwritten.

        by z_listener_callback function.

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        self.publisher1_.publish(
            self.np2ros(self.enhanced_frame_Z))
        self.get_logger().info(
            'Publishing ZED camera frame: {}'.format(self.ZEDpub))
        self.ZEDpub += 1

    def l_timer_callback(self):
        """
        publish enhanced_frame_L after overwritten.

        by l_listener_callback function.

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        self.publisher2_.publish(
            self.np2ros(self.enhanced_frame_L))
        self.get_logger().info(
            'Publishing low light camera frame: {}'.format(self.LLpub))
        self.LLpub += 1

    def enhance(self, cv_img):
        """
        Apply color correction class functions and auto contrast functions.

        to enhance the input image,
        Uncomment the following 3 code lines
        and comment the fourth one to see the effect of the power

        Parameters
        ----------
        cv_img: 3D numpy array
            converted ros message to numpy array

        Returns
        -------
        enhanced_frame (numpy.ndarray): 3D array
            image after applying enhancement techniques
        """
        # power_adjustment = auto_contrast(frame)
        # powered_img = power_adjustment.power_transformation()
        # color_adjustment = color_correction(powered_img)
        color_adjustment = color_correction(cv_img)
        enhanced_frame = color_adjustment. ImgChannels_correction()
        return enhanced_frame

    def z_listener_callback(self, msg):
        """
        receive the data from ZED camera publisher node.

        and overwrite the initialized enhanced_frame_Z with the result
        after the applying image enhancement techniques
        on the subscribed imag.

        Parameters
        ----------
        data :  1D array
            Image ros message

        Returns
        -------
        None
        """
        cv_image = np.array(msg.data).reshape(
            (msg.height, msg.width, msg.step))
        self.enhanced_frame_Z = self.enhance(cv_image)
        self.get_logger().info(
            'subscribing ZED camera  {}'.format(self.ZEDsub))
        self.ZEDsub += 1

    def l_listener_callback(self, msg):
        """
        receive the data from low light camera publisher node.

        and overwrite the initialized enhanced_frame_L with the result
        after the applying image enhancement techniques
        on the subscribed imag.

        Parameters
        ----------
        data :  1D array
            Image ros message

        Returns
        -------
        None
        """
        cv_image = np.array(msg.data).reshape(
            (msg.height, msg.width, msg.step))
        self.enhanced_frame_L = self.enhance(cv_image)
        self.get_logger().info(
            'subscribing low light camera{}'.format(self.LLsub))
        self.LLsub += 1

    def subscribe(self, sub_topic_name, listener_callback):
        """
        Create subscription which subscribe from the input sub_topic_name.

        and call the listener_callback.

        Parameters
        ----------
        sub_topic_name : str
            topic name at which image enhancement node
            publish the enhanced frames
        listener_callback:
            receive the data from specific publisher node.

        Returns
        -------
        None
        """
        self.subscription = self.create_subscription(
            Image, sub_topic_name, listener_callback, 10)
        self.subscription


def main(args=None):
    """
    spin minimal_Publisher_Subscriber .

    Parameters
    ----------
    None

    Returns
    -------
    None
    """
    rclpy.init(args=args)
    minimal_Publisher_Subscriber = MinimalPublisher_Subscriber()
    rclpy.spin(minimal_Publisher_Subscriber)
    minimal_Publisher_Subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
