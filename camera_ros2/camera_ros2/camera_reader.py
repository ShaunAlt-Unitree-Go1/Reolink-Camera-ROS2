# =============================================================================
# Camera Reader
# Created by: Shaun Altmann (saltmann@deakin.edu.au)
# =============================================================================
'''
Camera Reader
-
Reads the data from a RTSP stream, and publishes the data as a ROS
`sensor_msgs.msg.Image` message.
'''
# =============================================================================

# =============================================================================
# Imports
# =============================================================================

# used for system arguments
import argparse

# used for converting opencv images to `sensor_msgs.msg.Image`
from cv_bridge import CvBridge # type: ignore

# used for opencv2 image processing
import cv2 # type: ignore

# used for logging data
import logging

# used for reading environment variables
import os

# used for running ros2
import rclpy # type: ignore

# used for creating a ros2 node
from rclpy.node import Node # type: ignore

# used for regex string parsing
import regex as re # type: ignore

# used for publishing `sensor_msgs.msg.Image` messages
from sensor_msgs.msg import Image # type: ignore

# used for type-hinting
from typing import (
    Any, # any type
    Optional, # nullable data type
    Tuple, # tuple data type
)


# =============================================================================
# Camera Reader Node
# =============================================================================
class CameraReader(Node):
    '''
    Camera Reader Node
    -
    Reads the data from a RTSP stream, and publishes it as a ROS message.

    Fields
    -
    - _bridge : `CvBridge`
        - The bridge used to convert OpenCV2 images into ROS messages.
    - _camera : `cv2.VideoCapture`
        - The OpenCV2 video stream being read from the RTSP stream.
    - _frame_num : `int`
        - The number of frames that have been published.
    - _pub : `rclpy.publisher.Publisher`
        - The ROS publisher used to publish the camera data.
    - _resize : `tuple[int, int] | None`
        - If not `None`, sets the size that the camera data will be resized to.
    - conn_rtsp : `str`
        - Readonly.
        - The RTSP connection string.
    - logger : `logging.Logger`
        - Readonly.
        - The logger for this node.
    - param_camera_ip : `str`
        - Readonly.
        - The IP address of the Camera.
    - param_camera_port : `int`
        - Readonly.
        - The Port of the Camera.
    - param_camera_pwd : `str`
        - Readonly.
        - The Password of the Camera.
    - param_camera_suffix : `str`
        - Readonly.
        - The suffix of the Camera RTSP connection string.
    - param_camera_uid : `str`
        - Readonly.
        - The Username of the Camera.
    - param_publish_rate : `float`
        - Readonly.
        - The rate at which the camera data will be published.
    - param_resize : `tuple[int, int]`
        - Readonly.
        - The (width, height) to resize the image data to.
    - param_topic : `str`
        - Readonly.
        - The topic to publish the camera data to.

    Static Fields
    -
    - DEFAULT_CAM_PORT : `int`
        - The default port to use for the RTSP connection string.
    - DEFAULT_CAM_SUFF : `str`
        - The default suffix to use for the RTSP connection string.
    - DEFAULT_PUB_RATE : `float`
        - The default rate with which to publish the camera data.
    - DEFAULT_RESIZE : `tuple[int, int]`
        - The default (width, height) to resize the image data to.
    - DEFAULT_TOPIC : `str`
        - The default topic to publish the camera data on.
    - VALIDATOR_REGEX_IP : `str`
        - Regex search pattern used to validate IP addresses.

    Methods
    -
    - __init__(...) : `None`
        - Constructor Method.
        - Creates a new `CameraReader` object.
    - _publish() : `None`
        - Instance Method.
        - Publishes the camera data as a ROS message.
    '''

    # =============
    # Static Fields
    DEFAULT_CAM_PORT = 554
    ''' The default port to use for the RTSP connection string. '''
    DEFAULT_CAM_SUFF = 'h264Preview_01_Main'
    ''' The default suffix to use for the RTSP connection string. '''
    DEFAULT_RESIZE = (640, 460)
    ''' The default (width, height) to resize the image data to. '''
    DEFAULT_PUB_RATE = 20.0
    ''' The default rate with which to publish the camera data. '''
    DEFAULT_TOPIC = 'camera_image'
    ''' The default topic to publish the camera data on. '''
    VALIDATE_REGEX_IP = (
        r'^(?:(25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9][0-9]|[0-9])(\.(?!$)|$))' \
        + r'{4}$'
    )
    ''' Regex search pattern used to validate IP addresses. '''

    # ====================
    # Method - Constructor
    def __init__(self) -> None:
        '''
        Camera Reader Constructor
        -
        Creates a new `CameraReader` object.

        Parameters
        -
        None

        Returns
        -
        None
        '''

        # initialize node
        super().__init__('camera_reader')

        # log start of creation
        self.logger.info('Creating CameraReader()')

        # set ros parameters
        self.declare_parameter('camera_ip', '')
        self.declare_parameter('camera_port', CameraReader.DEFAULT_CAM_PORT)
        self.declare_parameter('camera_pwd', '')
        self.declare_parameter('camera_suffix', CameraReader.DEFAULT_CAM_SUFF)
        self.declare_parameter('camera_uid', '')
        self.declare_parameter('publish_rate', CameraReader.DEFAULT_PUB_RATE)
        self.declare_parameter('resize', CameraReader.DEFAULT_RESIZE)
        self.declare_parameter('topic', CameraReader.DEFAULT_TOPIC)

        # creating the camera connection
        self._camera = cv2.VideoCapture(self.conn_rtsp)
        ''' The OpenCV2 video stream being read from the RTSP stream. '''

        # creating the image converter
        self._bridge = CvBridge()
        ''' The bridge used to convert OpenCV2 images into ROS messages. '''

        # creating the publisher
        self._pub = self.create_publisher(Image, self.param_topic, 20)
        ''' The ROS publisher used to publish the camera data. '''

        # creating the timer
        self.create_timer(1.0 / self.param_publish_rate, self._publish)

        # creating the frame counter
        self._frame_num: int = 0
        ''' The number of frames that have been published. '''

        # log creation
        self.logger.info(f'Created {self!r}')

    # =================================
    # Property - RTSP Connection String
    @property
    def conn_rtsp(self) -> str:
        ''' The RTSP connection string. '''
        return (
            f'rtsp://{self.param_camera_uid}:{self.param_camera_pwd}@' \
            + f'{self.param_camera_ip}:{self.param_camera_port}/' \
            + f'{self.param_camera_suffix}'
        )
    
    # ======================
    # Property - Node Logger
    @property
    def logger(self) -> logging.Logger:
        ''' The logger for this node. '''
        return self.get_logger()
    
    # ====================================
    # Property - ROS Parameter - Camera IP
    @property
    def param_camera_ip(self) -> str:
        ''' The IP address of the Camera. '''

        # get `camera_ip` parameter value
        val: str = (
            self.get_parameter('camera_ip') \
            .get_parameter_value() \
            .string_value
        )

        # validate the value (IP4 address)
        if re.search(CameraReader.VALIDATE_REGEX_IP, val):
            return val
        
        raise ValueError(
            'Attempted to get ROS parameter `camera_ip`, got invalid value ' \
            + f'{val!r}'
        )

    # ======================================
    # Property - ROS Parameter - Camera Port
    @property
    def param_camera_port(self) -> int:
        ''' The Port of the Camera. '''

        # get `camera_port` parameter value
        val: int = (
            self.get_parameter('camera_port') \
            .get_parameter_value() \
            .integer_value
        )

        # validate the value (not empty)
        if val:
            return val
        
        raise ValueError(
            'Attempted to get ROS parameter `camera_port`, got invalid value' \
            + f' {val!r}'
        )

    # ==========================================
    # Property - ROS Parameter - Camera Password
    @property
    def param_camera_pwd(self) -> str:
        ''' The Password of the Camera. '''

        # get `camera_pwd` parameter value
        val: str = (
            self.get_parameter('camera_pwd') \
            .get_parameter_value() \
            .string_value
        )

        # validate the value (not empty)
        if val:
            return val
        
        raise ValueError(
            'Attempted to get ROS parameter `camera_pwd`, got invalid value ' \
            + f'{val!r}'
        )

    # =============================================
    # Property - ROS Parameter - Camera RTSP Suffix
    @property
    def param_camera_suffix(self) -> str:
        ''' The suffix of the Camera RTSP connection string. '''

        # get `camera_suffix` parameter value
        val: str = (
            self.get_parameter('camera_suffix') \
            .get_parameter_value() \
            .string_value
        )

        # validate the value (not empty)
        if val:
            return val
        
        raise ValueError(
            'Attempted to get ROS parameter `camera_suffix`, got invalid ' \
            + f'value {val!r}'
        )

    # ==========================================
    # Property - ROS Parameter - Camera Username
    @property
    def param_camera_uid(self) -> str:
        ''' The Username of the Camera. '''

        # get `camera_uid` parameter value
        val: str = (
            self.get_parameter('camera_uid') \
            .get_parameter_value() \
            .string_value
        )

        # validate the value (not empty)
        if val:
            return val
        
        raise ValueError(
            'Attempted to get ROS parameter `camera_uid`, got invalid value ' \
            + f'{val!r}'
        )

    # =======================================
    # Property - ROS Parameter - Publish Rate
    @property
    def param_publish_rate(self) -> float:
        ''' The rate at which the camera data will be published. '''

        # get `publish_rate` parameter value
        val: float = (
            self.get_parameter('publish_rate') \
            .get_parameter_value() \
            .double_value
        )

        # validate the value (not empty)
        if val:
            return val
        
        raise ValueError(
            'Attempted to get ROS parameter `publish_rate`, got invalid ' \
            + f'value {val!r}'
        )
    
    # =======================================
    # Property - ROS Parameter - Image Resize
    @property
    def param_resize(self) -> tuple[int, int]:
        ''' The (width, height) to resize the image data to. '''

        # get `resize` parameter value
        val: list[int] = (
            self.get_parameter('resize') \
            .get_parameter_value() \
            .integer_array_value
        )

        # validate the value (not empty)
        if (val) and (len(val) == 2):
            return (val[0], val[1])
        
        raise ValueError(
            'Attempted to get ROS parameter `resize`, got invalid ' \
            + f'value {val!r}'
        )

    # ================================
    # Property - ROS Parameter - Topic
    @property
    def param_topic(self) -> str:
        ''' The topic to publish the camera data to. '''

        # get `topic` parameter value
        val: str = (
            self.get_parameter('topic') \
            .get_parameter_value() \
            .string_value
        )

        # validate the value (not empty)
        if val:
            return val
        
        raise ValueError(
            'Attempted to get ROS parameter `topic`, got invalid value ' \
            + f'{val!r}'
        )

    # ===========================
    # Method - Publish Image Data
    def _publish(self) -> None:
        '''
        Publish Image Data
        -
        Publishes the camera data as a ROS message.

        Parameters
        -
        None

        Returns
        -
        None
        '''

        # get image data
        success, frame = self._camera.read()

        # check if frame was read successfully
        if not success:
            self.logger.warning('Failed to read image data')
            return None
        
        # resize image
        frame = cv2.resize(
            frame,
            self.param_resize,
            interpolation = cv2.INTER_CUBIC
        )

        # publish image data
        self._pub.publish(self._bridge.cv2_to_imgmsg(frame))

        # log publish
        self.logger.info(f'Published frame {self._frame_num}')

        # increment frame number
        self._frame_num += 1

        return None


# =============================================================================
# Main Function
# =============================================================================
def main(args = None) -> None:
    # initialize rclpy
    rclpy.init(args = args)

    # create camera reader instance
    camera_reader = CameraReader()

    # spin until shutdown
    rclpy.spin(camera_reader)

    # shutdown the node
    camera_reader.destroy_node()

    # shutdown the ROS system
    rclpy.shutdown()


# =============================================================================
# Main Function Call
# =============================================================================
if __name__ == '__main__':
    main()


# =============================================================================
# End of File
# =============================================================================
