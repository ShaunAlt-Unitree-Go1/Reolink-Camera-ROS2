# =============================================================================
# Camera Streamer
# Created by: Shaun Altmann (saltmann@deakin.edu.au)
# =============================================================================
'''
Camera Streamer
-
Streams image data from a topic being published to by a `CameraReader`.
'''
# =============================================================================

# =============================================================================
# Imports
# =============================================================================

# used for opencv2 image processing
import cv2 # type: ignore

# used for converting `sensor_msgs.msg.Image` into opencv images
from cv_bridge import CvBridge # type: ignore

# used for logging data
import logging

# used for running ros2
import rclpy # type: ignore

# used for creating a ros2 node
from rclpy.node import Node # type: ignore

# used for publishing `sensor_msgs.msg.Image` messages
from sensor_msgs.msg import Image # type: ignore


# =============================================================================
# Camera Streamer Node
# =============================================================================
class CameraStreamer(Node):
    '''
    Camera Streamer Node
    -
    Reads the data from the `CameraReader` node output, and displays it as an
    OpenCV video.

    Fields
    -
    - _bridge : `CvBridge`
        - The bridge used to convert ROS messages to OpenCV2 images.
    - logger : `logging.Logger`
        - Readonly.
        - The logger for this node.
    - param_title : `str`
        - Readonly.
        - The title of the video stream.
    - param_topic : `str`
        - Readonly.
        - The topic to read the camera data from.

    Static Fields
    -
    - DEFAULT_TITLE : `str`
        - The default title to use for the video stream.
    - DEFAULT_TOPIC : `str`
        - The default topic to read the camera data from.

    Methods
    -
    - __init__() : `None`
        - Constructor Method.
        - Creates a new `CameraStreamer` object.
    - __repr__() : `str`
        - Official Representation Method.
        - Returns the official string representation of this object.
    - __str__() : `str`
        - Informal Representation Method.
        - Returns a short, informal string representation of this object.
    - _read_image(msg) : `None`
        - Instance Method.
        - Reads the camera data from the subscribed topic, and displays it as a
            video.
    '''

    # =============
    # Static Fields
    DEFAULT_TITLE = 'Camera Image'
    ''' The default title to use for the video stream. '''
    DEFAULT_TOPIC = 'camera_image'
    ''' The default topic to read the camera data from. '''

    # ====================
    # Method - Constructor
    def __init__(self) -> None:
        '''
        Camera Streamer Constructor
        -
        Creates a new `CameraStreamer` object.

        Parameters
        -
        None

        Returns
        -
        None
        '''

        # initialize node
        super().__init__('camera_streamer')

        # log start of creation
        self.logger.debug('Creating CameraStreamer()')

        # set ros parameters
        self.declare_parameter('title', CameraStreamer.DEFAULT_TITLE)
        self.declare_parameter('topic', CameraStreamer.DEFAULT_TOPIC)

        # create the image converter
        self._bridge = CvBridge()
        ''' The bridge used to convert ROS messages to OpenCV2 images. '''

        # create topic subscriber
        self.create_subscription(Image, self.param_topic, self._read_image, 20)

        # log creation
        self.logger.info(f'Created {self!r}')

    # ================================
    # Method - Official Representation
    def __repr__(self) -> str:
        ''' Official Representation Method. '''
        output = 'CameraStreamer('
        for val in [
                'logger',
                'param_title',
                'param_topic',
        ]:
            try:
                output += f'\n\t{val} = {getattr(self, val)},'
            except Exception as e:
                output += f'\n\t{val} <ERROR> = {e}'
        return f'{output}\n)'

    # ================================
    # Method - Informal Representation
    def __str__(self) -> str:
        ''' Informal Representation Method. '''
        output = 'CameraStreamer('
        for val in [
                'param_title',
                'param_topic',
        ]:
            try:
                output += f' {val} = {getattr(self, val)},'
            except Exception as e:
                output += f' {val} <ERROR> = {e}'
        return f'{output} )'

    # ======================
    # Property - Node Logger
    @property
    def logger(self) -> logging.Logger:
        ''' The logger for this node. '''
        return self.get_logger()
    
    # ================================
    # Property - ROS Parameter - Title
    @property
    def param_title(self) -> str:
        ''' The title of the video stream. '''

        # get `title` parameter value
        val: str = (
            self.get_parameter('title') \
            .get_parameter_value() \
            .string_value
        )

        # validate the value (not empty)
        if val:
            return val
        
        raise ValueError(
            'Attempted to get ROS parameter `title`, got invalid value ' \
            + f'{val!r}'
        )
    
    # ================================
    # Property - ROS Parameter - Topic
    @property
    def param_topic(self) -> str:
        ''' The topic to read the camera data from. '''

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
    
    # ========================
    # Method - Read Image Data
    def _read_image(self, msg: Image) -> None:
        '''
        Read Image Data
        -
        Reads the camera data from the subscribed topic, and displays it as a
        video.

        Parameters
        -
        - msg : `sensor_msgs.msg.Image`
            - Image message data received from the topic.

        Returns
        -
        None
        '''

        # convert message to image + display
        cv2.imshow(self.param_title, self._bridge.imgmsg_to_cv2(msg))
        cv2.waitKey(1)


# =============================================================================
# Main Function
# =============================================================================
def main(args = None) -> None:
    # initialize rclpy
    rclpy.init(args = args)

    # create camera reader instance
    camera_streamer = CameraStreamer()

    # spin until shutdown
    rclpy.spin(camera_streamer)

    # shutdown the node
    camera_streamer.destroy_node()

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
