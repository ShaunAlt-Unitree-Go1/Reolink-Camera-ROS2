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
    - _conn_ip : `str`
        - The IP address of the RTSP connection string.
    - _conn_port : `str`
        - The port of the RTSP connection string.
    - _conn_pwd : `str`
        - The password of the RTSP connection string.
    - _conn_suffix : `str`
        - The suffix of the RTSP connection string.
    - _conn_uid : `str`
        - The username of the RTSP connection string.
    - _fps : `float`
        - The framerate with which to publish the camera data.
    - _frame_num : `int`
        - The number of frames that have been published.
    - _pub : `rclpy.publisher.Publisher`
        - The ROS publisher used to publish the camera data.
    - _resize : `tuple[int, int] | None`
        - If not `None`, sets the size that the camera data will be resized to.
    - _topic : `str`
        - The topic to publish the camera data on.
    - conn_rtsp : `str`
        - Readonly.
        - The RTSP connection string.
    - logger : `logging.Logger`
        - Readonly.
        - The logger for this node.

    Static Fields
    -
    - DEFAULT_CONN_PORT : `int`
        - The default port to use for the RTSP connection string.
    - DEFAULT_CONN_SUFFIX : `str`
        - The default suffix to use for the RTSP connection string.
    - DEFAULT_FPS : `float`
        - The default framerate with which to publish the camera data.
    - DEFAULT_TOPIC : `str`
        - The default topic to publish the camera data on.

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
    DEFAULT_CONN_PORT = 554
    ''' The default port to use for the RTSP connection string. '''
    DEFAULT_CONN_SUFFIX = 'h264Preview_01_Main'
    ''' The default suffix to use for the RTSP connection string. '''
    DEFAULT_FPS = 20.0
    ''' The default frame rate with which to publish the camera data. '''
    DEFAULT_TOPIC = 'camera_image'
    ''' The default topic to publish the camera data on. '''

    # ====================
    # Method - Constructor
    def __init__(
            self,
            conn_uid: Optional[str] = None,
            conn_pwd: Optional[str] = None,
            conn_ip: Optional[str] = None,
            conn_port: Optional[int] = None,
            conn_suffix: Optional[str] = None,
            fps: Optional[float] = None,
            resize: Optional[Tuple[int, int]] = None,
            topic: Optional[str] = None
    ) -> None:
        '''
        Camera Reader Constructor
        -
        Creates a new `CameraReader` object.

        Parameters
        -
        - conn_uid : `str | None`
            - The username of the RTSP connection string. If `None` then the
                environment variable `CONN_UID` will be used.
        - conn_pwd : `str | None`
            - The password of the RTSP connection string. If `None` then the
                environment variable `CONN_PWD` will be used.
        - conn_ip : `str | None`
            - The IP of the RTSP connection string. If `None`, then the 
                environment variable `CONN_IP` will be used.
        - conn_port : `int | None`
            - The port of the RTSP connection string. If `None`, then the
                `DEFAULT_CONN_PORT` will be used.
        - conn_suffix : `str | None`
            - The suffix of the connection string. If `None`, then the
                `DEFAULT_CONN_SUFFIX` will be used.
        - fps : `float | None`
            - The framerate with which to publish the camera data. If `None`,
                then the `DEFAULT_FPS` will be used.
        - resize : `tuple[int, int] | None`
            - The size to resize the camera image data to. If `None`, then the
                camera image will not be resized.
        - topic : `str | None`
            - The topic to publish the camera data on. If `None`, then the
                `DEFAULT_TOPIC` will be used.

        Returns
        -
        None
        '''

        # initialize node
        super().__init__('camera_reader')

        # log start of creation
        self.logger.info(
            f'Creating CameraReader(conn_uid={conn_uid}, conn_pwd={conn_pwd}' \
            + f', conn_ip={conn_ip}, conn_port={conn_port}, conn_suffix=' \
            + f'{conn_suffix}, fps={fps}, resize={resize}, topic={topic})'
        )

        # creating the connection string
        self._conn_ip: str = ''
        ''' The IP address of the RTSP connection string. '''
        if conn_ip: self._conn_ip = conn_ip
        else:
            try: self._conn_ip = os.environ['CONN_IP']
            except:
                raise RuntimeError(
                    'Failed to initialize `CameraReader` with undefined ' \
                    + '`conn_ip` and no `CONN_IP` environment variable set.'
                )

        self._conn_pwd: str = ''
        ''' The password of the RTSP connection string. '''
        if conn_pwd: self._conn_pwd = conn_pwd
        else:
            try: self._conn_pwd = os.environ['CONN_PWD']
            except:
                raise RuntimeError(
                    'Failed to initialize `CameraReader` with undefined ' \
                    + '`conn_pwd` and no `CONN_PWD` environment variable set.'
                )

        self._conn_uid: str = ''
        ''' The username of the RTSP connection string. '''
        if conn_uid: self._conn_uid = conn_uid
        else:
            try: self._conn_uid = os.environ['CONN_UID']
            except:
                raise RuntimeError(
                    'Failed to initialize `CameraReader` with undefined ' \
                    + '`conn_uid` and no `CONN_UID` environment variable set.'
                )

        self._conn_port: int = CameraReader.DEFAULT_CONN_PORT
        ''' The port of the RTSP connection string. '''
        if conn_port: self._conn_port = conn_port

        self._conn_suffix: str = CameraReader.DEFAULT_CONN_SUFFIX
        ''' The suffix of the RTSP connection string. '''
        if conn_suffix: self._conn_suffix = conn_suffix

        # creating the camera connection
        self._camera = cv2.VideoCapture(self.conn_rtsp)
        ''' The OpenCV2 video stream being read from the RTSP stream. '''
        self._resize: Optional[Tuple[int, int]] = resize
        ''' If not `None`, sets the size that the camera data will be resized
            to. '''

        # creating the image converter
        self._bridge = CvBridge()
        ''' The bridge used to convert OpenCV2 images into ROS messages. '''

        # creating the publisher
        self._topic = CameraReader.DEFAULT_TOPIC
        ''' The topic used to publish the camera data on. '''
        if topic: self._topic = topic
        self._pub = self.create_publisher(
            Image,
            self._topic,
            20
        )
        ''' The ROS publisher used to publish the camera data. '''

        # creating the timer
        self._fps: float = 0.0
        ''' The framerate with which to publish the camera data. '''
        self.create_timer(1.0 / self._fps, self._publish)

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
            f'rtsp://{self._conn_uid}:{self._conn_pwd}@{self._conn_ip}:' \
            + f'{self._conn_port}/{self._conn_suffix}'
        )
    
    # ======================
    # Property - Node Logger
    @property
    def logger(self) -> logging.Logger:
        ''' The logger for this node. '''
        return self.get_logger()
    
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
        
        # resize image if necessary
        if self._resize:
            frame = cv2.resize(
                frame,
                self._resize,
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
def main(
        conn_uid: Any = None,
        conn_pwd: Any = None,
        conn_ip: Any = None,
        conn_port: Any = None,
        conn_suffix: Any = None,
        fps: Any = None,
        resize: Any = None,
        topic: Any = None,
) -> None:
    # initialize rclpy
    rclpy.init(args = args)

    # validate arguments
    try:
        if conn_uid is not None: # str | None
            conn_uid = str(conn_uid)
        if conn_pwd is not None: # str | None
            conn_pwd = str(conn_pwd)
        if conn_ip is not None: # str | None
            conn_ip = str(conn_ip)
        if conn_port is not None: # int | None
            conn_port = int(conn_port)
        if conn_suffix is not None: # str | None
            conn_suffix = str(conn_suffix)
        if fps is not None: # float | None
            fps = float(fps)
        if resize is not None: # tuple[int, int] | None
            resize = (int(resize[0]), int(resize[1]))
        if topic is not None: # str | None
            topic = str(topic)
    except Exception as e:
        raise ValueError(
            f'Unable to determine the valud of the given parameters: {e!r}'
        )
    
    # create camera reader instance
    camera_reader = CameraReader(
        conn_uid = conn_uid,
        conn_pwd = conn_pwd,
        conn_ip = conn_ip,
        conn_port = conn_port,
        conn_suffix = conn_suffix,
        fps = fps,
        resize = resize,
        topic = topic,
    )

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
    # create python argument parser
    parser = argparse.ArgumentParser(description = 'Camera Reader')

    # add arguments
    parser.add_argument(
        '--conn_uid',
        type = str,
        help = 'The username for the RTSP connection string'
    )
    parser.add_argument(
        '--conn_pwd',
        type = str,
        help = 'The password for the RTSP connection string'
    )
    parser.add_argument(
        '--conn_ip',
        type = str,
        help = 'The IP address for the RTSP connection string'
    )
    parser.add_argument(
        '--conn_port',
        type = int,
        help = 'The port for the RTSP connection string'
    )
    parser.add_argument(
        '--conn_suffix',
        type = str,
        help = 'The suffix for the RTSP connection string'
    )
    parser.add_argument(
        '--fps',
        type = float,
        help = 'The framerate to publish the camera data at'
    )
    parser.add_argument(
        '--resize',
        type = lambda x: tuple(map(int, x.split(','))),
        help = 'The size to resize the camera data to (width, height)'
    )
    parser.add_argument(
        '--topic',
        type = str,
        help = 'The ROS topic to publish the camera data on'
    )

    # parse arguments and run main function
    args = parser.parse_args()
    main(
        conn_uid = args.conn_uid,
        conn_pwd = args.conn_pwd,
        conn_ip = args.conn_ip,
        conn_port = args.conn_port,
        conn_suffix = args.conn_suffix,
        fps = args.fps,
        resize = args.resize,
        topic = args.topic,
    )


# =============================================================================
# End of File
# =============================================================================
