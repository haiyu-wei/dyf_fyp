import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO
import numpy as np

from launch.actions import ExecuteProcess
from launch import LaunchDescription

gazebo_world_path = 'src/world/cafe.world'

# Load the YOLOv8 model
model = YOLO('yolov8n.pt')
model.classes = [2,5]


class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')

    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, '/camera1/image_raw', self.listener_callback, 10)
    self.subscription # prevent unused variable warning

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving video frame')

    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
    image = current_frame

    # Object Detection
    result = model(image)[0]
    boxes = result.boxes  # Boxes object for bounding box outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs
    # result.show()  # display to screen

    # Calculate Center Coordinate
    # print(boxes.xyxy)
    xyxy = boxes.xyxy[0]
    p1, p2 = (int(xyxy[0]),int(xyxy[1])),(int(xyxy[2]),int(xyxy[3]))
    center = (((p2[0] - p1[0]) / 2 +p1[0]) , ((p2[1] - p1[1]) / 2 + p1[1]))
    print(center)
    # result.save(filename='result.jpg')  # save to disk

    # results = model.predict(image, classes=[2,5])
    # img = results[0].plot()



    # Show Results
    img = result.plot()
    cv2.circle(self,((p1[0] + p2[0])//2, (p1[1] + p2[1])//2), self,(0, 0, 255), self)
    cv2.imshow('Detected Frame', img)
    cv2.waitKey(1)

def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Start Gazebo server
  ld = LaunchDescription()
  start_gazebo_cmd =  ExecuteProcess(
    cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
    output='screen')
  ld.add_action(start_gazebo_cmd)

  # Create the node
  image_subscriber = ImageSubscriber()

  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()


if __name__ == '__main__':
  main()
