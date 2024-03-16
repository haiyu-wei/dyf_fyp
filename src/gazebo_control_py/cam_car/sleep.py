import time
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ultralytics import YOLO

from geometry_msgs.msg import Twist
from math import pi
import numpy as np
import threading

gazebo_world_path = 'src/world/cafe.world'

# Load the YOLOv8 model
model = YOLO('yolov8n.pt')
model.classes = [2,5]
image_center = (320,320)
linear_speed = 0.2
init_area = -1


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

    # Publisher
    self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    self._rate1 = self.create_rate(10)

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
    angular_speed = 0.5 
    move_cmd = Twist()
    try:
        xyxy = boxes.xyxy[0]
        p1, p2 = (int(xyxy[0]),int(xyxy[1])),(int(xyxy[2]),int(xyxy[3]))
        center = (((p2[0] - p1[0]) / 2 +p1[0]) , ((p2[1] - p1[1]) / 2 + p1[1]))
        print(center)
        #judge the position
        if(center[0]<image_center[0]):
            print("object is on the left hand side")
            move_cmd.angular.z = angular_speed
            self.publisher.publish(move_cmd)
        else:
            print("object is on the right hand side")
            move_cmd.angular.z = -angular_speed
            self.publisher.publish(move_cmd)

        area = (p2[0] - p1[0])*(p2[1] - p1[1])

        #initialize area
        global init_area
        if(init_area==-1): 
            init_area = area

        if(area*0.7 > init_area):
            print("need go back")
            move_cmd = Twist()
            # Set the forward speed
            move_cmd.linear.x = -linear_speed
            self.publisher.publish(move_cmd)
        else:
            if(area<init_area*0.7):
                print("need go forward")
                move_cmd = Twist()
                # Set the forward speed
                move_cmd.linear.x = linear_speed
                self.publisher.publish(move_cmd)
            else:
                move_cmd = Twist()
                # Stop
                self.publisher.publish(move_cmd)
            
        
        print(area)

        #judge the distance
        
    except:
      print("no detection")

    # Show Results
    img = result.plot()
    cv2.imshow('Detected Frame', img)
    cv2.waitKey(1)

        
        
     



def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  image_subscriber = ImageSubscriber()

  # Spin the node so the callback function is called.
#   rclpy.spin(image_subscriber)
#   rclpy.spin_once(image_subscriber)
  r = image_subscriber.create_rate(10)
  start = time.time()
  for i in range(1000):
        print(f"do some {i}")
        time.sleep(0.1) #休眠1s
        rclpy.spin_once(image_subscriber)
        # r.sleep() #调用一次rate.sleep()
        end = time.time()

  print(f'循环10次，耗时：{end-start}')

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()


if __name__ == '__main__':
  main()
