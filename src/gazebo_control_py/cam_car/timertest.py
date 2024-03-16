import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MyNode(Node):

    def __init__(self):
        super().__init__('my_node_name')

        self.publisher = self.create_publisher(String, 'output_topic', 10)
        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.callback,
            10)

    def callback(self, msg):
        self.get_logger().info("Recieved: %s" % msg.data)
        self.publisher.publish(msg)
        
if __name__ == '__main__':
    rclpy.init()
    my_node = MyNode()
    r = my_node.create_rate(10)
    start = time.time()
    for i in range(10):
        print(f"do some {i}")
        time.sleep(1) #休眠1s
        rclpy.spin_once(my_node)
        r.sleep() #调用一次rate.sleep()
    end = time.time()
    print(f'循环10次，耗时：{end-start}')

    rclpy.spin(my_node)
    my_node.destroy_node()  # cleans up pub-subs, etc
    rclpy.shutdown()