import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import pi
from std_msgs.msg import String
import threading

class MyNode(Node):

    def __init__(self):
        super().__init__('my_node_name')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            String,
            'input_topic',
            self.callback,
            10)
        rate = 50
        self._rate1 = self.create_rate(50)
        r = self.create_rate(rate)
        
        # 线速度
        linear_speed = 0.2 
        # 目标距离
        goal_distance = 1.0
        # 到达目标的时间
        linear_duration = goal_distance / linear_speed
        # 角速度 1.0rad/s
        angular_speed = 1.0 
        # 转角为Pi(180 degrees)
        goal_angle = pi
        # How long should it take to rotate?
        angular_duration = goal_angle / angular_speed

        # Loop through the two legs of the trip  
        for i in range(2):
            # Initialize the movement command
            move_cmd = Twist()

            # Set the forward speed
            move_cmd.linear.x = linear_speed
            # 机器人向前运动，延时一定时间
            ticks = int(linear_duration * rate)
            for t in range(ticks):
                self.publisher.publish(move_cmd)
                
                

            # 发送一个空的Twist消息是机器人停止
            move_cmd = Twist()
            self.publisher.publish(move_cmd)
            move_cmd.angular.z = angular_speed
            # 机器人开始旋转，延时一定时间使机器人转180度
            ticks = int(goal_angle * rate)
            for t in range(ticks):
                self.publisher.publish(move_cmd)
                print("wake")

            # 停下来
            move_cmd = Twist()
            self.publisher.publish(move_cmd)
            # r.sleep()

        # 循环两次之后停止
        self.publisher.publish(Twist())

        thread = threading.Thread(target=self.test, args=(self), daemon=True)
        thread.start()


        
        # 定义在/cmd_vel Topic中发布Twist消息，控制机器人速度
        # self.cmd_vel = rclpy.Publisher('/cmd_vel', Twist, queue_size=1) 
        # rate = 50 
        # 设置更新频率为50HZ
        # r = rclpy.Rate(rate) 
        
    def callback(self, msg):
        self.get_logger().info("Recieved: %s" % msg.data)
        self.publisher.publish(msg)

    def test(self):
        ticks = int(pi * 50)
        move_cmd = Twist()
        self.publisher.publish(move_cmd)
        move_cmd.angular.z = 1.0
        for t in range(ticks):
            self.publisher.publish(move_cmd)
            print("spin")
            self._rate1.sleep()
            
            

if __name__ == '__main__':
    rclpy.init()
    my_node = MyNode()
    rclpy.spin(my_node)
    my_node.destroy_node()  # cleans up pub-subs, etc
    rclpy.shutdown()
