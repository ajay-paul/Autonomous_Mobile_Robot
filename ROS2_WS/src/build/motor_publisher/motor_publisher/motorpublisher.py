import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Int32
from std_msgs.msg import Float64
#import RPi.GPIO as GPIO
import time
from rclpy.time import Time
from rclpy.duration import Duration



class MotorDriver(Node):
        def __init__(self):
                super().__init__('commander')
                self.pub = self.create_publisher(String, 'cmd_vel', 10)
                #timer_period = 0.5
                #self.timer = self.create_timer(timer_period, self.motor_callback)

                #self.pub_servo = self.create_publisher(Int32, 'servo_cmd_vel', 5)

                self.sub_right = self.create_subscription(Int8, 'IR_right', self.right_callback, 10)
                self.sub_left = self.create_subscription(Int8, 'IR_left', self.left_callback, 10)

                #self.front_right, self.front_left = 0, 0
                self.sonar_middle, self.sonar_right, self.sonar_left, self.d_left, self.d_right = 0, 0, 0, 0, 0                       

        def motor_callback(self):
                msg = String()
                if self.front_right == 1 and self.front_left == 1:
                         msg.data = 'forward'
                         self.pub.publish(msg)
                         self.get_logger().info('Publishing Move: "%s"' % msg.data)

                elif self.front_right == 0 and self.front_left == 1:
                         msg.data = 'left'
                         self.pub.publish(msg)
                         self.get_logger().info('Publishing Move: "%s"' % msg.data)

                elif self.front_right == 1 and self.front_left == 0:
                         msg.data = 'right'
                         self.pub.publish(msg)
                         self.get_logger().info('Publishing Move: "%s"' % msg.data)

                else:
                         msg.data = 'backward'
                         self.pub.publish(msg)
                         self.get_logger().info('Publishing Move: "%s"' % msg.data)
        def right_callback(self, msg):
                self.front_right = msg.data

        def left_callback(self, msg):
        	    self.front_left = msg.data
        

        #def left_callback(self, msg):
        #       self.front_left = msg.data

def main(args=None):
        rclpy.init(args=args)

        md = MotorDriver()

        rclpy.spin(md)

        md.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
        main()

