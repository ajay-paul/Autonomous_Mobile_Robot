
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import String
import time
from std_msgs.msg import Int8
from std_msgs.msg import Float64
from std_msgs.msg import Int32
#from sensor_msgs import Range


IN1 = 12
IN2 = 13
ENA = 6
IN3 = 20
IN4 = 21
ENB = 26
DL = 19
DR = 16

SERVO = 27
TRIG = 17
ECHO = 5

class SubscriberMotor(Node):
        def __init__(self):
                super().__init__('motor_subscriber')
                #creating the subscriber
                self.sub = self.create_subscription(String, 'cmd_vel', self.motor_callback, 10)
                #self.subscriber = self.create_subscription(String, 'cmd_vel', self.motor_callback, 10)
                #self.sub_servo = self.create_subscription(Int32, 'servo_cmd_vel', self.servo_callback, 10)


                self.sensor_publisher_right = self.create_publisher(Int8, 'IR_right', 1)

                self.sensor_publisher_left = self.create_publisher(Int8, 'IR_left', 1)

                self.get_logger().info('Sensor enabled')

                timer_period = 0.5
                self.timer = self.create_timer(timer_period, self.timer_callback)


                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)

                GPIO.setup(IN1, GPIO.OUT) #left wheel forward
                GPIO.setup(IN2, GPIO.OUT) #left wheel backward
                GPIO.setup(ENA, GPIO.OUT) #enable left wheel
                GPIO.setup(IN3, GPIO.OUT) #right wheel forward
                GPIO.setup(IN4, GPIO.OUT) #right wheel backward
                GPIO.setup(ENB, GPIO.OUT) #enable right wheel

                GPIO.setup(DL, GPIO.IN, pull_up_down = GPIO.PUD_UP)
                GPIO.setup(DR, GPIO.IN, pull_up_down = GPIO.PUD_UP)

                self.PWMA = GPIO.PWM(ENA,500)
                self.PWMB = GPIO.PWM(ENB,500)
                self.PWMA.start(50)
                self.PWMB.start(50)
                
        def motor_callback(self, msg):
                if msg.data == 'forward':
                        GPIO.output(IN1, GPIO.HIGH)
                        GPIO.output(IN2, GPIO.LOW)
                        GPIO.output(ENA, GPIO.HIGH)

                        GPIO.output(IN3, GPIO.LOW)
                        GPIO.output(IN4, GPIO.HIGH)
                        GPIO.output(ENB, GPIO.HIGH)

                        self.get_logger().info('I heard: "%s"' % msg.data)


                elif msg.data == 'stop':
                        GPIO.output(IN1, GPIO.LOW)
                        GPIO.output(IN2, GPIO.LOW)
                        GPIO.output(ENA, GPIO.LOW)

                        GPIO.output(IN3, GPIO.LOW)
                        GPIO.output(IN4, GPIO.LOW)
                        GPIO.output(ENB, GPIO.LOW)

                        self.get_logger().info('I heard: "%s"' % msg.data)

                elif msg.data == 'left':
                        GPIO.output(IN1, GPIO.LOW)
                        GPIO.output(IN2, GPIO.HIGH)
                        GPIO.output(ENA, GPIO.HIGH)

                        GPIO.output(IN3, GPIO.LOW)
                        GPIO.output(IN4, GPIO.LOW)
                        GPIO.output(ENB, GPIO.LOW)

                        self.get_logger().info('I heard: "%s"' % msg.data)
                        
                elif msg.data == 'right':
                        GPIO.output(IN1, GPIO.HIGH)
                        GPIO.output(IN2, GPIO.HIGH)
                        GPIO.output(ENA, GPIO.HIGH)

                        GPIO.output(IN3, GPIO.HIGH)
                        GPIO.output(IN4, GPIO.LOW)
                        GPIO.output(ENB, GPIO.LOW)

                        self.get_logger().info('I heard: "%s"' % msg.data)

                else:

                     msg.data = 'backward'

                     GPIO.output(IN1, GPIO.HIGH)
                     GPIO.output(IN2, GPIO.LOW)
                     GPIO.output(ENA, GPIO.HIGH)
                     
                     GPIO.output(IN3, GPIO.LOW)
                     GPIO.output(IN4, GPIO.HIGH)
                     GPIO.output(ENB, GPIO.HIGH)


                     GPIO.output(IN1, GPIO.LOW)
                     GPIO.output(IN2, GPIO.HIGH)
                               
                     GPIO.output(IN3, GPIO.HIGH)
                     GPIO.output(IN4, GPIO.LOW)
                     self.get_logger().info('I heard: "%s"' % msg.data)

        def timer_callback(self):
                DR_status = GPIO.input(DR)
                print(type(DR_status))
                print(DR_status)
                msg_right = Int8()
                msg_right.data = DR_status
                self.sensor_publisher_right.publish(msg_right)
                self.get_logger().info('Publishing right: "%s"' % msg_right.data)

                DL_status = GPIO.input(DL)
                msg_left = Int8()
                msg_left.data = DL_status
                self.sensor_publisher_left.publish(msg_left)
                self.get_logger().info('Publishing left: "%s"' % msg_left.data)

def main(args=None):
        rclpy.init(args=args)

        sm = SubscriberMotor()

        rclpy.spin(sm)

        sm.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
        main()


#ros2 topic pub /cmd_vel std_msgs/msg/String "data: stop"
#ros2 run rplidar_ros rplidar_composition --ros-args -p serial_port:=/dev/ttyUSB0 -p frame_id:=laser_frame -p angle_compensate:=true -p scan_mode:=Standard
#ros2 launch motortest motortest_launch_file.launch.py
#ros2 launch motor_publisher motor_publisher_launch_file.launch.py
#ros2 run odometry_estimator odometry_estimator
#ros2 run f249driver f249driver
#ros2 topic pub /map nav_msgs/msg/OccupancyGrid "{header: {frame_id: map}, info: {resolution: 0.05000000074505806, width: 122,height: 104}}"
#ros2 topic pub /tf tf2_msgs/msg/TFMessage "{transforms: {header: frame_id: odom, child_frame_id: base_footprint}}"
#ros2 topic pub /tf tf2_msgs/msg/TFMessage "{transforms: {child_frame_id: base_footprint9}}"
#ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
#ros2 launch cartographer_slam cartographer.launch.py
#ros2 run tf2_ros static_transform_publisher 0.1 0 0.2 0 0 0 base_link base_laser