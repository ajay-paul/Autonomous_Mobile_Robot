import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import String
import time
#from sensor_msgs import Range


IN1 = 12
IN2 = 13
ENA = 6
IN3 = 20
IN4 = 21
ENB = 26

class Motor_Controll(Node):
        def __init__(self):
                super().__init__('Motors')
                #creating the subscriber
                self.sub = self.create_subscription(String, 'cmd_vel', self.motor_callback, 10)

                #self.get_logger().info('Sensor enabled')

                #timer_period = 0.5
                #self.timer = self.create_timer(timer_period, self.timer_callback)


                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)

                GPIO.setup(IN1, GPIO.OUT) #left wheel forward
                GPIO.setup(IN2, GPIO.OUT) #left wheel backward
                GPIO.setup(ENA, GPIO.OUT) #enable left wheel
                GPIO.setup(IN3, GPIO.OUT) #right wheel forward
                GPIO.setup(IN4, GPIO.OUT) #right wheel backward
                GPIO.setup(ENB, GPIO.OUT) #enable right wheel

                self.PWMA = GPIO.PWM(ENA,500)
                self.PWMB = GPIO.PWM(ENB,500)
                self.PWMA.start(100)
                self.PWMB.start(100)



        def motor_callback(self, msg):
                if msg.data == 'forward':
                        GPIO.output(IN1, GPIO.HIGH)
                        GPIO.output(IN2, GPIO.LOW)
                        #GPIO.output(ENA, GPIO.HIGH)

                        GPIO.output(IN3, GPIO.LOW)
                        GPIO.output(IN4, GPIO.HIGH)
                        #GPIO.output(ENB, GPIO.HIGH)

                        self.get_logger().info('I heard: "%s"' % msg.data)


                elif msg.data == 'stop':
                        GPIO.output(IN1, GPIO.LOW)
                        GPIO.output(IN2, GPIO.LOW)
                        #GPIO.output(ENA, GPIO.LOW)

                        GPIO.output(IN3, GPIO.LOW)
                        GPIO.output(IN4, GPIO.LOW)
                        #GPIO.output(ENB, GPIO.LOW)

                        self.get_logger().info('I heard: "%s"' % msg.data)


                elif msg.data == 'left':
                        GPIO.output(IN1, GPIO.LOW)
                        GPIO.output(IN2, GPIO.HIGH)
                        #GPIO.output(ENA, GPIO.HIGH)

                        GPIO.output(IN3, GPIO.LOW)
                        GPIO.output(IN4, GPIO.LOW)
                        #GPIO.output(ENB, GPIO.LOW)

                        self.get_logger().info('I heard: "%s"' % msg.data)

                elif msg.data == 'right':
                        GPIO.output(IN1, GPIO.LOW)
                        GPIO.output(IN2, GPIO.LOW)
                        #GPIO.output(ENA, GPIO.LOW)

                        GPIO.output(IN3, GPIO.HIGH)
                        GPIO.output(IN4, GPIO.LOW)
                        #GPIO.output(ENB, GPIO.HIGH)

                        self.get_logger().info('I heard: "%s"' % msg.data)

                else:
                        msg.data = 'backward'
                        GPIO.output(IN1, GPIO.LOW)
                        GPIO.output(IN2, GPIO.HIGH)
                       # GPIO.output(ENA, GPIO.HIGH)

                        GPIO.output(IN3, GPIO.HIGH)
                        GPIO.output(IN4, GPIO.LOW)
                       # GPIO.output(ENB, GPIO.HIGH)

                        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
        rclpy.init(args=args)

        sm = Motor_Controll()

        rclpy.spin(sm)

        sm.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
        main()