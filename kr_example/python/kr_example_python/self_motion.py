import rclpy
from rclpy.node import Node

from kr_msgs.msg import SelfMotion


class SelfMotionPublisher(Node):

    def __init__(self):
        super().__init__('self_motion_publisher')
        self.publisher_ = self.create_publisher(SelfMotion, "/kr/motion/self_motion", 10)
        timer_period = 0.002  # seconds
        self.timer = self.create_timer(timer_period, self.self_motion_callback)

    def self_motion_callback(self):
        msg = SelfMotion()

        msg.speed = 0.5

        print("PUBLISHING SELF MOTION \n")
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    self_motion_publisher = SelfMotionPublisher()
    rclpy.spin(self_motion_publisher)
    self_motion_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
