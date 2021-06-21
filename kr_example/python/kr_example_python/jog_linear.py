import rclpy
from rclpy.node import Node

from kr_msgs.msg import JogLinear


class JogLinearPublisher(Node):

    def __init__(self):
        super().__init__('jog_linear_publisher')
        self.publisher_ = self.create_publisher(JogLinear, "/kr/motion/jog_linear", 10)
        timer_period = 0.002  # seconds
        self.timer = self.create_timer(timer_period, self.jog_linear_callback)

    def jog_linear_callback(self):
        msg = JogLinear()
        vel = [100., 0., 0.]
        rot = [0., 0., 0.]

        msg.vel = vel
        msg.rot = rot

        print("PUBLISHING JOG LINEAR \n")
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    jog_linear_publisher = JogLinearPublisher()
    rclpy.spin(jog_linear_publisher)
    jog_linear_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
