import rclpy
from rclpy.node import Node

from kr_msgs.msg import FollowJoint


class FollowJointPublisher(Node):

    def __init__(self):
        super().__init__('follow_joint_publisher')
        self.publisher_ = self.create_publisher(FollowJoint, "/kr/motion/follow_joint", 10)
        timer_period = 5  # seconds
        self.temp = True
        self.timer = self.create_timer(timer_period, self.follow_joint_callback)

    def follow_joint_callback(self):
        msg = FollowJoint()
        config_1 = [0., 35., 9., 116., 0., 0., 0.]
        config_2 = [54., 35., 9., 116., 0., 0., 0.]
        if self.temp:
            msg.jsconf = config_1
        else:
            msg.jsconf = config_2

        msg.ttype = 1
        msg.tvalue = 3.
        msg.bpoint = 0
        msg.btype = 1
        msg.bvalue = 2.

        print("PUBLISHING FOLLOW JOINT \n")
        self.publisher_.publish(msg)
        self.temp = not self.temp


def main(args=None):
    rclpy.init(args=args)

    follow_joint_publisher = FollowJointPublisher()
    rclpy.spin(follow_joint_publisher)
    follow_joint_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
