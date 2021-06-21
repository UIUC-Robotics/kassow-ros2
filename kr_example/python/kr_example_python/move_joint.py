import rclpy
from rclpy.node import Node

from kr_msgs.srv import MoveJoint


class MoveJointPublisher(Node):

    def __init__(self):
        super().__init__('move_joint_publisher')
        self.cli = self.create_client(MoveJoint, '/kr/motion/move_joint')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MoveJoint.Request()

    def send_request(self):
        config_1 = [0., 35., 9., 116., 0., 0., 0.]

        self.req.jsconf = config_1

        self.req.ttype = 1
        self.req.tvalue = 5.
        self.req.bpoint = 0
        self.req.btype = 1
        self.req.bvalue = 2.
        self.req.sync = 0.
        self.req.chaining = 1

        print("PUBLISHING MOVE JOINT \n")

        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    client = MoveJointPublisher()
    client.send_request()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                client.get_logger().info('Successfull move joint request.')
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
